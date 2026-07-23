#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <cmath>
#include "config.h"   // Zentrale Konfiguration (Pins, WebGUI-Schalter, Defaults)
#if ENABLE_OLED_MENU
#include <Wire.h>
#include <U8g2lib.h>
#endif
#if !ENABLE_DAC2
#include "driver/i2s.h"
#endif
#include "sounds.h"   // Optionale Sprach-/Audio-Clips (WAV im Flash)

bool bt_started = false;                    // Bluetooth erst nach WiFi-Timeout starten

// Koppel-/Entkoppel-Sound: wird in der BT-Callback nur als Flag gesetzt und
// im loop() abgespielt (nicht im Callback-Kontext, das wäre unsicher).
// 0 = nichts, 1 = verbunden (Koppel), 2 = getrennt (Entkoppel)
volatile int pending_sound = 0;

#if ENABLE_BT_CONFIG
// SPP-Konfigurationskanal: empfängt Textbefehle vom Handy
BluetoothSerial bt_serial;
static char bt_rx_buf[80];   // Zeilenpuffer (max. 79 Zeichen pro Befehl)
static uint8_t bt_rx_len = 0;
#endif

// Audio-Objekte als Pointer – Konstruktoren allozieren Heap,
// dürfen daher NICHT als globale Objekte initialisiert werden (Static Init Fiasco)
I2SStream*          i2s_tops  = nullptr;
I2SStream*          i2s_sub   = nullptr;
MultiOutput*        multi_out = nullptr;
BluetoothA2DPSink*  a2dp_sink = nullptr;
CallbackStream*     cb_tops   = nullptr;
CallbackStream*     cb_sub    = nullptr;

#if !ENABLE_DAC2
// PCM1808-Line-In auf den ehemaligen DAC2-Pins (I2S Port 1, RX)
static bool pcm1808_ready = false;
static int32_t pcm1808_rx32[512];
static int16_t pcm1808_mix16[512];
#if ENABLE_BT_FALLBACK_WITH_AUX
static volatile bool aux_priority_active = false;
static volatile bool bt_connected = false;
static uint32_t aux_last_signal_ms = 0;
static float aux_mix = 0.0f; // 0 = nur BT, 1 = nur AUX
#endif
#endif

Preferences g_prefs;
bool g_prefs_ready = false;

enum ProfileId : uint8_t {
    PROFILE_ID_MTH30_TOP = PROFILE_MTH30_TOP,
    PROFILE_ID_Z2300_SAT = PROFILE_Z2300_SAT,
    PROFILE_ID_USER = PROFILE_USER,
};

uint8_t active_profile_id = DEFAULT_PROFILE_ID;
bool profile_bootstrapped = false;

// Globale DSP-Filtervariablen
float sub_subsonic = DEFAULT_SUB_SUBSONIC;
float sub_lowpass  = DEFAULT_SUB_LOWPASS;
float tops_highpass = DEFAULT_TOPS_HIGHPASS;
const float sample_rate = SAMPLE_RATE;

// Tops-Delay (zur Laufzeit-Synchronisation mit dem Horn/Sub)
float tops_delay_ms = DEFAULT_TOPS_DELAY_MS;
const int TOPS_DELAY_MAX = 1400;              // ~31.7 ms @ 44.1 kHz
// Delay-Ring-Buffer DAC 1 und DAC 2 (je L + R, unabhängig voneinander)
float dac1_dly_L[TOPS_DELAY_MAX] = {0};
float dac1_dly_R[TOPS_DELAY_MAX] = {0};
int   dac1_dly_idx = 0;
float dac2_dly_L[TOPS_DELAY_MAX] = {0};
float dac2_dly_R[TOPS_DELAY_MAX] = {0};
int   dac2_dly_idx = 0;
volatile int tops_delay_samples = 0;

// Rohsignal-Sicherung: process_dac1 speichert hier die unmodifizierten Eingangs-
// Samples, damit process_dac2 unabhängig davon auf die Originaldaten zugreift.
static int16_t s_raw_buf[1024];   // 512 Stereo-Frames = 2 KB
static size_t  s_raw_len = 0;

// Lautstärke pro Kanal (0.0 bis 1.0) – je DAC-Ausgang einzeln einstellbar
float vol_dac1_l = DEFAULT_VOL_DAC1_L;  // DAC1 L
float vol_dac1_r = DEFAULT_VOL_DAC1_R;  // DAC1 R
float vol_dac2_l = DEFAULT_VOL_DAC2_L;  // DAC2 L
float vol_dac2_r = DEFAULT_VOL_DAC2_R;  // DAC2 R
float master_gain = STARTUP_MASTER_VOL_PCT / 100.0f;  // Gesamtlautstaerke (0.0 .. 1.0)
float master_volume_pct_runtime = STARTUP_MASTER_VOL_PCT;

// Poti-Smoothing (exponentieller MA, alpha=0.1 = sanfte Glättung)
static float poti_l_smooth = 0.5f;
static float poti_r_smooth = 0.5f;
const float POTI_ALPHA = 0.1f;  // Smoothing-Faktor (0.0 = stark geglättet, 1.0 = keine Glättung)

// Limiter-Parameter (zur Laufzeit über die Weboberfläche einstellbar).
// Startwerte + feste Attack-Zeiten kommen aus config.h.
float lim_tops_thresh = DEFAULT_LIM_TOPS_THRESH;   // Ziel-Spitzenwert Tops
float lim_tops_rel    = DEFAULT_LIM_TOPS_REL;      // Release Tops (ms)
float lim_sub_thresh  = DEFAULT_LIM_SUB_THRESH;    // Ziel-Spitzenwert Sub
float lim_sub_rel     = DEFAULT_LIM_SUB_REL;       // Release Sub (ms)

// Filter-Instanzen für die Kanäle
// Alle Trennfilter als Linkwitz-Riley konfigurierbar (LR12/LR24/LR48) =
// Kaskade aus bis zu 4 Biquads pro Kanal.
LowPassFilter<float> sub_lp_L(sub_lowpass, sample_rate);
LowPassFilter<float> sub_lp_R(sub_lowpass, sample_rate);
LowPassFilter<float> sub_lp_L2(sub_lowpass, sample_rate);
LowPassFilter<float> sub_lp_R2(sub_lowpass, sample_rate);
LowPassFilter<float> sub_lp_L3(sub_lowpass, sample_rate);
LowPassFilter<float> sub_lp_R3(sub_lowpass, sample_rate);
LowPassFilter<float> sub_lp_L4(sub_lowpass, sample_rate);
LowPassFilter<float> sub_lp_R4(sub_lowpass, sample_rate);
// Subsonic-Filter: konfigurierbar (Horn-Schutz)
HighPassFilter<float> sub_sub_L(sub_subsonic, sample_rate);
HighPassFilter<float> sub_sub_R(sub_subsonic, sample_rate);
HighPassFilter<float> sub_sub_L2(sub_subsonic, sample_rate);
HighPassFilter<float> sub_sub_R2(sub_subsonic, sample_rate);
HighPassFilter<float> sub_sub_L3(sub_subsonic, sample_rate);
HighPassFilter<float> sub_sub_R3(sub_subsonic, sample_rate);
HighPassFilter<float> sub_sub_L4(sub_subsonic, sample_rate);
HighPassFilter<float> sub_sub_R4(sub_subsonic, sample_rate);

HighPassFilter<float> tops_hp_L(tops_highpass, sample_rate);
HighPassFilter<float> tops_hp_R(tops_highpass, sample_rate);
HighPassFilter<float> tops_hp_L2(tops_highpass, sample_rate);
HighPassFilter<float> tops_hp_R2(tops_highpass, sample_rate);
HighPassFilter<float> tops_hp_L3(tops_highpass, sample_rate);
HighPassFilter<float> tops_hp_R3(tops_highpass, sample_rate);
HighPassFilter<float> tops_hp_L4(tops_highpass, sample_rate);
HighPassFilter<float> tops_hp_R4(tops_highpass, sample_rate);


// --- Peak-Limiter ----------------------------------------------------------
// Sanfter Pegelbegrenzer: senkt die Verstärkung mit Attack/Release, bevor das
// Signal die Clip-Grenze erreicht. Schützt das Horn/den Sub vor Übersteuerung
// und vermeidet harte Clipping-Verzerrung.
struct PeakLimiter {
    float gain = 1.0f;
    float threshold = 32000.0f;   // Ziel-Spitzenwert (Headroom unter 32767)
    float attack_coeff = 0.0f;    // schnelles Absenken bei Pegelspitzen
    float release_coeff = 0.0f;   // langsames Zurückfahren der Absenkung

    void begin(float thresh, float sr, float attack_ms, float release_ms) {
        threshold     = thresh;
        attack_coeff  = expf(-1.0f / (attack_ms  * 0.001f * sr));
        release_coeff = expf(-1.0f / (release_ms * 0.001f * sr));
    }

    // peak = Betrag der aktuellen Sample-Spitze; liefert den anzuwendenden Gain
    float compute(float peak) {
        float desired = 1.0f;
        if (peak > threshold && peak > 0.0f) desired = threshold / peak;
        float coeff = (desired < gain) ? attack_coeff : release_coeff;
        gain = coeff * gain + (1.0f - coeff) * desired;
        return gain;
    }
};

PeakLimiter lim_tops;   // Tops (stereo, gemeinsamer Gain für L/R)
PeakLimiter lim_sub;    // Sub  (mono)

// Helper: Q-Faktor basierend auf Filter-Typ
static float get_q_factor() {
    switch (FILTER_TYPE) {
        case 0: return 0.7071f;   // Butterworth (standard)
        case 1: return 0.5773f;   // Bessel (minimum phase)
        case 2: return 1.3065f;   // Chebyshev 0.5dB ripple
        default: return 0.7071f;
    }
}

// Helper: Anzahl der Filter-Stufen basierend auf Ordnung
static int get_filter_stages() {
    // FILTER_ORDER: 0=LR12 (1 Biquad), 1=LR24 (2 Biquads), 2=LR48 (4 Biquads)
    return (FILTER_ORDER + 1);
}

// Aktualisiert alle Filter mit korrektem Q-Faktor und Frequenzen
static void update_all_filters(float q_factor = -1.0f) {
    if (q_factor < 0) q_factor = get_q_factor();
    int stages = get_filter_stages();

    // Sub Subsonic (HighPass)
    sub_sub_L.begin(sub_subsonic, sample_rate, q_factor);
    sub_sub_R.begin(sub_subsonic, sample_rate, q_factor);
    if (stages >= 2) {
        sub_sub_L2.begin(sub_subsonic, sample_rate, q_factor);
        sub_sub_R2.begin(sub_subsonic, sample_rate, q_factor);
    }
    if (stages >= 4) {
        sub_sub_L3.begin(sub_subsonic, sample_rate, q_factor);
        sub_sub_R3.begin(sub_subsonic, sample_rate, q_factor);
        sub_sub_L4.begin(sub_subsonic, sample_rate, q_factor);
        sub_sub_R4.begin(sub_subsonic, sample_rate, q_factor);
    }

    // Sub LowPass
    sub_lp_L.begin(sub_lowpass, sample_rate, q_factor);
    sub_lp_R.begin(sub_lowpass, sample_rate, q_factor);
    if (stages >= 2) {
        sub_lp_L2.begin(sub_lowpass, sample_rate, q_factor);
        sub_lp_R2.begin(sub_lowpass, sample_rate, q_factor);
    }
    if (stages >= 4) {
        sub_lp_L3.begin(sub_lowpass, sample_rate, q_factor);
        sub_lp_R3.begin(sub_lowpass, sample_rate, q_factor);
        sub_lp_L4.begin(sub_lowpass, sample_rate, q_factor);
        sub_lp_R4.begin(sub_lowpass, sample_rate, q_factor);
    }

    // Tops HighPass
    tops_hp_L.begin(tops_highpass, sample_rate, q_factor);
    tops_hp_R.begin(tops_highpass, sample_rate, q_factor);
    if (stages >= 2) {
        tops_hp_L2.begin(tops_highpass, sample_rate, q_factor);
        tops_hp_R2.begin(tops_highpass, sample_rate, q_factor);
    }
    if (stages >= 4) {
        tops_hp_L3.begin(tops_highpass, sample_rate, q_factor);
        tops_hp_R3.begin(tops_highpass, sample_rate, q_factor);
        tops_hp_L4.begin(tops_highpass, sample_rate, q_factor);
        tops_hp_R4.begin(tops_highpass, sample_rate, q_factor);
    }
}

void apply_param(const String& key, float val, bool persist_as_user = true);

// Berechnet den float-Ausgabewert für einen konfigurierten DAC-Kanal.
// raw_l / raw_r = UNVERARBEITETE 16-Bit-Eingangswerte als float.
// Jede Filter-Instanz darf pro Sample-Frame nur einmal aufgerufen werden –
// denselben Modus also nie auf zwei Kanälen gleichzeitig konfigurieren.
static float compute_ch(int mode, float raw_l, float raw_r) {
    int stages = get_filter_stages();

    switch (mode) {
        case DAC_CH_SUB: {
            float m = 0.5f * (raw_l + raw_r);
            // Subsonic cascade
            m = sub_sub_L.process(m);
            if (stages >= 2) m = sub_sub_L2.process(m);
            if (stages >= 4) {
                m = sub_sub_L3.process(m);
                m = sub_sub_L4.process(m);
            }
            // LowPass cascade
            m = sub_lp_L.process(m);
            if (stages >= 2) m = sub_lp_L2.process(m);
            if (stages >= 4) {
                m = sub_lp_L3.process(m);
                m = sub_lp_L4.process(m);
            }
            return m;
        }
        case DAC_CH_MONO:
            return 0.5f * (raw_l + raw_r);
        case DAC_CH_TOPS_L: {
            float l = tops_hp_L.process(raw_l);
            if (stages >= 2) l = tops_hp_L2.process(l);
            if (stages >= 4) {
                l = tops_hp_L3.process(l);
                l = tops_hp_L4.process(l);
            }
            return l;
        }
        case DAC_CH_TOPS_R: {
            float r = tops_hp_R.process(raw_r);
            if (stages >= 2) r = tops_hp_R2.process(r);
            if (stages >= 4) {
                r = tops_hp_R3.process(r);
                r = tops_hp_R4.process(r);
            }
            return r;
        }
        case DAC_CH_LEFT:
            return raw_l;
        case DAC_CH_RIGHT:
            return raw_r;
        default:            // DAC_CH_OFF
            return 0.0f;
    }
}

// Gibt an, ob ein Kanal-Modus die Tops-Delay-Line nutzen soll
// (Synchronisation Tops/Mono mit dem Sub).
static inline bool ch_uses_delay(int mode) {
    return (mode == DAC_CH_MONO || mode == DAC_CH_TOPS_L || mode == DAC_CH_TOPS_R);
}

// DAC 1 – Verarbeitung (Kanal-Belegung per config.h: DAC1_L_MODE / DAC1_R_MODE)
size_t process_dac1(uint8_t *data, size_t len) {
#if !ENABLE_DAC2 && ENABLE_BT_FALLBACK_WITH_AUX
    float bt_mix = 1.0f - aux_mix;
    if (bt_mix < 0.001f) {
        memset(data, 0, len);
        return len;
    }
#endif

    // Rohsignal für process_dac2 sichern – vor der In-Place-Modifikation
    s_raw_len = (len <= sizeof(s_raw_buf)) ? len : sizeof(s_raw_buf);
    memcpy(s_raw_buf, data, s_raw_len);

    int16_t *samples = (int16_t*)data;
    for (int i = 0; i < (int)(len / 2); i += 2) {
        float raw_l = (float)samples[i];
        float raw_r = (float)samples[i + 1];

        float l = compute_ch(DAC1_L_MODE, raw_l, raw_r) * vol_dac1_l;
        float r = compute_ch(DAC1_R_MODE, raw_l, raw_r) * vol_dac1_r;
        l *= master_gain;
        r *= master_gain;
    #if !ENABLE_DAC2 && ENABLE_BT_FALLBACK_WITH_AUX
        l *= bt_mix;
        r *= bt_mix;
    #endif

        // Delay-Line DAC 1 (Synchronisation mit Sub/Horn)
        dac1_dly_L[dac1_dly_idx] = l;
        dac1_dly_R[dac1_dly_idx] = r;
        int rd = dac1_dly_idx - tops_delay_samples;
        if (rd < 0) rd += TOPS_DELAY_MAX;
        if (ch_uses_delay(DAC1_L_MODE)) l = dac1_dly_L[rd];
        if (ch_uses_delay(DAC1_R_MODE)) r = dac1_dly_R[rd];
        dac1_dly_idx = (dac1_dly_idx + 1) % TOPS_DELAY_MAX;

        // Limiter DAC 1
        float g = lim_tops.compute(fmaxf(fabsf(l), fabsf(r)));
        l *= g;
        r *= g;

        samples[i]     = (int16_t)constrain(l, -32768, 32767);
        samples[i + 1] = (int16_t)constrain(r, -32768, 32767);
    }
    return len;
}

// DAC 2 – Verarbeitung (Kanal-Belegung per config.h: DAC2_L_MODE / DAC2_R_MODE)
size_t process_dac2(uint8_t *data, size_t len) {
    // Rohsignal aus dem gesicherten Puffer lesen (unmodifiziert durch process_dac1)
    const int16_t *raw = (s_raw_len >= len) ? s_raw_buf : (const int16_t*)data;

    int16_t *samples = (int16_t*)data;
    for (int i = 0; i < (int)(len / 2); i += 2) {
        float raw_l = (float)raw[i];
        float raw_r = (float)raw[i + 1];

        float l = compute_ch(DAC2_L_MODE, raw_l, raw_r) * vol_dac2_l;
        float r = compute_ch(DAC2_R_MODE, raw_l, raw_r) * vol_dac2_r;
        l *= master_gain;
        r *= master_gain;

        // Delay-Line DAC 2 (eigener Ringpuffer, unabhängig von DAC 1)
        dac2_dly_L[dac2_dly_idx] = l;
        dac2_dly_R[dac2_dly_idx] = r;
        int rd = dac2_dly_idx - tops_delay_samples;
        if (rd < 0) rd += TOPS_DELAY_MAX;
        if (ch_uses_delay(DAC2_L_MODE)) l = dac2_dly_L[rd];
        if (ch_uses_delay(DAC2_R_MODE)) r = dac2_dly_R[rd];
        dac2_dly_idx = (dac2_dly_idx + 1) % TOPS_DELAY_MAX;

        // Limiter DAC 2
        float g = lim_sub.compute(fmaxf(fabsf(l), fabsf(r)));
        l *= g;
        r *= g;

        samples[i]     = (int16_t)constrain(l, -32768, 32767);
        samples[i + 1] = (int16_t)constrain(r, -32768, 32767);
    }
    return len;
}

#if !ENABLE_DAC2
// PCM1808 (Stereo) -> Mono-Summe, danach bestehende DSP-Logik fuer DAC1 nutzen.
static inline void sum_stereo_to_mono_inplace(int16_t *samples, int count) {
    for (int i = 0; i + 1 < count; i += 2) {
        int32_t m = ((int32_t)samples[i] + (int32_t)samples[i + 1]) / 2;
        if (m > 32767) m = 32767;
        if (m < -32768) m = -32768;
        int16_t mono = (int16_t)m;
        samples[i] = mono;
        samples[i + 1] = mono;
    }
}

static void setup_pcm1808_input() {
    i2s_config_t cfg = {};
    cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    cfg.sample_rate = SAMPLE_RATE;
    cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
    cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count = 8;
    cfg.dma_buf_len = 256;
    cfg.use_apll = false;
    cfg.tx_desc_auto_clear = false;
    cfg.fixed_mclk = 0;

    i2s_pin_config_t pins = {};
    pins.bck_io_num = SUB_PIN_BCK;
    pins.ws_io_num = SUB_PIN_WS;
    pins.data_out_num = I2S_PIN_NO_CHANGE;
    pins.data_in_num = SUB_PIN_DATA;

    if (i2s_driver_install(I2S_NUM_1, &cfg, 0, nullptr) != ESP_OK) {
        Serial.println("[PCM1808] i2s_driver_install fehlgeschlagen");
        return;
    }
    if (i2s_set_pin(I2S_NUM_1, &pins) != ESP_OK) {
        Serial.println("[PCM1808] i2s_set_pin fehlgeschlagen");
        i2s_driver_uninstall(I2S_NUM_1);
        return;
    }
    i2s_zero_dma_buffer(I2S_NUM_1);
    pcm1808_ready = true;
    Serial.println("[PCM1808] I2S-Input aktiv (Stereo -> Mono)");
}

static void service_pcm1808_input() {
    if (!pcm1808_ready || !i2s_tops) return;
    size_t bytes_read = 0;
    esp_err_t err = i2s_read(I2S_NUM_1, pcm1808_rx32, sizeof(pcm1808_rx32), &bytes_read, 0);
    if (err != ESP_OK || bytes_read < 4) return;

    int sample_count_32 = (int)(bytes_read / sizeof(int32_t));
    int sample_count_16 = sample_count_32;
    if (sample_count_16 > (int)(sizeof(pcm1808_mix16) / sizeof(pcm1808_mix16[0]))) {
        sample_count_16 = (int)(sizeof(pcm1808_mix16) / sizeof(pcm1808_mix16[0]));
    }

    int peak = 0;
    for (int i = 0; i < sample_count_16; i++) {
        // PCM1808 liefert 24-bit in 32-bit Frames -> auf 16-bit skalieren.
        int16_t s = (int16_t)(pcm1808_rx32[i] >> 16);
        pcm1808_mix16[i] = s;
        int v = abs((int)s);
        if (v > peak) peak = v;
    }

#if ENABLE_BT_FALLBACK_WITH_AUX
    uint32_t now = millis();
    if (bt_connected) {
        aux_priority_active = false;
    } else if (peak >= AUX_ACTIVITY_ON_THRESHOLD) {
        aux_last_signal_ms = now;
        aux_priority_active = true;
    } else if (peak <= AUX_ACTIVITY_OFF_THRESHOLD && (now - aux_last_signal_ms) > (unsigned long)AUX_PRIORITY_HOLD_MS) {
        aux_priority_active = false;
    }

    float target = aux_priority_active ? 1.0f : 0.0f;
    if (aux_mix < target) {
        aux_mix += AUX_BT_XFADE_STEP;
        if (aux_mix > target) aux_mix = target;
    } else if (aux_mix > target) {
        aux_mix -= AUX_BT_XFADE_STEP;
        if (aux_mix < target) aux_mix = target;
    }
    if (aux_mix < 0.001f) return;
#endif

    // Erst summieren, dann wie gewohnt mit den bestehenden DSP-Einstellungen verarbeiten.
    sum_stereo_to_mono_inplace(pcm1808_mix16, sample_count_16);

    size_t bytes16 = (size_t)sample_count_16 * sizeof(int16_t);
    process_dac1((uint8_t*)pcm1808_mix16, bytes16);

#if ENABLE_BT_FALLBACK_WITH_AUX
    if (aux_mix < 0.999f) {
        for (int i = 0; i < sample_count_16; i++) {
            float s = (float)pcm1808_mix16[i] * aux_mix;
            pcm1808_mix16[i] = (int16_t)constrain(s, -32768.0f, 32767.0f);
        }
    }
#endif

    i2s_tops->write((uint8_t*)pcm1808_mix16, bytes16);
}
#endif

// CallbackStream-Wrapper werden in setup() erstellt (siehe unten)

// Kurzen Sinus-Beep auf beide DACs ausgeben (blockierend, nur für Signal-Töne)
void play_beep(float freq, int duration_ms, float amp = SND_VOLUME) {
    if (!i2s_tops) return;
    const int sr = (int)sample_rate;
    const int total = sr * duration_ms / 1000;
    const int CHUNK = 128;
    int16_t buf[CHUNK * 2];
    float phase = 0.0;
    const float dphi = 2.0 * PI * freq / sr;
    int done = 0;
    while (done < total) {
        int n = min(CHUNK, total - done);
        for (int i = 0; i < n; i++) {
            // sanftes Ein-/Ausblenden gegen Knackser
            float env = 1.0;
            int pos = done + i;
            int fade = sr / 200; // ~5 ms
            if (pos < fade) env = (float)pos / fade;
            else if (pos > total - fade) env = (float)(total - pos) / fade;
            int16_t s = (int16_t)(sinf(phase) * 32767.0 * amp * env);
            buf[i * 2] = s; buf[i * 2 + 1] = s;
            phase += dphi;
            if (phase > 2.0 * PI) phase -= 2.0 * PI;
        }
        size_t bytes = n * 2 * sizeof(int16_t);
        i2s_tops->write((const uint8_t*)buf, bytes);
        if (i2s_sub) i2s_sub->write((const uint8_t*)buf, bytes);
        done += n;
    }
}

// --- Eigene Melodien -------------------------------------------------------
// Eine Note besteht aus Frequenz (Hz) und Dauer (ms). Frequenz 0 = Pause.
// Hier kannst du beliebige eigene Melodien zusammenstellen.
struct Note { float freq; int ms; };

// Ein paar Notennamen als Hilfe (Oktave 5)
#define C5 523.25f
#define D5 587.33f
#define E5 659.25f
#define F5 698.46f
#define G5 783.99f
#define A5 880.00f
#define H5 987.77f
#define C6 1046.50f
#define REST 0.0f

// KOPPEL-Melodie (Verbindung hergestellt) – nach Wunsch anpassen
const Note melody_connect[] = {
    { E5, 90 }, { G5, 90 }, { C6, 150 },
};
// ENTKOPPEL-Melodie (Verbindung getrennt) – nach Wunsch anpassen
const Note melody_disconnect[] = {
    { C6, 90 }, { G5, 90 }, { E5, 150 },
};

// Melodie abspielen (Note-Array + Anzahl)
void play_melody(const Note* notes, int count) {
    for (int i = 0; i < count; i++) {
        if (notes[i].freq <= 0.0f) delay(notes[i].ms);          // Pause
        else play_beep(notes[i].freq, notes[i].ms);
    }
}

void play_connect_sound()    { play_melody(melody_connect,    sizeof(melody_connect)    / sizeof(Note)); }
void play_disconnect_sound() { play_melody(melody_disconnect, sizeof(melody_disconnect) / sizeof(Note)); }

// PCM-Clip abspielen: 16-bit MONO @ 44.1 kHz, wird auf beide DACs (L=R) gelegt.
// Die Sample-Daten liegen im Flash (PROGMEM/.rodata) und sind auf dem ESP32
// direkt lesbar (Flash ist memory-mapped).
void play_pcm_mono(const int16_t* data, size_t samples, float amp = SND_VOLUME) {
    if (!i2s_tops || !data || samples == 0) return;
    const int CHUNK = 128;
    int16_t buf[CHUNK * 2];
    size_t done = 0;
    while (done < samples) {
        int n = (int)min((size_t)CHUNK, samples - done);
        for (int i = 0; i < n; i++) {
            int16_t s = data[done + i];
            if (amp != 1.0f) s = (int16_t)(s * amp);
            buf[i * 2] = s; buf[i * 2 + 1] = s;
        }
        size_t bytes = n * 2 * sizeof(int16_t);
        i2s_tops->write((const uint8_t*)buf, bytes);
        if (i2s_sub) i2s_sub->write((const uint8_t*)buf, bytes);
        done += n;
    }
}

static void set_master_volume_pct_runtime(float pct, bool persist) {
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    master_volume_pct_runtime = pct;
    master_gain = pct / 100.0f;
    DEBUG_LOG("Master Volume: %.0f%%", pct);
    if (persist && g_prefs_ready) g_prefs.putFloat("master", master_volume_pct_runtime);
}

static void save_active_profile_id() {
    if (g_prefs_ready) g_prefs.putUChar("profile", active_profile_id);
}

static void save_user_profile_to_nvs() {
    if (!g_prefs_ready) return;
    g_prefs.putFloat("u_sub_sb", sub_subsonic);
    g_prefs.putFloat("u_sub_lp", sub_lowpass);
    g_prefs.putFloat("u_tops_hp", tops_highpass);
    g_prefs.putFloat("u_t_dly", tops_delay_ms);
    g_prefs.putFloat("u_v_d1l", vol_dac1_l * 100.0f);
    g_prefs.putFloat("u_v_d1r", vol_dac1_r * 100.0f);
    g_prefs.putFloat("u_v_d2l", vol_dac2_l * 100.0f);
    g_prefs.putFloat("u_v_d2r", vol_dac2_r * 100.0f);
    g_prefs.putFloat("u_lim_s_th", lim_sub_thresh / 32767.0f * 100.0f);
    g_prefs.putFloat("u_lim_s_rl", lim_sub_rel);
    g_prefs.putFloat("u_lim_t_th", lim_tops_thresh / 32767.0f * 100.0f);
    g_prefs.putFloat("u_lim_t_rl", lim_tops_rel);
    g_prefs.putFloat("u_master", master_volume_pct_runtime);
}

static bool load_user_profile_from_nvs() {
    if (!g_prefs_ready || !g_prefs.isKey("u_sub_sb")) return false;
    apply_param("sub_sb", g_prefs.getFloat("u_sub_sb", DEFAULT_SUB_SUBSONIC), false);
    apply_param("sub_lp", g_prefs.getFloat("u_sub_lp", DEFAULT_SUB_LOWPASS), false);
    apply_param("tops_hp", g_prefs.getFloat("u_tops_hp", DEFAULT_TOPS_HIGHPASS), false);
    apply_param("t_dly", g_prefs.getFloat("u_t_dly", DEFAULT_TOPS_DELAY_MS), false);
    apply_param("v_dac1_l", g_prefs.getFloat("u_v_d1l", DEFAULT_VOL_DAC1_L * 100.0f), false);
    apply_param("v_dac1_r", g_prefs.getFloat("u_v_d1r", DEFAULT_VOL_DAC1_R * 100.0f), false);
    apply_param("v_dac2_l", g_prefs.getFloat("u_v_d2l", DEFAULT_VOL_DAC2_L * 100.0f), false);
    apply_param("v_dac2_r", g_prefs.getFloat("u_v_d2r", DEFAULT_VOL_DAC2_R * 100.0f), false);
    apply_param("lim_s_th", g_prefs.getFloat("u_lim_s_th", DEFAULT_LIM_SUB_THRESH / 32767.0f * 100.0f), false);
    apply_param("lim_s_rl", g_prefs.getFloat("u_lim_s_rl", DEFAULT_LIM_SUB_REL), false);
    apply_param("lim_t_th", g_prefs.getFloat("u_lim_t_th", DEFAULT_LIM_TOPS_THRESH / 32767.0f * 100.0f), false);
    apply_param("lim_t_rl", g_prefs.getFloat("u_lim_t_rl", DEFAULT_LIM_TOPS_REL), false);
    set_master_volume_pct_runtime(g_prefs.getFloat("u_master", STARTUP_MASTER_VOL_PCT), false);
    return true;
}

static void apply_profile(uint8_t profile_id) {
    if (profile_id > PROFILE_ID_USER) profile_id = DEFAULT_PROFILE_ID;
    active_profile_id = profile_id;

    if (profile_id == PROFILE_ID_MTH30_TOP) {
        apply_param("sub_sb", P_MTH30_SUB_SB, false);
        apply_param("sub_lp", P_MTH30_SUB_LP, false);
        apply_param("tops_hp", P_MTH30_TOPS_HP, false);
        apply_param("t_dly", P_MTH30_T_DLY, false);
        apply_param("v_dac1_l", P_MTH30_V_DAC1_L, false);
        apply_param("v_dac1_r", P_MTH30_V_DAC1_R, false);
        apply_param("v_dac2_l", P_MTH30_V_DAC2_L, false);
        apply_param("v_dac2_r", P_MTH30_V_DAC2_R, false);
        apply_param("lim_s_th", P_MTH30_LIM_S_TH, false);
        apply_param("lim_s_rl", P_MTH30_LIM_S_RL, false);
        apply_param("lim_t_th", P_MTH30_LIM_T_TH, false);
        apply_param("lim_t_rl", P_MTH30_LIM_T_RL, false);
        set_master_volume_pct_runtime(P_MTH30_MASTER_VOL, false);
    } else if (profile_id == PROFILE_ID_Z2300_SAT) {
        apply_param("sub_sb", P_Z2300_SUB_SB, false);
        apply_param("sub_lp", P_Z2300_SUB_LP, false);
        apply_param("tops_hp", P_Z2300_TOPS_HP, false);
        apply_param("t_dly", P_Z2300_T_DLY, false);
        apply_param("v_dac1_l", P_Z2300_V_DAC1_L, false);
        apply_param("v_dac1_r", P_Z2300_V_DAC1_R, false);
        apply_param("v_dac2_l", P_Z2300_V_DAC2_L, false);
        apply_param("v_dac2_r", P_Z2300_V_DAC2_R, false);
        apply_param("lim_s_th", P_Z2300_LIM_S_TH, false);
        apply_param("lim_s_rl", P_Z2300_LIM_S_RL, false);
        apply_param("lim_t_th", P_Z2300_LIM_T_TH, false);
        apply_param("lim_t_rl", P_Z2300_LIM_T_RL, false);
        set_master_volume_pct_runtime(P_Z2300_MASTER_VOL, false);
    } else {
        if (!load_user_profile_from_nvs()) {
            set_master_volume_pct_runtime(STARTUP_MASTER_VOL_PCT, false);
            save_user_profile_to_nvs();
        }
    }

    save_active_profile_id();
}

static void activate_user_profile_from_manual_change() {
    if (!profile_bootstrapped) return;
    active_profile_id = PROFILE_ID_USER;
    save_active_profile_id();
    save_user_profile_to_nvs();
}

// Gemeinsame Parameter-Logik für Web/BT/OLED
void apply_param(const String& key, float val, bool persist_as_user) {
    float q = get_q_factor();
    int stages = get_filter_stages();

    if (key == "sub_sb") {
        if (val >= 30.0) {
            sub_subsonic = val;
            sub_sub_L.begin(val, sample_rate, q);
            sub_sub_R.begin(val, sample_rate, q);
            if (stages >= 2) {
                sub_sub_L2.begin(val, sample_rate, q);
                sub_sub_R2.begin(val, sample_rate, q);
            }
            if (stages >= 4) {
                sub_sub_L3.begin(val, sample_rate, q);
                sub_sub_R3.begin(val, sample_rate, q);
                sub_sub_L4.begin(val, sample_rate, q);
                sub_sub_R4.begin(val, sample_rate, q);
            }
        }
    } else if (key == "sub_lp") {
        sub_lowpass = val;
        sub_lp_L.begin(sub_lowpass, sample_rate, q);
        sub_lp_R.begin(sub_lowpass, sample_rate, q);
        if (stages >= 2) {
            sub_lp_L2.begin(sub_lowpass, sample_rate, q);
            sub_lp_R2.begin(sub_lowpass, sample_rate, q);
        }
        if (stages >= 4) {
            sub_lp_L3.begin(sub_lowpass, sample_rate, q);
            sub_lp_R3.begin(sub_lowpass, sample_rate, q);
            sub_lp_L4.begin(sub_lowpass, sample_rate, q);
            sub_lp_R4.begin(sub_lowpass, sample_rate, q);
        }
    } else if (key == "tops_hp") {
        tops_highpass = val;
        tops_hp_L.begin(tops_highpass, sample_rate, q);
        tops_hp_R.begin(tops_highpass, sample_rate, q);
        if (stages >= 2) {
            tops_hp_L2.begin(tops_highpass, sample_rate, q);
            tops_hp_R2.begin(tops_highpass, sample_rate, q);
        }
        if (stages >= 4) {
            tops_hp_L3.begin(tops_highpass, sample_rate, q);
            tops_hp_R3.begin(tops_highpass, sample_rate, q);
            tops_hp_L4.begin(tops_highpass, sample_rate, q);
            tops_hp_R4.begin(tops_highpass, sample_rate, q);
        }
    } else if (key == "t_dly") {
        tops_delay_ms = val;
        int s = (int)(tops_delay_ms * sample_rate / 1000.0);
        if (s < 0) s = 0;
        if (s > TOPS_DELAY_MAX - 1) s = TOPS_DELAY_MAX - 1;
        tops_delay_samples = s;
    } else if (key == "v_dac1_l") {
        vol_dac1_l = val / 100.0f;
    } else if (key == "v_dac1_r") {
        vol_dac1_r = val / 100.0f;
    } else if (key == "v_dac2_l") {
        vol_dac2_l = val / 100.0f;
    } else if (key == "v_dac2_r") {
        vol_dac2_r = val / 100.0f;
    } else if (key == "v_tops") {   // Alias: alle Nicht-Sub-Kanäle gemeinsam
        vol_dac1_r = val / 100.0f;
        vol_dac2_l = val / 100.0f;
        vol_dac2_r = val / 100.0f;
    } else if (key == "v_sub") {    // Alias: Sub-Kanal (DAC1 L)
        vol_dac1_l = val / 100.0f;
    } else if (key == "lim_s_th") {
        // Threshold in % der Vollaussteuerung (50..100) -> Sample-Wert
        lim_sub_thresh = val / 100.0f * 32767.0f;
        lim_sub.begin(lim_sub_thresh, sample_rate, LIM_SUB_ATTACK_MS, lim_sub_rel);
    } else if (key == "lim_s_rl") {
        lim_sub_rel = val;
        lim_sub.begin(lim_sub_thresh, sample_rate, LIM_SUB_ATTACK_MS, lim_sub_rel);
    } else if (key == "lim_t_th") {
        lim_tops_thresh = val / 100.0f * 32767.0f;
        lim_tops.begin(lim_tops_thresh, sample_rate, LIM_TOPS_ATTACK_MS, lim_tops_rel);
    } else if (key == "lim_t_rl") {
        lim_tops_rel = val;
        lim_tops.begin(lim_tops_thresh, sample_rate, LIM_TOPS_ATTACK_MS, lim_tops_rel);
    }
    Serial.printf("[SET] %s = %.1f\n", key.c_str(), val);

    if (persist_as_user) activate_user_profile_from_manual_change();
}

#if ENABLE_POT_VOLUME
// Zwei Drehpotis regeln v_dac1_l/v_dac1_r direkt per ADC.
struct PotChannel {
    uint8_t pin;
    const char* key;
    float smoothed;            // geglätteter ADC-Rohwert (0..4095)
    float last_committed_pct;  // zuletzt an apply_param() übergebener Wert, -1 = noch nie
    uint32_t last_change_ms;
    bool pending_save;
};

static PotChannel pot_dac1_l = {POT_PIN_DAC1_L, "v_dac1_l", 0.0f, -1.0f, 0, false};
static PotChannel pot_dac1_r = {POT_PIN_DAC1_R, "v_dac1_r", 0.0f, -1.0f, 0, false};

static void service_pot_channel(PotChannel& ch) {
    int raw = analogRead(ch.pin);
    ch.smoothed += (raw - ch.smoothed) * POT_SMOOTHING_ALPHA;

    float pct = ch.smoothed / 4095.0f * 100.0f;
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    if (ch.last_committed_pct < 0.0f || fabsf(pct - ch.last_committed_pct) >= POT_DEADBAND_PCT) {
        apply_param(ch.key, pct, false);   // sofort hörbar, aber noch nicht ins NVS schreiben
        ch.last_committed_pct = pct;
        ch.last_change_ms = millis();
        ch.pending_save = true;
    } else if (ch.pending_save && millis() - ch.last_change_ms >= POT_SAVE_DEBOUNCE_MS) {
        apply_param(ch.key, ch.last_committed_pct, true);  // Poti steht still -> jetzt persistieren
        ch.pending_save = false;
    }
}

static void setup_pot_volume() {
    analogSetPinAttenuation(POT_PIN_DAC1_L, ADC_11db);
    analogSetPinAttenuation(POT_PIN_DAC1_R, ADC_11db);
    // Mit aktueller Poti-Stellung vorbelegen, damit beim ersten service_pot_volume()
    // kein Sprung vom (falschen) Default-Wert aus passiert.
    pot_dac1_l.smoothed = analogRead(POT_PIN_DAC1_L);
    pot_dac1_r.smoothed = analogRead(POT_PIN_DAC1_R);
}

static void service_pot_volume() {
    service_pot_channel(pot_dac1_l);
    service_pot_channel(pot_dac1_r);
}
#endif

#if ENABLE_OLED_MENU
U8G2_SH1106_128X64_NONAME_F_SW_I2C oled(U8G2_R0, /* clock=*/ OLED_PIN_SCL, /* data=*/ OLED_PIN_SDA, /* reset=*/ U8X8_PIN_NONE);

struct MenuItem {
    const char* key;
    const char* label;
    float min_v;
    float max_v;
    float step;
    const char* unit;
};

static const MenuItem kMenuItems[] = {
    {"profile",  "Profile",    0.0f,   2.0f,   1.0f, ""},
    {"sub_sb",   "Subsonic",   30.0f,  60.0f,   1.0f, "Hz"},
    {"sub_lp",   "Sub LP",     60.0f, 180.0f,   1.0f, "Hz"},
    {"tops_hp",  "Tops HP",    60.0f, 150.0f,   1.0f, "Hz"},
    {"t_dly",    "Delay",       0.0f,  30.0f,   0.1f, "ms"},
    {"v_dac1_l", "Vol Sub",     0.0f, 100.0f,   1.0f, "%"},
    {"v_dac1_r", "Vol Mono",    0.0f, 100.0f,   1.0f, "%"},
    {"v_dac2_l", "Vol Tops L",  0.0f, 100.0f,   1.0f, "%"},
    {"v_dac2_r", "Vol Tops R",  0.0f, 100.0f,   1.0f, "%"},
    {"lim_s_th", "Lim Sub Th", 50.0f, 100.0f,   1.0f, "%"},
    {"lim_s_rl", "Lim Sub Rel",20.0f, 500.0f,   5.0f, "ms"},
    {"lim_t_th", "Lim Top Th", 50.0f, 100.0f,   1.0f, "%"},
    {"lim_t_rl", "Lim Top Rel",10.0f, 500.0f,   5.0f, "ms"},
};

static const int MENU_COUNT = (int)(sizeof(kMenuItems) / sizeof(kMenuItems[0]));
static int menu_index = 0;
static bool menu_edit = false;
static bool menu_nav_mode = false; // false = Master-Volume, true = Menue
static bool menu_ready = false;
static uint32_t menu_last_draw_ms = 0;
static uint32_t menu_last_input_ms = 0;
static bool menu_force_redraw = true;
static bool oled_awake = true;
static bool oled_dimmed = false;

// Rotary-State (Gray-Code 2-bit)
static uint8_t enc_prev = 0;

struct DebouncedButton {
    uint8_t pin;
    bool last_raw;
    bool stable;
    uint32_t last_change_ms;
};

static DebouncedButton btn_back = {BTN_BACK_PIN, false, false, 0};
static DebouncedButton btn_confirm = {BTN_CONFIRM_PIN, false, false, 0};
static DebouncedButton btn_encoder_push = {BTN_ENCODER_PUSH_PIN, false, false, 0};

static bool read_button_pressed(uint8_t pin) {
#if MENU_BUTTON_ACTIVE_LOW
    return digitalRead(pin) == LOW;
#else
    return digitalRead(pin) == HIGH;
#endif
}

static bool button_pressed_event(DebouncedButton& b) {
    const uint32_t DEBOUNCE_MS = 25;
    bool raw = read_button_pressed(b.pin);
    if (raw != b.last_raw) {
        b.last_raw = raw;
        b.last_change_ms = millis();
    }
    if ((millis() - b.last_change_ms) > DEBOUNCE_MS && raw != b.stable) {
        b.stable = raw;
        if (b.stable) return true;
    }
    return false;
}

static float menu_get_value(const char* key) {
    if (!strcmp(key, "profile")) return (float)active_profile_id;
    if (!strcmp(key, "sub_sb")) return sub_subsonic;
    if (!strcmp(key, "sub_lp")) return sub_lowpass;
    if (!strcmp(key, "tops_hp")) return tops_highpass;
    if (!strcmp(key, "t_dly")) return tops_delay_ms;
    if (!strcmp(key, "v_dac1_l")) return vol_dac1_l * 100.0f;
    if (!strcmp(key, "v_dac1_r")) return vol_dac1_r * 100.0f;
    if (!strcmp(key, "v_dac2_l")) return vol_dac2_l * 100.0f;
    if (!strcmp(key, "v_dac2_r")) return vol_dac2_r * 100.0f;
    if (!strcmp(key, "lim_s_th")) return lim_sub_thresh / 32767.0f * 100.0f;
    if (!strcmp(key, "lim_s_rl")) return lim_sub_rel;
    if (!strcmp(key, "lim_t_th")) return lim_tops_thresh / 32767.0f * 100.0f;
    if (!strcmp(key, "lim_t_rl")) return lim_tops_rel;
    return 0.0f;
}

static void set_master_volume_pct(float pct) {
    set_master_volume_pct_runtime(pct, true);
    activate_user_profile_from_manual_change();
}

static void menu_set_value(const MenuItem& item, float value) {
    if (value < item.min_v) value = item.min_v;
    if (value > item.max_v) value = item.max_v;
    if (!strcmp(item.key, "profile")) {
        uint8_t p = (uint8_t)(value + 0.5f);
        apply_profile(p);
        return;
    }
    apply_param(item.key, value);
}

static const char* profile_name(uint8_t p) {
    if (p == PROFILE_ID_MTH30_TOP) return "MTH30+TOP";
    if (p == PROFILE_ID_Z2300_SAT) return "Z2300+SAT";
    return "USER";
}

static void draw_menu() {
    if (!menu_ready || !oled_awake) return;
    char line[40];
    oled.firstPage();
    do {
        oled.setFont(u8g2_font_6x12_tf);
        if (!menu_nav_mode) {
            oled.setFont(u8g2_font_6x12_tf);
            snprintf(line, sizeof(line), "Master: %.0f%%", master_volume_pct_runtime);
            oled.drawStr(0, 12, line);
            snprintf(line, sizeof(line), "Vol R: %.0f%%", vol_dac2_r * 100.0f);
            oled.drawStr(0, 26, line);
            snprintf(line, sizeof(line), "Vol L: %.0f%%", vol_dac2_l * 100.0f);
            oled.drawStr(0, 40, line);
            continue;
        }

        oled.drawStr(0, 10, "DSP Menu");
        oled.drawStr(76, 10, menu_edit ? "EDIT" : "NAV");

        int top = menu_index - 2;
        if (top < 0) top = 0;
        if (top > MENU_COUNT - 4) top = MENU_COUNT - 4;
        if (top < 0) top = 0;

        for (int row = 0; row < 4; row++) {
            int idx = top + row;
            if (idx >= MENU_COUNT) break;
            int y = 24 + row * 10;
            bool selected = (idx == menu_index);

            float v = menu_get_value(kMenuItems[idx].key);
            int decimals = (kMenuItems[idx].step < 1.0f) ? 1 : 0;
            if (!strcmp(kMenuItems[idx].key, "profile")) {
                snprintf(line, sizeof(line), "%s %s", kMenuItems[idx].label, profile_name((uint8_t)v));
            } else {
                snprintf(line, sizeof(line), "%s %.1f%s", kMenuItems[idx].label, v, kMenuItems[idx].unit);
                if (decimals == 0) snprintf(line, sizeof(line), "%s %.0f%s", kMenuItems[idx].label, v, kMenuItems[idx].unit);
            }

            if (selected) {
                oled.drawBox(0, y - 9, 128, 10);
                oled.setDrawColor(0);
                oled.drawStr(1, y, line);
                oled.setDrawColor(1);
            } else {
                oled.drawStr(1, y, line);
            }
        }
    } while (oled.nextPage());
}

static int read_encoder_delta() {
    // 4-bit Zustandsautomat: prev(AB)<<2 | curr(AB)
    static const int8_t table[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
    uint8_t a = (uint8_t)digitalRead(ROTARY_PIN_A);
    uint8_t b = (uint8_t)digitalRead(ROTARY_PIN_B);
    uint8_t curr = (a << 1) | b;
    uint8_t idx = (enc_prev << 2) | curr;
    enc_prev = curr;
    int d = table[idx];
#if ROTARY_INVERT_DIR
    d = -d;
#endif
    return d;
}

static void setup_oled_menu() {
    pinMode(ROTARY_PIN_A, INPUT_PULLUP);
    pinMode(ROTARY_PIN_B, INPUT_PULLUP);
    pinMode(BTN_BACK_PIN, INPUT_PULLUP);
    pinMode(BTN_CONFIRM_PIN, INPUT_PULLUP);
    if (BTN_ENCODER_PUSH_PIN != BTN_CONFIRM_PIN && BTN_ENCODER_PUSH_PIN != BTN_BACK_PIN) {
        pinMode(BTN_ENCODER_PUSH_PIN, INPUT_PULLUP);
    }
    btn_back.last_raw = btn_back.stable = read_button_pressed(btn_back.pin);
    btn_confirm.last_raw = btn_confirm.stable = read_button_pressed(btn_confirm.pin);
    btn_encoder_push.last_raw = btn_encoder_push.stable = read_button_pressed(btn_encoder_push.pin);

    uint8_t a = (uint8_t)digitalRead(ROTARY_PIN_A);
    uint8_t b = (uint8_t)digitalRead(ROTARY_PIN_B);
    enc_prev = (a << 1) | b;

    Wire.begin(OLED_PIN_SDA, OLED_PIN_SCL);

    // Full I2C bus scan - print every device that responds
    Serial.printf("[MENU] I2C Scan auf SDA=GPIO%d SCL=GPIO%d ...\n", OLED_PIN_SDA, OLED_PIN_SCL);
    uint8_t found_addr = 0;
    int found_count = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("[MENU] I2C Geraet gefunden: 0x%02X\n", addr);
            if (!found_addr) found_addr = addr;
            found_count++;
        }
    }
    if (found_count == 0) {
        Serial.println("[MENU] KEIN I2C Geraet gefunden! Verkabelung pruefen.");
        found_addr = OLED_I2C_ADDR;
    } else {
        Serial.printf("[MENU] Verwende OLED auf 0x%02X\n", found_addr);
    }

    oled.setI2CAddress((uint8_t)(found_addr << 1));
    oled.begin();
    oled.setPowerSave(0);
    oled.setContrast(OLED_ACTIVE_CONTRAST);
    set_master_volume_pct_runtime(master_volume_pct_runtime, false);
    menu_ready = true;
    menu_force_redraw = true;
    menu_last_input_ms = millis();
    menu_nav_mode = false;
    menu_edit = false;
    Serial.println("[MENU] OLED + Rotary aktiv");
}

static void service_oled_menu() {
    if (!menu_ready) return;

    int d = read_encoder_delta();
    bool ev_confirm = button_pressed_event(btn_confirm);
    bool ev_encoder_push = false;
    if (BTN_ENCODER_PUSH_PIN != BTN_CONFIRM_PIN && BTN_ENCODER_PUSH_PIN != BTN_BACK_PIN) {
        ev_encoder_push = button_pressed_event(btn_encoder_push);
    }
    bool ev_back = button_pressed_event(btn_back);
    bool ev_enter = ev_confirm || ev_encoder_push;

    if (d != 0) {
        DEBUG_LOG("Rotary: d=%d, mode=%s", d, menu_nav_mode ? "MENU" : "VOL");
    }
    if (ev_enter) {
        DEBUG_LOG("Event: ENTER pressed");
    }
    if (ev_back) {
        DEBUG_LOG("Event: BACK pressed");
    }

    if (d != 0 || ev_enter || ev_back) {
        menu_last_input_ms = millis();
        if (!oled_awake) {
            oled_awake = true;
            oled.setPowerSave(0);
            oled.setContrast(OLED_ACTIVE_CONTRAST);
        }
        if (oled_dimmed) {
            oled_dimmed = false;
            oled.setContrast(OLED_ACTIVE_CONTRAST);
        }
        menu_force_redraw = true;
    }

    if (d != 0) {
        if (!menu_nav_mode) {
            set_master_volume_pct(master_volume_pct_runtime + d * MASTER_VOL_STEP_PCT);
        } else if (menu_edit) {
            const MenuItem& item = kMenuItems[menu_index];
            float v = menu_get_value(item.key);
            menu_set_value(item, v + d * item.step);
        } else {
            menu_index += d;
            if (menu_index < 0) menu_index = MENU_COUNT - 1;
            if (menu_index >= MENU_COUNT) menu_index = 0;
        }
    }

    if (ev_enter) {
        if (!menu_nav_mode) {
            menu_nav_mode = true;
            menu_edit = false;
        } else {
            menu_edit = !menu_edit;
        }
    }

    if (ev_back) {
        if (menu_edit) {
            menu_edit = false;
        } else if (menu_nav_mode) {
            menu_nav_mode = false;
        }
    }

    if (oled_awake && !oled_dimmed && (millis() - menu_last_input_ms > (unsigned long)OLED_DIM_TIMEOUT_SEC * 1000UL)) {
        oled.setContrast(OLED_DIM_CONTRAST);
        oled_dimmed = true;
    }

    if (oled_awake && (millis() - menu_last_input_ms > (unsigned long)OLED_SLEEP_TIMEOUT_SEC * 1000UL)) {
        oled.setPowerSave(1);
        oled_awake = false;
        oled_dimmed = false;
    }

    if (oled_awake && menu_force_redraw) {
        draw_menu();
        menu_last_draw_ms = millis();
        menu_force_redraw = false;
    }
}
#endif

// Bluetooth-A2DP starten (einmalig). Wird entweder direkt beim Boot aufgerufen
// (WebGUI aus) oder erst nach dem WiFi-Timeout (WebGUI an), damit WLAN und BT
// nie gleichzeitig Speicher belegen.
void start_bluetooth() {
#if !ENABLE_DAC2 && !ENABLE_BT_FALLBACK_WITH_AUX
    // PCM1808-Line-In-Modus nutzt keinen A2DP-Stream als Quelle.
    return;
#else
    if (bt_started) return;
    Serial.println("Starting Bluetooth...");
    a2dp_sink->start(BT_DEVICE_NAME);
    bt_started = true;
#if ENABLE_BT_CONFIG
    // SPP nach A2DP starten – der BT-Stack läuft bereits, bt_serial hängt sich
    // als zusätzliches Profil ein (kein erneutes esp_bluedroid_init).
    bt_serial.begin(BT_DEVICE_NAME);
    Serial.println("BT-Config (SPP) aktiv - App: 'Serial Bluetooth Terminal'");
#endif
    Serial.println("Bluetooth started");
#endif
}

void setup() {
    Serial.begin(115200);
    delay(200); // UART settle
    Serial.printf("\n\n=== SETUP START  free heap: %lu ===\n", (unsigned long)ESP.getFreeHeap());

    // I2C Scanner
#if ENABLE_OLED_MENU
    Wire.begin(OLED_PIN_SDA, OLED_PIN_SCL);
    Serial.println("\n[I2C] Scanne alle Adressen...");
    int found = 0;
    for (byte addr = 0; addr < 128; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  -> I2C Device: 0x%02X (%d)\n", addr, addr);
            found++;
        }
    }
    Serial.printf("[I2C] Insgesamt %d Device(s) gefunden\n\n", found);

    // ADC für Potentiometer konfigurieren
    analogReadResolution(12);  // 12-bit (0-4095)
    analogSetAttenuation(ADC_11db);  // 0-3.3V Bereich
    pinMode(POT_PIN_DAC1_L, INPUT);
    pinMode(POT_PIN_DAC1_R, INPUT);
    Serial.println("[ADC] Potentiometer auf GPIO 34/35 aktiviert");
#endif

    // Audio-Objekte hier auf dem Heap erzeugen (Heap ist jetzt initialisiert)
    Serial.println("[1] new I2SStream tops");
    i2s_tops  = new I2SStream();
#if ENABLE_DAC2
    Serial.println("[2] new I2SStream sub");
    i2s_sub   = new I2SStream();
#endif
    Serial.println("[3] new MultiOutput");
    multi_out = new MultiOutput();
    Serial.println("[4] new CallbackStream tops");
    cb_tops   = new CallbackStream(*i2s_tops, process_dac1);
#if ENABLE_DAC2
    Serial.println("[5] new CallbackStream sub");
    cb_sub    = new CallbackStream(*i2s_sub,  process_dac2);
#endif
#if ENABLE_DAC2 || ENABLE_BT_FALLBACK_WITH_AUX
    Serial.printf("[6] new BluetoothA2DPSink  free heap: %lu\n", (unsigned long)ESP.getFreeHeap());
    a2dp_sink = new BluetoothA2DPSink(*multi_out);
#else
    Serial.printf("[6] PCM1808-LineIn-Modus  free heap: %lu\n", (unsigned long)ESP.getFreeHeap());
#endif

    // Limiter konfigurieren (Threshold ~ -0,2 dBFS Headroom).
    Serial.println("[7] limiter begin");
    lim_tops.begin(lim_tops_thresh, sample_rate, LIM_TOPS_ATTACK_MS, lim_tops_rel);
    lim_sub.begin(lim_sub_thresh,  sample_rate, LIM_SUB_ATTACK_MS,  lim_sub_rel);

    g_prefs_ready = g_prefs.begin("dspcfg", false);
    if (!g_prefs_ready) Serial.println("[NVS] open failed, running without persistence");

    uint8_t boot_profile = DEFAULT_PROFILE_ID;
    if (g_prefs_ready) boot_profile = g_prefs.getUChar("profile", DEFAULT_PROFILE_ID);
    apply_profile(boot_profile);
    profile_bootstrapped = true;

    // Filter-System mit korrektem Typ, Ordnung und Q-Faktor initialisieren
    float q = get_q_factor();
    int stages = get_filter_stages();
    update_all_filters(q);
    const char* filter_types[] = {"Butterworth", "Bessel", "Chebyshev"};
    int filter_db_oct = stages * 12;
    Serial.printf("[Filter] Type=%s, Order=LR%d (%ddB/Oct), Q=%.4f\n",
        filter_types[FILTER_TYPE], filter_db_oct, filter_db_oct, q);

    // DAC 1 Pins (Canton Tops)
    Serial.println("[8] i2s_tops begin");
    auto config_tops = i2s_tops->defaultConfig();
    config_tops.port_no    = 0;
    config_tops.sample_rate = 44100;
    config_tops.bits_per_sample = 16;
    config_tops.channels   = 2;
    config_tops.pin_bck  = TOPS_PIN_BCK; config_tops.pin_ws = TOPS_PIN_WS; config_tops.pin_data = TOPS_PIN_DATA;
    // DMA-Puffer groß genug, damit bei zwei aktiven I2S-Ports keine Underruns
    // auftreten (Underruns klingen wie "zu niedrige Bitrate" / grieselig).
    config_tops.buffer_count = 8;
    config_tops.buffer_size  = 512;
    i2s_tops->begin(config_tops);
    Serial.println("[8] i2s_tops OK");

#if ENABLE_DAC2
    // DAC 2 Pins (MTH-30 Sub) – bei ENABLE_DAC2 0 bleiben diese Pins frei
    // (z.B. fuer PCM1808 ADC-Eingang; Konfiguration als I2S-Input separat noetig)
    Serial.println("[9] i2s_sub begin");
    auto config_sub = i2s_sub->defaultConfig();
    config_sub.port_no    = 1;
    config_sub.sample_rate = 44100;
    config_sub.bits_per_sample = 16;
    config_sub.channels   = 2;
    config_sub.pin_bck  = SUB_PIN_BCK; config_sub.pin_ws = SUB_PIN_WS; config_sub.pin_data = SUB_PIN_DATA;
    config_sub.buffer_count = 8;
    config_sub.buffer_size  = 512;
    i2s_sub->begin(config_sub);
    Serial.println("[9] i2s_sub OK");
#else
    Serial.println("[9] setup PCM1808 input");
    setup_pcm1808_input();
#endif

    multi_out->add(*cb_tops);
#if ENABLE_DAC2
    multi_out->add(*cb_sub);
#endif

#if ENABLE_DAC2 || ENABLE_BT_FALLBACK_WITH_AUX
    // Verbindungs-Callback zum Debuggen
    a2dp_sink->set_on_connection_state_changed([](esp_a2d_connection_state_t state, void* ptr){
        Serial.printf("[BT] Connection state: %d (2=connected)\n", state);
        if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            pending_sound = 1;
#if ENABLE_BT_FALLBACK_WITH_AUX
            bt_connected = true;
            aux_priority_active = false;
#endif
        } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            pending_sound = 2;
#if ENABLE_BT_FALLBACK_WITH_AUX
            bt_connected = false;
#endif
        }
    });
    a2dp_sink->set_on_audio_state_changed([](esp_a2d_audio_state_t state, void* ptr){
        Serial.printf("[BT] Audio state: %d (1=started)\n", state);
    });
#endif

    // Betriebsmodus:
    //  - ENABLE_WEBGUI = 0: Bluetooth startet sofort (kein WLAN).
    //  - ENABLE_WEBGUI = 1: ZUERST nur WLAN-AP + Webserver. Bluetooth startet
    //    erst, wenn der Webserver wegen Inaktivität abgeschaltet wird
    //    (siehe shutdown_wifi()). So laufen WLAN und BT nie gleichzeitig ->
    //    kein Speicherkonflikt (esp_wifi_init / NO_MEM).
#if !ENABLE_WEBGUI
    start_bluetooth();
#else
    // WiFi AP zuerst (reiner AP-Modus spart RAM).
    Serial.println("Starting WiFi AP...");
    WiFi.mode(WIFI_AP);
    // KEIN softAPConfig: Standard-AP-IP ist bereits 192.168.4.1 und
    // der ESP32-DHCP-Server läuft damit zuverlässig.
    // Kanal 6, versteckt=false, max. 4 Clients
    bool ap_ok = WiFi.softAP(ap_ssid, ap_password, 6, 0, 4);
    delay(500);
    Serial.print("AP started: "); Serial.println(ap_ok ? "YES" : "NO");
    Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

    // BT/WLAN-Koexistenz auf BALANCE stellen. Die A2DP-Lib bevorzugt sonst
    // Bluetooth, wodurch WLAN beim DHCP-Handshake verhungert. BALANCE gibt
    // WLAN genug Funkzeit -> IP-Vergabe klappt auch bei aktivem Audio.
    esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
    Serial.println("Koexistenz-Priorität: BALANCE");

    // DNS-Server: leitet ALLE Domains auf die ESP32-IP -> Captive Portal
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(DNS_PORT, "*", ap_ip);
    Serial.println("Captive-Portal DNS gestartet");

    // GUI-Seite gepaced in kleinen Blöcken senden. WICHTIG: zwischen den
    // Blöcken delay() aufrufen, damit der Bluetooth-Host-Task Zeit bekommt,
    // seine HCI-RX-Pakete abzuholen. Sonst läuft der BT-Controller über
    // und löst den Absturz "host_recv_pkt_cb hci_hal_h4.c" aus.
    auto send_gui = []() {
        last_web_activity = millis();
        WiFiClient client = server.client();
        client.setTimeout(5000);
        client.print("HTTP/1.1 200 OK\r\n");
        client.print("Content-Type: text/html\r\n");
        client.print("Cache-Control: no-cache\r\n");
        client.print("Connection: close\r\n");
        client.printf("Content-Length: %u\r\n\r\n", (unsigned)strlen(HTML_GUI));
        const size_t CHUNK = 512;
        size_t total = strlen(HTML_GUI);
        size_t sent = 0;
        uint32_t start = millis();
        while (sent < total && client.connected()) {
            size_t n = min(CHUNK, total - sent);
            size_t w = client.write((const uint8_t*)(HTML_GUI + sent), n);
            if (w == 0) {
                // Puffer voll (EAGAIN): BT+WLAN Zeit geben, dann erneut
                if (millis() - start > 5000) break; // Sicherheits-Timeout
                delay(3);
                continue;
            }
            sent += w;
            delay(2); // BT-Host-Task bedienen -> kein HCI-Überlauf
        }
        client.flush();
        client.stop();
    };

    server.on("/", send_gui);
    server.on("/set", handle_update);
    server.on("/status", handle_status);

    // GUI-Button "Auf Bluetooth wechseln": Antwort sofort senden, das eigentliche
    // Abschalten passiert im loop() (Server darf nicht mitten im Request sterben).
    server.on("/bt", []() {
        last_web_activity = millis();
        server.send(200, "text/plain", "OK - wechsle auf Bluetooth");
        bt_switch_requested = true;
    });

    // Captive-Portal-Erkennung der Betriebssysteme:
    // Android /generate_204 /gen_204, Apple /hotspot-detect.html,
    // Windows /connecttest.txt /ncsi.txt. Alle unbekannten URLs
    // leiten auf die Bedienseite -> Handy öffnet sie automatisch.
    server.onNotFound([]() {
        server.sendHeader("Location", "http://192.168.4.1/", true);
        server.send(302, "text/plain", "");
    });
    server.begin();
    Serial.println("HTTP server started");
    Serial.println("WebGUI aktiv - Bluetooth startet nach Inaktivitäts-Timeout");
    last_web_activity = millis();
#endif

#if ENABLE_OLED_MENU
    setup_oled_menu();
#endif
#if ENABLE_POT_VOLUME
    setup_pot_volume();
#endif
}

#if ENABLE_BT_CONFIG
// Eingehende SPP-Zeichen verarbeiten (zeilenweise Befehle).
// Wird im loop() aufgerufen, blockiert nicht.
void handle_bt_serial() {
    while (bt_serial.available()) {
        char c = (char)bt_serial.read();
        if (c == '\n' || c == '\r') {
            if (bt_rx_len == 0) continue;
            bt_rx_buf[bt_rx_len] = '\0';
            bt_rx_len = 0;
            String line(bt_rx_buf);
            line.trim();
            if (line == "status" || line == "?") {
                // Maschinenlesbares Format: key=wert, eine Zeile je Parameter.
                // Wird von der App Inventor App beim Verbinden geparst.
                bt_serial.printf(
                    "sub_sb=%.1f\r\nsub_lp=%.1f\r\ntops_hp=%.1f\r\nt_dly=%.1f\r\n"
                    "v_dac1_l=%.0f\r\nv_dac1_r=%.0f\r\nv_dac2_l=%.0f\r\nv_dac2_r=%.0f\r\n"
                    "lim_s_th=%.0f\r\nlim_s_rl=%.0f\r\nlim_t_th=%.0f\r\nlim_t_rl=%.0f\r\n"
                    "END\r\n",
                    sub_subsonic, sub_lowpass, tops_highpass, tops_delay_ms,
                    vol_dac1_l * 100.0f, vol_dac1_r * 100.0f,
                    vol_dac2_l * 100.0f, vol_dac2_r * 100.0f,
                    lim_sub_thresh  / 32767.0f * 100.0f, lim_sub_rel,
                    lim_tops_thresh / 32767.0f * 100.0f, lim_tops_rel);
            } else if (line == "reset") {
                // Alle Parameter auf die in config.h definierten Standardwerte zurücksetzen.
                // apply_param() wird genutzt damit Filter/Limiter korrekt neu initialisiert werden.
                apply_param("sub_sb",   DEFAULT_SUB_SUBSONIC);
                apply_param("sub_lp",   DEFAULT_SUB_LOWPASS);
                apply_param("tops_hp",  DEFAULT_TOPS_HIGHPASS);
                apply_param("t_dly",    DEFAULT_TOPS_DELAY_MS);
                apply_param("v_dac1_l", DEFAULT_VOL_DAC1_L * 100.0f);
                apply_param("v_dac1_r", DEFAULT_VOL_DAC1_R * 100.0f);
                apply_param("v_dac2_l", DEFAULT_VOL_DAC2_L * 100.0f);
                apply_param("v_dac2_r", DEFAULT_VOL_DAC2_R * 100.0f);
                apply_param("lim_s_th", DEFAULT_LIM_SUB_THRESH  / 32767.0f * 100.0f);
                apply_param("lim_s_rl", DEFAULT_LIM_SUB_REL);
                apply_param("lim_t_th", DEFAULT_LIM_TOPS_THRESH / 32767.0f * 100.0f);
                apply_param("lim_t_rl", DEFAULT_LIM_TOPS_REL);
                bt_serial.print("OK reset – alle Standardwerte wiederhergestellt\r\n");
            } else if (line == "help") {
                bt_serial.print(
                    "-- Befehle --\r\n"
                    "key=wert     Parameter setzen\r\n"
                    "status / ?   Alle Werte anzeigen\r\n"
                    "reset        Alle Standardwerte aus config.h wiederherstellen\r\n"
                    "-- Parameter --\r\n"
                    "sub_sb       Subsonic Hz (30-60)\r\n"
                    "sub_lp       Sub-Tiefpass Hz (60-180)\r\n"
                    "tops_hp      Tops-Hochpass Hz (60-150)\r\n"
                    "t_dly        Tops-Delay ms (0-30)\r\n"
                    "v_dac1_l     DAC1 L Lautst. % (Sub)\r\n"
                    "v_dac1_r     DAC1 R Lautst. % (Mono)\r\n"
                    "v_dac2_l     DAC2 L Lautst. % (Tops L)\r\n"
                    "v_dac2_r     DAC2 R Lautst. % (Tops R)\r\n"
                    "lim_s_th     Sub Limiter Threshold %\r\n"
                    "lim_s_rl     Sub Limiter Release ms\r\n"
                    "lim_t_th     Tops Limiter Threshold %\r\n"
                    "lim_t_rl     Tops Limiter Release ms\r\n");
            } else {
                int eq = line.indexOf('=');
                if (eq > 0) {
                    String key = line.substring(0, eq);
                    float  val = line.substring(eq + 1).toFloat();
                    apply_param(key, val);
                    bt_serial.printf("OK %s=%.2f\r\n", key.c_str(), val);
                } else {
                    bt_serial.print("ERR Unbekannt. 'help' fuer Befehlsliste.\r\n");
                }
            }
        } else if (bt_rx_len < (uint8_t)(sizeof(bt_rx_buf) - 1)) {
            bt_rx_buf[bt_rx_len++] = c;
        }
    }
}
#endif

void loop() {
    // Potentiometer auslesen und Volume regeln
#if ENABLE_OLED_MENU
    int raw_l = analogRead(POT_PIN_DAC1_L);
    int raw_r = analogRead(POT_PIN_DAC1_R);

    // Normalisieren auf 0.0-1.0 (12-bit: 0-4095)
    float norm_l = raw_l / 4095.0f;
    float norm_r = raw_r / 4095.0f;

    // Exponentielles Moving Average (Smoothing gegen Jitter)
    float old_l = poti_l_smooth;
    float old_r = poti_r_smooth;
    poti_l_smooth = poti_l_smooth * (1.0f - POTI_ALPHA) + norm_l * POTI_ALPHA;
    poti_r_smooth = poti_r_smooth * (1.0f - POTI_ALPHA) + norm_r * POTI_ALPHA;

    // Volumes aktualisieren
    vol_dac1_l = poti_l_smooth;
    vol_dac1_r = poti_r_smooth;

    // OLED aktualisieren, wenn sich die Werte signifikant ändern (>2%)
    if (fabs(poti_l_smooth - old_l) > 0.02f || fabs(poti_r_smooth - old_r) > 0.02f) {
        menu_force_redraw = true;
        DEBUG_LOG("Poti: L=%.0f%% R=%.0f%%", vol_dac1_l * 100.0f, vol_dac1_r * 100.0f);
    }
#endif

    if (pending_sound) {
        int s = pending_sound;
        pending_sound = 0;
#ifdef HAS_SND_CONNECTED
        if (s == 1) play_pcm_mono(snd_connected, snd_connected_len);
#else
        if (s == 1) play_connect_sound();
#endif
#ifdef HAS_SND_DISCONNECTED
        else if (s == 2) play_pcm_mono(snd_disconnected, snd_disconnected_len);
#else
        else if (s == 2) play_disconnect_sound();
#endif
    }
#if ENABLE_BT_CONFIG
    if (bt_started) handle_bt_serial();
#endif
#if !ENABLE_DAC2
    service_pcm1808_input();
#endif

#if ENABLE_OLED_MENU
    service_oled_menu();
#endif
#if ENABLE_POT_VOLUME
    service_pot_volume();
#endif
}
