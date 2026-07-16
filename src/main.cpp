#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include <BluetoothSerial.h>
#include "config.h"   // Zentrale Konfiguration (Pins, WebGUI-Schalter, Defaults)
#if !ENABLE_DAC2
#include "driver/i2s.h"
#endif
#if ENABLE_WEBGUI
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include "esp_coexist.h"
#endif
#include "sounds.h"   // Optionale Sprach-/Audio-Clips (WAV im Flash)

#if ENABLE_WEBGUI
const char* ap_ssid = AP_SSID;
const char* ap_password = AP_PASSWORD;

// Feste AP-IP + DNS-Server für Captive Portal (automatisches Öffnen der Bedienseite)
IPAddress ap_ip(192, 168, 4, 1);
IPAddress ap_gw(192, 168, 4, 1);
IPAddress ap_mask(255, 255, 255, 0);
const byte DNS_PORT = 53;
DNSServer dnsServer;

WebServer server(80);

// Inaktivitäts-Timeout: WiFi-AP + Webserver abschalten, wenn längere Zeit
// niemand die Weiche bedient. Gibt Bluetooth die volle Funkzeit (besserer
// Sound) und spart Strom. Wird bei jedem Webzugriff zurückgesetzt.
const unsigned long WIFI_TIMEOUT_MS = (unsigned long)WIFI_TIMEOUT_SEC * 1000UL;
unsigned long last_web_activity = 0;
volatile bool bt_switch_requested = false;  // GUI-Button: sofort auf BT wechseln
#endif

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
static uint8_t pcm1808_buf[1024];
#endif

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

// Limiter-Parameter (zur Laufzeit über die Weboberfläche einstellbar).
// Startwerte + feste Attack-Zeiten kommen aus config.h.
float lim_tops_thresh = DEFAULT_LIM_TOPS_THRESH;   // Ziel-Spitzenwert Tops
float lim_tops_rel    = DEFAULT_LIM_TOPS_REL;      // Release Tops (ms)
float lim_sub_thresh  = DEFAULT_LIM_SUB_THRESH;    // Ziel-Spitzenwert Sub
float lim_sub_rel     = DEFAULT_LIM_SUB_REL;       // Release Sub (ms)

// Filter-Instanzen für die Kanäle
// Alle Trennfilter als Linkwitz-Riley 24 dB/Okt (LR24) =
// Kaskade aus 2 identischen Butterworth-Biquads (Q=0.707) pro Kanal.
LowPassFilter<float> sub_lp_L(sub_lowpass, sample_rate);
LowPassFilter<float> sub_lp_R(sub_lowpass, sample_rate);
LowPassFilter<float> sub_lp_L2(sub_lowpass, sample_rate);
LowPassFilter<float> sub_lp_R2(sub_lowpass, sample_rate);
// Subsonic-Filter: LR24 (Horn-Schutz)
HighPassFilter<float> sub_sub_L(sub_subsonic, sample_rate);
HighPassFilter<float> sub_sub_R(sub_subsonic, sample_rate);
HighPassFilter<float> sub_sub_L2(sub_subsonic, sample_rate);
HighPassFilter<float> sub_sub_R2(sub_subsonic, sample_rate);

HighPassFilter<float> tops_hp_L(tops_highpass, sample_rate);
HighPassFilter<float> tops_hp_R(tops_highpass, sample_rate);
HighPassFilter<float> tops_hp_L2(tops_highpass, sample_rate);
HighPassFilter<float> tops_hp_R2(tops_highpass, sample_rate);

#if ENABLE_WEBGUI
// Web-GUI: Konfigurationsoberfläche (Captive Portal / WLAN-AP)
const char HTML_GUI[] = R"rawliteral(
<!DOCTYPE html><html>
<head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>DSP Weiche</title>
<style>
body{font-family:'Segoe UI',Arial,sans-serif;background:#121212;color:#fff;padding:12px;margin:0}
.container{max-width:500px;margin:auto}
.card{background:#1e1e1e;padding:16px;border-radius:12px;margin-bottom:16px;border-left:5px solid #00adb5;box-shadow:0 4px 12px rgba(0,0,0,.5)}
.card.t2{border-left-color:#ff5722}
h1{font-size:1.4em;text-align:center;color:#eee;margin:8px 0 16px}
h2{margin:0 0 10px;font-size:1.1em;color:#00adb5}
.t2 h2{color:#ff5722}
hr{border:none;border-top:1px solid #333;margin:10px 0}
.row{display:flex;align-items:center;gap:6px;margin:6px 0}
.row label{flex:0 0 150px;font-size:.82em;color:#ccc;text-align:left}
.row input[type=range]{flex:1;accent-color:#00adb5;min-width:0}
.t2 .row input[type=range]{accent-color:#ff5722}
.row input[type=number]{width:54px;background:#2a2a2a;border:1px solid #444;border-radius:5px;color:#fff;padding:3px 4px;font-size:.85em;text-align:right;-moz-appearance:textfield}
.row input[type=number]::-webkit-inner-spin-button{opacity:1}
.row span{font-size:.78em;color:#888;flex:0 0 20px}
btn{display:block;width:100%;padding:13px;font-size:1em;font-weight:bold;color:#fff;background:#0055bb;border:none;border-radius:10px;cursor:pointer;margin-top:8px}
#msg{text-align:center;color:#00adb5;min-height:1.2em;margin-top:6px}
</style></head>
<body><div class="container">
<h1>&#127927; DSP Weiche</h1>

<div class="card">
<h2>DAC 1 &ndash; Subwoofer / Mono</h2><hr>
<div class="row"><label>&#128266; Sub Lautst.</label><input type="range" id="v_dac1_l" min="0" max="100" value="80" oninput="sl('v_dac1_l',this.value)"><input type="number" id="v_dac1_l_i" min="0" max="100" value="80" onchange="si('v_dac1_l',this.value)"><span>%</span></div>
<div class="row"><label>&#128266; Mono Lautst.</label><input type="range" id="v_dac1_r" min="0" max="100" value="20" oninput="sl('v_dac1_r',this.value)"><input type="number" id="v_dac1_r_i" min="0" max="100" value="20" onchange="si('v_dac1_r',this.value)"><span>%</span></div>
<hr>
<div class="row"><label>Subsonic (Horn-Schutz)</label><input type="range" id="sub_sb" min="40" max="60" value="42" oninput="sl('sub_sb',this.value)"><input type="number" id="sub_sb_i" min="30" max="60" value="42" onchange="si('sub_sb',this.value)"><span>Hz</span></div>
<div class="row"><label>Tiefpass (Trennung)</label><input type="range" id="sub_lp" min="60" max="180" value="95" oninput="sl('sub_lp',this.value)"><input type="number" id="sub_lp_i" min="60" max="180" value="95" onchange="si('sub_lp',this.value)"><span>Hz</span></div>
<hr>
<div class="row"><label>Limiter Threshold</label><input type="range" id="lim_s_th" min="50" max="100" value="98" oninput="sl('lim_s_th',this.value)"><input type="number" id="lim_s_th_i" min="50" max="100" value="98" onchange="si('lim_s_th',this.value)"><span>%</span></div>
<div class="row"><label>Limiter Release</label><input type="range" id="lim_s_rl" min="20" max="500" value="150" oninput="sl('lim_s_rl',this.value)"><input type="number" id="lim_s_rl_i" min="20" max="500" value="150" onchange="si('lim_s_rl',this.value)"><span>ms</span></div>
</div>

<div class="card t2">
<h2>DAC 2 &ndash; Tops L / R</h2><hr>
<div class="row"><label>&#128266; Tops L Lautst.</label><input type="range" id="v_dac2_l" min="0" max="100" value="80" oninput="sl('v_dac2_l',this.value)"><input type="number" id="v_dac2_l_i" min="0" max="100" value="80" onchange="si('v_dac2_l',this.value)"><span>%</span></div>
<div class="row"><label>&#128266; Tops R Lautst.</label><input type="range" id="v_dac2_r" min="0" max="100" value="80" oninput="sl('v_dac2_r',this.value)"><input type="number" id="v_dac2_r_i" min="0" max="100" value="80" onchange="si('v_dac2_r',this.value)"><span>%</span></div>
<hr>
<div class="row"><label>Hochpass (Entlastung)</label><input type="range" id="tops_hp" min="60" max="150" value="120" oninput="sl('tops_hp',this.value)"><input type="number" id="tops_hp_i" min="60" max="150" value="120" onchange="si('tops_hp',this.value)"><span>Hz</span></div>
<div class="row"><label>Delay (Sync zum Sub)</label><input type="range" id="t_dly" min="0" max="30" step="0.1" value="0" oninput="sl('t_dly',this.value)"><input type="number" id="t_dly_i" min="0" max="30" step="0.1" value="0" onchange="si('t_dly',this.value)"><span>ms</span></div>
<hr>
<div class="row"><label>Limiter Threshold</label><input type="range" id="lim_t_th" min="50" max="100" value="98" oninput="sl('lim_t_th',this.value)"><input type="number" id="lim_t_th_i" min="50" max="100" value="98" onchange="si('lim_t_th',this.value)"><span>%</span></div>
<div class="row"><label>Limiter Release</label><input type="range" id="lim_t_rl" min="10" max="500" value="100" oninput="sl('lim_t_rl',this.value)"><input type="number" id="lim_t_rl_i" min="10" max="500" value="100" onchange="si('lim_t_rl',this.value)"><span>ms</span></div>
</div>

<button class="btn" onclick="switchBT()">&#127911; Auf Bluetooth wechseln (WLAN aus)</button>
<div id="msg"></div>
</div>
<script>
function sl(id,v){document.getElementById(id+'_i').value=v;fetch('/set?'+id+'='+v);}
function si(id,v){var s=document.getElementById(id);v=Math.min(Math.max(parseFloat(v)||0,parseFloat(s.min)),parseFloat(s.max));document.getElementById(id+'_i').value=v;s.value=v;fetch('/set?'+id+'='+v);}
function switchBT(){document.getElementById('msg').innerText='WLAN wird abgeschaltet, Bluetooth startet...';fetch('/bt');}
window.onload=function(){fetch('/status').then(function(r){return r.json();}).then(function(d){for(var k in d){var s=document.getElementById(k),i=document.getElementById(k+'_i');if(s)s.value=d[k];if(i)i.value=d[k];}}).catch(function(){});};}
</script></body></html>
)rawliteral";
#endif // ENABLE_WEBGUI

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

// Berechnet den float-Ausgabewert für einen konfigurierten DAC-Kanal.
// raw_l / raw_r = UNVERARBEITETE 16-Bit-Eingangswerte als float.
// Jede Filter-Instanz darf pro Sample-Frame nur einmal aufgerufen werden –
// denselben Modus also nie auf zwei Kanälen gleichzeitig konfigurieren.
static float compute_ch(int mode, float raw_l, float raw_r) {
    switch (mode) {
        case DAC_CH_SUB: {
            float m = 0.5f * (raw_l + raw_r);
            m = sub_sub_L2.process(sub_sub_L.process(m));
            return sub_lp_L2.process(sub_lp_L.process(m));
        }
        case DAC_CH_MONO:
            return 0.5f * (raw_l + raw_r);
        case DAC_CH_TOPS_L:
            return tops_hp_L2.process(tops_hp_L.process(raw_l));
        case DAC_CH_TOPS_R:
            return tops_hp_R2.process(tops_hp_R.process(raw_r));
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
    // Rohsignal für process_dac2 sichern – vor der In-Place-Modifikation
    s_raw_len = (len <= sizeof(s_raw_buf)) ? len : sizeof(s_raw_buf);
    memcpy(s_raw_buf, data, s_raw_len);

    int16_t *samples = (int16_t*)data;
    for (int i = 0; i < (int)(len / 2); i += 2) {
        float raw_l = (float)samples[i];
        float raw_r = (float)samples[i + 1];

        float l = compute_ch(DAC1_L_MODE, raw_l, raw_r) * vol_dac1_l;
        float r = compute_ch(DAC1_R_MODE, raw_l, raw_r) * vol_dac1_r;

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
static inline void sum_stereo_to_mono_inplace(uint8_t *data, size_t len) {
    int16_t *samples = (int16_t*)data;
    int count = (int)(len / 2);
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
    cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
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
    esp_err_t err = i2s_read(I2S_NUM_1, pcm1808_buf, sizeof(pcm1808_buf), &bytes_read, 0);
    if (err != ESP_OK || bytes_read < 4) return;

    // Erst summieren, dann wie gewohnt mit den bestehenden DSP-Einstellungen verarbeiten.
    sum_stereo_to_mono_inplace(pcm1808_buf, bytes_read);
    process_dac1(pcm1808_buf, bytes_read);
    i2s_tops->write(pcm1808_buf, bytes_read);
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

// Gemeinsame Parameter-Logik für die Web-Steuerung
void apply_param(const String& key, float val) {
    if (key == "sub_sb") {
        if (val >= 30.0) { sub_subsonic = val; sub_sub_L.begin(val, sample_rate); sub_sub_R.begin(val, sample_rate); sub_sub_L2.begin(val, sample_rate); sub_sub_R2.begin(val, sample_rate); }
    } else if (key == "sub_lp") {
        sub_lowpass = val;
        sub_lp_L.begin(sub_lowpass, sample_rate); sub_lp_R.begin(sub_lowpass, sample_rate);
        sub_lp_L2.begin(sub_lowpass, sample_rate); sub_lp_R2.begin(sub_lowpass, sample_rate);
    } else if (key == "tops_hp") {
        tops_highpass = val;
        tops_hp_L.begin(tops_highpass, sample_rate); tops_hp_R.begin(tops_highpass, sample_rate);
        tops_hp_L2.begin(tops_highpass, sample_rate); tops_hp_R2.begin(tops_highpass, sample_rate);
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
}

#if ENABLE_WEBGUI
// Liefert alle aktuellen DSP-Parameter als JSON (wird beim Seitenload abgerufen)
void handle_status() {
    last_web_activity = millis();
    char buf[320];
    snprintf(buf, sizeof(buf),
        "{\"sub_sb\":%.1f,\"sub_lp\":%.1f,\"tops_hp\":%.1f,\"t_dly\":%.1f,"
        "\"v_dac1_l\":%.0f,\"v_dac1_r\":%.0f,\"v_dac2_l\":%.0f,\"v_dac2_r\":%.0f,"
        "\"lim_s_th\":%.0f,\"lim_s_rl\":%.0f,\"lim_t_th\":%.0f,\"lim_t_rl\":%.0f}",
        sub_subsonic, sub_lowpass, tops_highpass, tops_delay_ms,
        vol_dac1_l * 100.0f, vol_dac1_r * 100.0f,
        vol_dac2_l * 100.0f, vol_dac2_r * 100.0f,
        lim_sub_thresh  / 32767.0f * 100.0f, lim_sub_rel,
        lim_tops_thresh / 32767.0f * 100.0f, lim_tops_rel);
    server.send(200, "application/json", buf);
}

void handle_update() {
    last_web_activity = millis();
    const char* keys[] = {
        "sub_sb","sub_lp","tops_hp","t_dly",
        "v_dac1_l","v_dac1_r","v_dac2_l","v_dac2_r",
        "v_tops","v_sub",
        "lim_s_th","lim_s_rl","lim_t_th","lim_t_rl"
    };
    for (auto k : keys)
        if (server.hasArg(k)) apply_param(k, server.arg(k).toFloat());
    server.send(200, "text/plain", "OK");
}
#endif // ENABLE_WEBGUI

// Bluetooth-A2DP starten (einmalig). Wird entweder direkt beim Boot aufgerufen
// (WebGUI aus) oder erst nach dem WiFi-Timeout (WebGUI an), damit WLAN und BT
// nie gleichzeitig Speicher belegen.
void start_bluetooth() {
#if !ENABLE_DAC2
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
#if ENABLE_DAC2
    Serial.printf("[6] new BluetoothA2DPSink  free heap: %lu\n", (unsigned long)ESP.getFreeHeap());
    a2dp_sink = new BluetoothA2DPSink(*multi_out);
#else
    Serial.printf("[6] PCM1808-LineIn-Modus  free heap: %lu\n", (unsigned long)ESP.getFreeHeap());
#endif

    // Limiter konfigurieren (Threshold ~ -0,2 dBFS Headroom).
    Serial.println("[7] limiter begin");
    lim_tops.begin(lim_tops_thresh, sample_rate, LIM_TOPS_ATTACK_MS, lim_tops_rel);
    lim_sub.begin(lim_sub_thresh,  sample_rate, LIM_SUB_ATTACK_MS,  lim_sub_rel);

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

#if ENABLE_DAC2
    // Verbindungs-Callback zum Debuggen
    a2dp_sink->set_on_connection_state_changed([](esp_a2d_connection_state_t state, void* ptr){
        Serial.printf("[BT] Connection state: %d (2=connected)\n", state);
        if (state == ESP_A2D_CONNECTION_STATE_CONNECTED)       pending_sound = 1;
        else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) pending_sound = 2;
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
}

// WiFi-AP + Webserver + DNS abschalten (Inaktivitäts-Timeout), dann Bluetooth an
#if ENABLE_WEBGUI
void shutdown_wifi() {
    Serial.println("WiFi-Timeout: AP + Webserver werden abgeschaltet");
    server.stop();
    dnsServer.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    wifi_active = false;
    Serial.println("WiFi aus -> Bluetooth wird gestartet");
    // Jetzt ist der WLAN-Speicher frei -> Bluetooth kann sauber starten
    start_bluetooth();
}
#endif

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
#if ENABLE_WEBGUI
    static bool wifi_active = true;
    if (wifi_active) {
        dnsServer.processNextRequest();
        server.handleClient();
        // GUI-Button gedrückt -> sofort auf Bluetooth wechseln
        if (bt_switch_requested) {
            delay(150);          // kurz warten, damit die HTTP-Antwort raus ist
            shutdown_wifi();
        } else if (millis() - last_web_activity > WIFI_TIMEOUT_MS) {
            shutdown_wifi();
        }
    }
#endif

#if !ENABLE_DAC2
    service_pcm1808_input();
#endif
}
