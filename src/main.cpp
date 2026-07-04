#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include "esp_coexist.h"
#include "config.h"   // Zentrale Konfiguration (Pins, WebGUI-Schalter, Defaults)
#include "sounds.h"   // Optionale Sprach-/Audio-Clips (WAV im Flash)

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
bool wifi_active = (ENABLE_WEBGUI != 0);   // WiFi/AP/Webserver nur wenn WebGUI aktiv
bool bt_started = false;                    // Bluetooth erst nach WiFi-Timeout starten
volatile bool bt_switch_requested = false;  // GUI-Button: sofort auf BT wechseln

// Koppel-/Entkoppel-Sound: wird in der BT-Callback nur als Flag gesetzt und
// im loop() abgespielt (nicht im Callback-Kontext, das wäre unsicher).
// 0 = nichts, 1 = verbunden (Koppel), 2 = getrennt (Entkoppel)
volatile int pending_sound = 0;

// Audio-Objekte als Pointer – Konstruktoren allozieren Heap,
// dürfen daher NICHT als globale Objekte initialisiert werden (Static Init Fiasco)
I2SStream*          i2s_tops  = nullptr;
I2SStream*          i2s_sub   = nullptr;
MultiOutput*        multi_out = nullptr;
BluetoothA2DPSink*  a2dp_sink = nullptr;
CallbackStream*     cb_tops   = nullptr;
CallbackStream*     cb_sub    = nullptr;

// Globale DSP-Filtervariablen
float sub_subsonic = DEFAULT_SUB_SUBSONIC;
float sub_lowpass  = DEFAULT_SUB_LOWPASS;
float tops_highpass = DEFAULT_TOPS_HIGHPASS;
const float sample_rate = SAMPLE_RATE;

// Tops-Delay (zur Laufzeit-Synchronisation mit dem Horn/Sub)
float tops_delay_ms = DEFAULT_TOPS_DELAY_MS;
const int TOPS_DELAY_MAX = 1400;              // ~31.7 ms @ 44.1 kHz
float tops_dly_L[TOPS_DELAY_MAX] = {0};
float tops_dly_R[TOPS_DELAY_MAX] = {0};
int tops_dly_idx = 0;
volatile int tops_delay_samples = 0;

// Globale Lautstärkefaktoren (0.0 bis 1.0)
float vol_tops = DEFAULT_VOL_TOPS;
float vol_sub  = DEFAULT_VOL_SUB;

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

// Erweitertes Web-GUI mit Audio- und Lautstärkereglern
const char HTML_GUI[] = R"rawliteral(
<!DOCTYPE html><html>
<head><meta name="viewport" content="width=device-width, initial-scale=1">
<title>3-Channel DSP Controller</title>
<style>
  body { font-family: 'Segoe UI', Arial, sans-serif; text-align: center; background: #121212; color: #fff; padding: 15px; margin: 0; }
  .container { max-width: 450px; margin: auto; }
  .card { background: #1e1e1e; padding: 20px; border-radius: 15px; margin-bottom: 20px; box-shadow: 0 4px 15px rgba(0,0,0,0.5); border-left: 5px solid #00adb5; }
  .card.tops { border-left-color: #ff5722; }
  h2 { color: #00adb5; margin-top: 0; font-size: 1.4em; }
  .card.tops h2 { color: #ff5722; }
  p { margin: 15px 0 5px 0; font-weight: bold; font-size: 1em; color: #ccc; }
  .vol-title { color: #ffeb3b !important; }
  input[type=range] { width: 90%; margin: 10px 0; accent-color: #00adb5; }
  .card.tops input[type=range] { accent-color: #ff5722; }
  .val { font-size: 1.2em; font-weight: bold; color: #fff; background: #2a2a2a; display: inline-block; padding: 3px 12px; border-radius: 8px; }
</style>
</head>
<body>
<div class="container">
  <h1 style="font-size: 1.6em; margin: 15px 0; color: #eee;">🎛️ 3-Kanal DSP Weiche</h1>
  
  <!-- SUBWOOFER CARD -->
  <div class="card">
    <h2>MTH-30 Subwoofer</h2>
    <hr style="border-color:#333;">
    <p class="vol-title">🔈 Subwoofer Lautstärke</p>
    <input type="range" id="v_sub" min="0" max="100" value="80" onchange="update('v_sub', this.value)">
    <div class="val"><span id="v_sub_val">80</span> %</div>
    
    <p>Subsonic Filter (Horn-Schutz)</p>
    <input type="range" id="sub_sb" min="40" max="60" value="42" onchange="update('sub_sb', this.value)">
    <div class="val"><span id="sub_sb_val">42</span> Hz</div>
    
    <p>Low-Pass (Trennfrequenz)</p>
    <input type="range" id="sub_lp" min="60" max="120" value="80" onchange="update('sub_lp', this.value)">
    <div class="val"><span id="sub_lp_val">80</span> Hz</div>

    <p>Limiter Threshold</p>
    <input type="range" id="lim_s_th" min="50" max="100" value="98" onchange="update('lim_s_th', this.value)">
    <div class="val"><span id="lim_s_th_val">98</span> %</div>

    <p>Limiter Release</p>
    <input type="range" id="lim_s_rl" min="20" max="500" value="150" onchange="update('lim_s_rl', this.value)">
    <div class="val"><span id="lim_s_rl_val">150</span> ms</div>
  </div>

  <!-- TOPS CARD -->
  <div class="card tops">
    <h2>Canton Tops (Stereo)</h2>
    <hr style="border-color:#333;">
    <p class="vol-title">🔈 Tops Lautstärke</p>
    <input type="range" id="v_tops" min="0" max="100" value="80" onchange="update('v_tops', this.value)">
    <div class="val"><span id="v_tops_val">80</span> %</div>
    
    <p>High-Pass (Entlastung)</p>
    <input type="range" id="tops_hp" min="60" max="150" value="80" onchange="update('tops_hp', this.value)">
    <div class="val"><span id="tops_hp_val">80</span> Hz</div>

    <p>Delay (Sync zum Sub/Horn)</p>
    <input type="range" id="t_dly" min="0" max="30" step="0.1" value="0" onchange="update('t_dly', this.value)">
    <div class="val"><span id="t_dly_val">0</span> ms</div>

    <p>Limiter Threshold</p>
    <input type="range" id="lim_t_th" min="50" max="100" value="98" onchange="update('lim_t_th', this.value)">
    <div class="val"><span id="lim_t_th_val">98</span> %</div>

    <p>Limiter Release</p>
    <input type="range" id="lim_t_rl" min="10" max="500" value="100" onchange="update('lim_t_rl', this.value)">
    <div class="val"><span id="lim_t_rl_val">100</span> ms</div>
  </div>

  <button onclick="switchBT()" style="width:100%;padding:15px;font-size:1.1em;font-weight:bold;color:#fff;background:#0066cc;border:none;border-radius:12px;cursor:pointer;box-shadow:0 4px 15px rgba(0,0,0,0.5);">
    🎧 Auf Bluetooth wechseln (WLAN aus)
  </button>
  <p id="bt_msg" style="color:#00adb5;min-height:1.2em;"></p>
</div>

<script>
function update(type, val) {
  document.getElementById(type + '_val').innerText = val;
  fetch('/set?' + type + '=' + val);
}
function switchBT() {
  document.getElementById('bt_msg').innerText = 'WLAN wird abgeschaltet, Bluetooth startet...';
  fetch('/bt');
}
</script>
</body></html>
)rawliteral";

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

// Audio-Verarbeitung für die Tops (DAC 1) inkl. Lautstärkeregelung
size_t process_tops(uint8_t *data, size_t len) {
    int16_t *samples = (int16_t*) data;
    for (int i = 0; i < len / 2; i += 2) {
        float l = tops_hp_L2.process(tops_hp_L.process(samples[i])) * vol_tops;
        float r = tops_hp_R2.process(tops_hp_R.process(samples[i + 1])) * vol_tops;

        // Delay-Line (Ringpuffer) zur Synchronisation mit dem Sub/Horn
        tops_dly_L[tops_dly_idx] = l;
        tops_dly_R[tops_dly_idx] = r;
        int rd = tops_dly_idx - tops_delay_samples;
        if (rd < 0) rd += TOPS_DELAY_MAX;
        l = tops_dly_L[rd];
        r = tops_dly_R[rd];
        tops_dly_idx = (tops_dly_idx + 1) % TOPS_DELAY_MAX;

        // Limiter: gemeinsamer Gain aus der lauteren Kanalspitze (erhält Stereobild)
        float g = lim_tops.compute(fmaxf(fabsf(l), fabsf(r)));
        l *= g;
        r *= g;

        // Werte begrenzen (Clipping-Schutz als letzte Reserve)
        samples[i]     = (int16_t)constrain(l, -32768, 32767);
        samples[i + 1] = (int16_t)constrain(r, -32768, 32767);
    }
    return len;
}

// Audio-Verarbeitung für den Subwoofer (DAC 2) inkl. Lautstärkeregelung
size_t process_sub(uint8_t *data, size_t len) {
    int16_t *samples = (int16_t*) data;
    for (int i = 0; i < len / 2; i += 2) {
        // Sub = MONO: beide Eingangskanäle zu einem Signal mischen (0,5·(L+R)
        // gegen Clipping), dann gemeinsam filtern und auf beide DAC-Kanäle legen.
        float mono_in = 0.5f * ((float)samples[i] + (float)samples[i + 1]);
        float raw = sub_sub_L2.process(sub_sub_L.process(mono_in));
        float m = sub_lp_L2.process(sub_lp_L.process(raw)) * vol_sub;

        // Limiter zum Schutz des Horns/Subs vor Übersteuerung
        m *= lim_sub.compute(fabsf(m));

        int16_t out = (int16_t)constrain(m, -32768, 32767);
        samples[i]     = out;
        samples[i + 1] = out;
    }
    return len;
}

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
        if (val >= 40.0) { sub_subsonic = val; sub_sub_L.begin(val, sample_rate); sub_sub_R.begin(val, sample_rate); sub_sub_L2.begin(val, sample_rate); sub_sub_R2.begin(val, sample_rate); }
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
    } else if (key == "v_tops") {
        vol_tops = val / 100.0;
    } else if (key == "v_sub") {
        vol_sub = val / 100.0;
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

void handle_update() {
    last_web_activity = millis();
    if (server.hasArg("sub_sb")) apply_param("sub_sb", server.arg("sub_sb").toFloat());
    if (server.hasArg("sub_lp")) apply_param("sub_lp", server.arg("sub_lp").toFloat());
    if (server.hasArg("tops_hp")) apply_param("tops_hp", server.arg("tops_hp").toFloat());
    if (server.hasArg("t_dly")) apply_param("t_dly", server.arg("t_dly").toFloat());
    if (server.hasArg("v_tops")) apply_param("v_tops", server.arg("v_tops").toFloat());
    if (server.hasArg("v_sub")) apply_param("v_sub", server.arg("v_sub").toFloat());
    if (server.hasArg("lim_s_th")) apply_param("lim_s_th", server.arg("lim_s_th").toFloat());
    if (server.hasArg("lim_s_rl")) apply_param("lim_s_rl", server.arg("lim_s_rl").toFloat());
    if (server.hasArg("lim_t_th")) apply_param("lim_t_th", server.arg("lim_t_th").toFloat());
    if (server.hasArg("lim_t_rl")) apply_param("lim_t_rl", server.arg("lim_t_rl").toFloat());
    server.send(200, "text/plain", "OK");
}

// Bluetooth-A2DP starten (einmalig). Wird entweder direkt beim Boot aufgerufen
// (WebGUI aus) oder erst nach dem WiFi-Timeout (WebGUI an), damit WLAN und BT
// nie gleichzeitig Speicher belegen.
void start_bluetooth() {
    if (bt_started) return;
    Serial.println("Starting Bluetooth...");
    a2dp_sink->start(BT_DEVICE_NAME);
    bt_started = true;
    Serial.println("Bluetooth started");
}

void setup() {
    Serial.begin(115200);

    // Audio-Objekte hier auf dem Heap erzeugen (Heap ist jetzt initialisiert)
    i2s_tops  = new I2SStream();
    i2s_sub   = new I2SStream();
    multi_out = new MultiOutput();
    cb_tops   = new CallbackStream(*i2s_tops, process_tops);
    cb_sub    = new CallbackStream(*i2s_sub,  process_sub);
    a2dp_sink = new BluetoothA2DPSink(*multi_out);

    // Limiter konfigurieren (Threshold ~ -0,2 dBFS Headroom).
    // Tops: schneller Attack, mittlerer Release. Sub: etwas trägere Zeiten,
    // damit tiefe Bässe nicht "pumpen".
    lim_tops.begin(lim_tops_thresh, sample_rate, LIM_TOPS_ATTACK_MS, lim_tops_rel);
    lim_sub.begin(lim_sub_thresh,  sample_rate, LIM_SUB_ATTACK_MS,  lim_sub_rel);

    // DAC 1 Pins (Canton Tops)
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

    // DAC 2 Pins (MTH-30 Sub)
    auto config_sub = i2s_sub->defaultConfig();
    config_sub.port_no    = 1;
    config_sub.sample_rate = 44100;
    config_sub.bits_per_sample = 16;
    config_sub.channels   = 2;
    config_sub.pin_bck  = SUB_PIN_BCK; config_sub.pin_ws = SUB_PIN_WS; config_sub.pin_data = SUB_PIN_DATA;
    config_sub.buffer_count = 8;
    config_sub.buffer_size  = 512;
    i2s_sub->begin(config_sub);

    multi_out->add(*cb_tops);
    multi_out->add(*cb_sub);

    // Verbindungs-Callback zum Debuggen
    a2dp_sink->set_on_connection_state_changed([](esp_a2d_connection_state_t state, void* ptr){
        Serial.printf("[BT] Connection state: %d (2=connected)\n", state);
        if (state == ESP_A2D_CONNECTION_STATE_CONNECTED)       pending_sound = 1;
        else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) pending_sound = 2;
    });
    a2dp_sink->set_on_audio_state_changed([](esp_a2d_audio_state_t state, void* ptr){
        Serial.printf("[BT] Audio state: %d (1=started)\n", state);
    });

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
}
