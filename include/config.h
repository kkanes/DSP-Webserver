#pragma once
// =====================================================================
//                      ZENTRALE KONFIGURATION
// Alle einstellbaren Werte an einem Ort. Nach Änderungen neu hochladen.
// =====================================================================

// --- Weboberfläche / WLAN-Access-Point -------------------------------
// 1 = WebGUI + AP aktiv, 0 = komplett aus (volle Funkzeit fürs Bluetooth)
#define ENABLE_WEBGUI          0
#define AP_SSID                "Kevaier-DSP"
#define AP_PASSWORD            "subwoofer123"
#define WIFI_TIMEOUT_SEC       5        // Sekunden bis WLAN bei Inaktivität abschaltet

// --- Bluetooth -------------------------------------------------------
#define BT_DEVICE_NAME         "Kevaier-Audio"

// --- Audio-Basis -----------------------------------------------------
#define SAMPLE_RATE            44100

// --- DAC 1 (Canton Tops) Pins ---------------------------------------
#define TOPS_PIN_BCK           14
#define TOPS_PIN_WS            27
#define TOPS_PIN_DATA          22

// --- DAC 2 (MTH-30 Sub) Pins ----------------------------------------
#define SUB_PIN_BCK            21
#define SUB_PIN_WS             17
#define SUB_PIN_DATA           16

// --- Crossover-Filter (Startwerte, per WebGUI zur Laufzeit änderbar) --
#define DEFAULT_SUB_SUBSONIC   42.0f    // Hz  Subsonic-Hochpass (Horn-Schutz)
#define DEFAULT_SUB_LOWPASS    95.0f    // Hz  Sub-Tiefpass (Trennfrequenz)
#define DEFAULT_TOPS_HIGHPASS  120.0f   // Hz  Tops-Hochpass

// --- Tops-Delay (Synchronisation zum Sub/Horn) -----------------------
#define DEFAULT_TOPS_DELAY_MS  5.0f     // ms

// --- Lautstärke (0.0 .. 1.0) ----------------------------------------
#define DEFAULT_VOL_TOPS       0.8f
#define DEFAULT_VOL_SUB        0.8f

// --- Koppel-/Entkoppel-Ton ------------------------------------------
// Lautstärke der Signaltöne (0.0 .. 1.0). Klein halten, damit der Ton beim
// Verbinden nicht zu laut ist. Gilt für Beep-Melodie und PCM-Clips.
#define SND_VOLUME             0.05f

// --- Limiter ---------------------------------------------------------
// Threshold in Sample-Einheiten (max. 32767), Release/Attack in ms.
#define LIM_TOPS_ATTACK_MS      1.0f
#define LIM_SUB_ATTACK_MS       5.0f
#define DEFAULT_LIM_TOPS_THRESH 32000.0f
#define DEFAULT_LIM_TOPS_REL    100.0f
#define DEFAULT_LIM_SUB_THRESH  32000.0f
#define DEFAULT_LIM_SUB_REL     150.0f
