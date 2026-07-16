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

// --- Bluetooth-Konfiguration per SPP ---------------------------------
// 1 = Einstellungen per Classic-BT-Serial (SPP) vom Handy aus ändern.
// Passende App: "Serial Bluetooth Terminal" (Android, kostenlos).
// Verbinden mit "Kevaier-Audio", dann Befehle senden:
//   key=wert     z.B.  v_dac1_l=75   oder   sub_lp=90
//   status       alle aktuellen Werte ausgeben
//   help         Befehlsliste anzeigen
// Hinweis: läuft gleichzeitig mit A2DP auf demselben BT-Chip.
#define ENABLE_BT_CONFIG       1

// --- Audio-Basis -----------------------------------------------------
#define SAMPLE_RATE            44100

// --- DAC 1 (Canton Tops) Pins ---------------------------------------
#define TOPS_PIN_BCK           14
#define TOPS_PIN_WS            27
#define TOPS_PIN_DATA          22

// --- DAC 2 / PCM1808 Pins -------------------------------------------
// Bei ENABLE_DAC2=1: I2S-Ausgang fuer DAC2 (z.B. Tops/Sub je nach Routing)
// Bei ENABLE_DAC2=0: I2S-Eingang vom PCM1808 (Stereo Line-In)
#define SUB_PIN_BCK            21
#define SUB_PIN_WS             17
#define SUB_PIN_DATA           16

// --- DAC 2 aktivieren ------------------------------------------------
// 1 = DAC 2 (SUB_PIN_*) aktiv als I2S-Ausgang  (PCM5102 o.ä.)
// 0 = DAC 2 deaktiviert, stattdessen PCM1808-Line-In auf SUB_PIN_*.
//     Das PCM1808-Stereo-Signal wird zu Mono summiert und mit den
//     vorhandenen DSP-Einstellungen weiterverarbeitet.
#define ENABLE_DAC2            0

// --- Crossover-Filter (Startwerte, per WebGUI zur Laufzeit änderbar) --
#define DEFAULT_SUB_SUBSONIC   43.0f    // Hz  Subsonic-Hochpass (Horn-Schutz)
#define DEFAULT_SUB_LOWPASS    95.0f    // Hz  Sub-Tiefpass (Trennfrequenz)
#define DEFAULT_TOPS_HIGHPASS  95.0f   // Hz  Tops-Hochpass

// --- Tops-Delay (Synchronisation zum Sub/Horn) -----------------------
#define DEFAULT_TOPS_DELAY_MS  0.0f     // ms

// --- Lautstärke pro Kanal (0.0 .. 1.0) ------------------------------
// Jeder der vier DAC-Ausgänge hat einen eigenen Lautstärkewert.
// Zur Laufzeit per WebGUI oder apply_param() änderbar.
#define DEFAULT_VOL_DAC1_L     0.2f    // DAC1 L  (aktuell: Subwoofer)
#define DEFAULT_VOL_DAC1_R     0.2f    // DAC1 R  (aktuell: Stereo-Mono)
#define DEFAULT_VOL_DAC2_L     0.8f    // DAC2 L  (aktuell: Tops L)
#define DEFAULT_VOL_DAC2_R     0.8f    // DAC2 R  (aktuell: Tops R)

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

// --- DAC-Kanal-Routing -----------------------------------------------
// Legt fest, welches Signal auf jedem der vier DAC-Ausgänge liegt.
// Beide DACs haben je einen L- und R-Kanal.
//
// Mögliche Werte:
//   DAC_CH_OFF     – Stille
//   DAC_CH_SUB     – Subwoofer (Tiefpass + Subsonic, Mono-Mix, vol_sub)
//   DAC_CH_MONO    – Stereo auf Mono gemischt, kein Filter  (vol_tops)
//   DAC_CH_TOPS_L  – Hochpass links  (vol_tops)
//   DAC_CH_TOPS_R  – Hochpass rechts (vol_tops)
//   DAC_CH_LEFT    – Eingang L, ungefiltert
//   DAC_CH_RIGHT   – Eingang R, ungefiltert
//
// Hinweis: Jede Filter-Instanz darf pro Sample-Frame nur EINMAL
// aufgerufen werden. DAC_CH_SUB oder DAC_CH_TOPS_L/R also nicht
// mehrfach parallel belegen.
#define DAC_CH_OFF     0
#define DAC_CH_SUB     1
#define DAC_CH_MONO    2
#define DAC_CH_TOPS_L  3
#define DAC_CH_TOPS_R  4
#define DAC_CH_LEFT    5
#define DAC_CH_RIGHT   6

// DAC 1 (Pins TOPS_PIN_*): Kanal-Belegung
#define DAC1_L_MODE    DAC_CH_SUB     // L = Subwoofer
#define DAC1_R_MODE    DAC_CH_MONO    // R = Stereo auf Mono

// DAC 2 (Pins SUB_PIN_*): Kanal-Belegung
#define DAC2_L_MODE    DAC_CH_TOPS_L
#define DAC2_R_MODE    DAC_CH_TOPS_R
