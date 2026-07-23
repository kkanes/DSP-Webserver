#pragma once
// =====================================================================
//                      ZENTRALE KONFIGURATION
// Alle einstellbaren Werte an einem Ort. Nach Änderungen neu hochladen.
// =====================================================================

//DSP-Version
// für esp-32 

// --- Debug -------------------------------------------------------
#ifndef DEBUG_EVENTS
#define DEBUG_EVENTS 0
#endif
#if DEBUG_EVENTS
#define DEBUG_LOG(fmt, ...) Serial.printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)
#else
#define DEBUG_LOG(fmt, ...) do {} while(0)
#endif

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

// --- OLED + Rotary-Menue (HW-787AB) ----------------------------------
// 1 = lokales Menue auf 1.3" OLED aktiv (I2C + Rotary + Back/Confirm)
// 0 = Menue deaktiviert
// Menue-Dokumentation: siehe include/menu_visualization.h
#define ENABLE_OLED_MENU       1

// OLED (SH1106 128x64, I2C) -- Hersteller-Aufdruck lautet "SH1306" (Tippfehler)
#define OLED_I2C_ADDR          0x3C
#define OLED_PIN_SDA           18
#define OLED_PIN_SCL           19

// Rotary-Encoder (HC11): A/B fuer Drehung
#define ROTARY_PIN_A           25
#define ROTARY_PIN_B           26

// Drehrichtung invertieren (falls Links/Rechts vertauscht ist)
#define ROTARY_INVERT_DIR      0

// Tasten (active low gegen GND)
#define BTN_BACK_PIN           4
#define BTN_CONFIRM_PIN        5
// Extra Encoder-Push (separater Taster am Encoder)
// Standard: gleich Confirm, damit bestehende Verdrahtung unveraendert bleibt.
#define BTN_ENCODER_PUSH_PIN   BTN_CONFIRM_PIN

// 1 = Taster aktiv LOW, 0 = Taster aktiv HIGH
#define MENU_BUTTON_ACTIVE_LOW 1

// OLED schlaeft nach Inaktivitaet ein (Sekunden). Drehung/Tastendruck weckt.
#define OLED_SLEEP_TIMEOUT_SEC 3600

// Vor dem Sleep erst dimmen (Sekunden ohne Eingabe)
#define OLED_DIM_TIMEOUT_SEC   3600
#define OLED_ACTIVE_CONTRAST   255
#define OLED_DIM_CONTRAST      24

// Schrittweite fuer Master-Volume bei Drehung (in Prozentpunkten)
#define MASTER_VOL_STEP_PCT    1.0f

// Sicherheits-Startlautstaerke nach Power-On/Reset (in %)
// Gilt immer beim Boot, unabhaengig von OLED/Web/BT.
#define STARTUP_MASTER_VOL_PCT 15.0f

// --- Analoge Lautstaerke-Potis fuer DAC1 (Sub/Mono) -------------------
// 1 = zwei Drehpotis regeln v_dac1_l/v_dac1_r direkt (per ADC ausgelesen).
// Ueberschreibt WebGUI/BT/OLED-Werte, sobald am Poti gedreht wird.
#define ENABLE_POT_VOLUME      1
#define POT_PIN_DAC1_L         34   // ADC1_CH6, input-only, kein WLAN/BT-Konflikt
#define POT_PIN_DAC1_R         35   // ADC1_CH7, input-only, kein WLAN/BT-Konflikt

// Glaettung des ADC-Rohwerts (0..1, kleiner = traeger/glatter, groesser = direkter)
#define POT_SMOOTHING_ALPHA    0.1f
// Mindestaenderung in %, bevor ein neuer Wert uebernommen wird (Rauschunterdrueckung)
#define POT_DEADBAND_PCT       0.5f
// Wartezeit ohne weitere Bewegung, bis der Wert im NVS gespeichert wird (ms)
#define POT_SAVE_DEBOUNCE_MS   1500
// Potentiometer-Kontrolle aktivieren (1 = Potis regeln Volumes, 0 = nur Bluetooth)
#define ENABLE_POTI_CONTROL    0

// --- Profile ---------------------------------------------------------
// 0 = MTH30+TOP, 1 = Z2300+SAT, 2 = USER
#define PROFILE_MTH30_TOP      0
#define PROFILE_Z2300_SAT      1
#define PROFILE_USER           2

// Welches Profil nach erstem Flashen bzw. ohne NVS-Daten aktiv sein soll
#define DEFAULT_PROFILE_ID     PROFILE_Z2300_SAT

// Preset: MTH30 + TOP
#define P_MTH30_SUB_SB         43.0f
#define P_MTH30_SUB_LP         95.0f
#define P_MTH30_TOPS_HP        95.0f
#define P_MTH30_T_DLY          0.0f
#define P_MTH30_V_DAC1_L       80.0f
#define P_MTH30_V_DAC1_R       20.0f
#define P_MTH30_V_DAC2_L       80.0f
#define P_MTH30_V_DAC2_R       80.0f
#define P_MTH30_LIM_S_TH       98.0f
#define P_MTH30_LIM_S_RL       150.0f
#define P_MTH30_LIM_T_TH       98.0f
#define P_MTH30_LIM_T_RL       100.0f
#define P_MTH30_MASTER_VOL     STARTUP_MASTER_VOL_PCT

// Preset: Z2300 + SAT
#define P_Z2300_SUB_SB         35.0f
#define P_Z2300_SUB_LP         100.0f
#define P_Z2300_TOPS_HP        100.0f
#define P_Z2300_T_DLY          0.0f
#define P_Z2300_V_DAC1_L       80.0f
#define P_Z2300_V_DAC1_R       20.0f
#define P_Z2300_V_DAC2_L       60.0f
#define P_Z2300_V_DAC2_R       60.0f
#define P_Z2300_LIM_S_TH       92.0f
#define P_Z2300_LIM_S_RL       120.0f
#define P_Z2300_LIM_T_TH       95.0f
#define P_Z2300_LIM_T_RL       90.0f

/*
#define LIM_TOPS_ATTACK_MS      2.0f
#define LIM_SUB_ATTACK_MS       20.0f
#define DEFAULT_LIM_TOPS_THRESH 17560.0f
#define DEFAULT_LIM_TOPS_REL    30.0f
#define DEFAULT_LIM_SUB_THRESH  2100.0f 
#define DEFAULT_LIM_SUB_REL     200.0f
*/
#define P_Z2300_MASTER_VOL     STARTUP_MASTER_VOL_PCT

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

// --- AUX/BT Quellen-Prioritaet (nur bei ENABLE_DAC2=0) --------------
// 1 = Bluetooth als Fallback aktiv, aber AUX hat Vorrang bei erkanntem Signal
// 0 = kein Bluetooth im PCM1808-Modus
#define ENABLE_BT_FALLBACK_WITH_AUX 1

// Schwellwerte fuer AUX-Signalerkennung (Hysterese, 16-bit Peak)
// ON: ab diesem Peak wird AUX aktiv. OFF: unter diesem Peak kann AUX auslaufen.
#define AUX_ACTIVITY_ON_THRESHOLD   8000
#define AUX_ACTIVITY_OFF_THRESHOLD  4000

// Haltezeit fuer AUX-Prioritaet nach letztem erkannten Signal (ms)
#define AUX_PRIORITY_HOLD_MS    1200

// Crossfade Schritt pro Audioblock (0.01 = langsam, 0.2 = sehr schnell)
#define AUX_BT_XFADE_STEP       0.08f

// --- Crossover-Filter (Startwerte, per WebGUI zur Laufzeit änderbar) --
#define DEFAULT_SUB_SUBSONIC   43.0f    // Hz  Subsonic-Hochpass (Horn-Schutz)
#define DEFAULT_SUB_LOWPASS    95.0f    // Hz  Sub-Tiefpass (Trennfrequenz)
#define DEFAULT_TOPS_HIGHPASS  95.0f   // Hz  Tops-Hochpass

// --- Filter-Typ und Ordnung --------------------------------------------------
// FILTER_TYPE: 0=Butterworth (flach), 1=Bessel (minimum phase), 2=Chebyshev (steil)
#define FILTER_TYPE 0

// FILTER_ORDER: 0=LR12 (12dB/Okt), 1=LR24 (24dB/Okt), 2=LR48 (48dB/Okt)
#define FILTER_ORDER 1

// --- Tops-Delay (Synchronisation zum Sub/Horn) -----------------------
#define DEFAULT_TOPS_DELAY_MS  0.0f     // ms

// --- Lautstärke pro Kanal (0.0 .. 1.0) ------------------------------
// Jeder der vier DAC-Ausgänge hat einen eigenen Lautstärkewert.
// Zur Laufzeit per WebGUI oder apply_param() änderbar.
#define DEFAULT_VOL_DAC1_L     0.8f    // DAC1 L  (aktuell: Subwoofer)
#define DEFAULT_VOL_DAC1_R     0.2f    // DAC1 R  (aktuell: Stereo-Mono)
#define DEFAULT_VOL_DAC2_L     0.8f    // DAC2 L  (aktuell: Tops L)
#define DEFAULT_VOL_DAC2_R     0.8f    // DAC2 R  (aktuell: Tops R)

// --- Koppel-/Entkoppel-Ton ------------------------------------------
// Lautstärke der Signaltöne (0.0 .. 1.0). Klein halten, damit der Ton beim
// Verbinden nicht zu laut ist. Gilt für Beep-Melodie und PCM-Clips.
#define SND_VOLUME             0.005f

// --- Limiter ---------------------------------------------------------
// Threshold in Sample-Einheiten (max. 32767), Release/Attack in ms.
#define LIM_TOPS_ATTACK_MS      2.0f
#define LIM_SUB_ATTACK_MS       20.0f
#define DEFAULT_LIM_TOPS_THRESH 17560.0f
#define DEFAULT_LIM_TOPS_REL    30.0f
#define DEFAULT_LIM_SUB_THRESH  2100.0f
#define DEFAULT_LIM_SUB_REL     200.0f

//LIMTIER FÜR CANTON TOPS mit z-2300 SUB
/*
// --- Limiter ---------------------------------------------------------
// Threshold in Sample-Einheiten (max. 32767), Release/Attack in ms.

// TOPTEILE (2x Canton Plus GX.3 parallel an Kanal 1+2)
#define LIM_TOPS_ATTACK_MS      2.0f      // Schneller Schutz für die Hochtöner
#define DEFAULT_LIM_TOPS_REL    30.0f     // Schnelles Öffnen für natürlichen Klang
#define DEFAULT_LIM_TOPS_THRESH 17560.0f  // Begrenzt auf sichere ~50W pro Box an 2 Ohm

// SUBWOOFER (Logitech Z-2300 Chassis an einem einzelnen 8-Ohm-Kanal)
#define LIM_SUB_ATTACK_MS       20.0f     // Schützt vor Verzerrungen bei 50Hz Low-Cut
#define DEFAULT_LIM_SUB_REL     200.0f    // Verhindert das "Bass-Pumpen"
#define DEFAULT_LIM_SUB_THRESH  21000.0f  // Riegelt ab, bevor der einzelne Kanal ins Clipping gerät (~35-40W)

*/

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
