#pragma once

// ============================================================================
// OLED MENU VISUALIZATION (HW-787AB)
// ---------------------------------------------------------------------------
// This file documents the local OLED menu layout and control behavior.
// It is for visualization/documentation only and does not contain runtime code.
// ============================================================================

/*
CONTROL MODEL
-------------
- Rotary turn:
  - Home screen: master volume up/down (display shows only this value)
  - NAV mode: move selection up/down through items
  - EDIT mode: change current value by item step size

- Confirm button:
  - Home screen -> enter menu NAV mode
  - In menu: toggle NAV <-> EDIT mode for selected item

- Encoder push button (separate pin):
  - Same behavior as Confirm (ENTER)

- Back button:
  - In EDIT mode: leave EDIT mode (keep value)
  - In NAV mode: return to Home screen (master volume)

- OLED sleep timer:
  - Display is dimmed first after OLED_DIM_TIMEOUT_SEC without input
  - Display powers down after OLED_SLEEP_TIMEOUT_SEC without input
  - Any rotary movement or button press wakes display immediately

Config flags that affect behavior:
- ROTARY_INVERT_DIR      (0/1)
- MENU_BUTTON_ACTIVE_LOW (1=active low, 0=active high)
- BTN_ENCODER_PUSH_PIN   (extra ENTER input; can be same pin as Confirm)
*/

/*
MENU TREE (single page list, scrolls in firmware)
--------------------------------------------------
Home
`- Master Vol    [0..100 %]      step MASTER_VOL_STEP_PCT

DSP Menu
|- Profile       [MTH30+TOP / Z2300+SAT / USER]
|- Subsonic      [30..60 Hz]      step 1
|- Sub LP        [60..180 Hz]     step 1
|- Tops HP       [60..150 Hz]     step 1
|- Delay         [0.0..30.0 ms]   step 0.1
|- Vol Sub       [0..100 %]       step 1
|- Vol Mono      [0..100 %]       step 1
|- Vol Tops L    [0..100 %]       step 1
|- Vol Tops R    [0..100 %]       step 1
|- Lim Sub Th    [50..100 %]      step 1
|- Lim Sub Rel   [20..500 ms]     step 5
|- Lim Top Th    [50..100 %]      step 1
`- Lim Top Rel   [10..500 ms]     step 5
*/

/*
DAC / VOLUME ORIENTATION (important in practice)
-------------------------------------------------
Default routing from config.h in this project:
- DAC1_L_MODE = DAC_CH_SUB   (Subwoofer path)
- DAC1_R_MODE = DAC_CH_MONO  (summed mono path)
- DAC2_L_MODE = DAC_CH_TOPS_L
- DAC2_R_MODE = DAC_CH_TOPS_R

Physical I2S outputs:
- DAC1 stream uses TOPS_PIN_*  (BCK/WS/DATA)
- DAC2 stream uses SUB_PIN_*   (BCK/WS/DATA), only when ENABLE_DAC2 = 1

Meaning of volume menu entries:
- Vol Sub     = v_dac1_l = DAC1 left output level (Sub path)
- Vol Mono    = v_dac1_r = DAC1 right output level (Mono path)
- Vol Tops L  = v_dac2_l = DAC2 left output level (Tops L)
- Vol Tops R  = v_dac2_r = DAC2 right output level (Tops R)

Mode note:
- If ENABLE_DAC2 = 1: all four volume items are active on real outputs.
- If ENABLE_DAC2 = 0 (PCM1808 line-in mode): no physical DAC2 output exists,
  therefore Vol Tops L/R change internal values but do not drive a DAC2 channel.
*/

/*
PARAMETER MAPPING (OLED -> apply_param key)
--------------------------------------------
- Profile       -> internal profile switch (not apply_param)
- Subsonic      -> "sub_sb"
- Sub LP        -> "sub_lp"
- Tops HP       -> "tops_hp"
- Delay         -> "t_dly"
- Vol Sub       -> "v_dac1_l"
- Vol Mono      -> "v_dac1_r"
- Vol Tops L    -> "v_dac2_l"
- Vol Tops R    -> "v_dac2_r"
- Lim Sub Th    -> "lim_s_th"
- Lim Sub Rel   -> "lim_s_rl"
- Lim Top Th    -> "lim_t_th"
- Lim Top Rel   -> "lim_t_rl"

Note:
All OLED edits call apply_param(), so OLED, WebGUI and BT config share
exactly the same DSP runtime path.

Source priority note (ENABLE_DAC2=0):
- If ENABLE_BT_FALLBACK_WITH_AUX=1, Bluetooth remains available as fallback.
- Detected AUX signal (PCM1808) has priority and fades BT out.
- AUX detection tuning: AUX_ACTIVITY_ON_THRESHOLD, AUX_ACTIVITY_OFF_THRESHOLD,
  AUX_PRIORITY_HOLD_MS and AUX_BT_XFADE_STEP in config.h.

Profile note:
- MTH30+TOP and Z2300+SAT load fixed presets.
- USER is persistent in NVS and is updated when parameters are changed manually.
*/
