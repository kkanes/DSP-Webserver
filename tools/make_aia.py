#!/usr/bin/env python3
import json, zipfile, io, os

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "DSP_Weiche.aia")
APP = "DSP_Weiche"
PKG = f"appinventor.ai_user.{APP}"

PARAMS = [
    ("v_dac1_l", "Sub Lautstaerke",    0,  100,  80, "%"),
    ("v_dac1_r", "Mono Lautstaerke",   0,  100,  20, "%"),
    ("v_dac2_l", "Tops L Lautstaerke", 0,  100,  80, "%"),
    ("v_dac2_r", "Tops R Lautstaerke", 0,  100,  80, "%"),
    ("sub_sb",   "Subsonic Hz",        40,  60,  42, "Hz"),
    ("sub_lp",   "Tiefpass Hz",        60, 120,  95, "Hz"),
    ("tops_hp",  "Hochpass Hz",        60, 150, 120, "Hz"),
    ("t_dly",    "Tops Delay ms",       0,  30,   0, "ms"),
    ("lim_s_th", "Sub Lim Thresh",     50, 100,  98, "%"),
    ("lim_s_rl", "Sub Lim Release",    20, 500, 150, "ms"),
    ("lim_t_th", "Tops Lim Thresh",    50, 100,  98, "%"),
    ("lim_t_rl", "Tops Lim Release",   10, 500, 100, "ms"),
]

PROPS = f"""main={PKG}.Screen1
name={APP}
assets=../assets
source=../src
build=../build
versioncode=1
versionname=1.0
useslocation=False
aname=DSP Weiche
sizing=Responsive
showlistsasjsonarray=True
actionbar=True
theme=AppTheme.Light.DarkActionBar
color.primary=&HFF00796B
color.primary.dark=&HFF004D40
color.accent=&HFFFF5722
color.primary.text=&HFFFFFFFF
color.icon=&HFFFFFFFF
"""

def make_scm():
    comps = [
        {"$Name":"BT","$Type":"BluetoothClient","$Version":"6"},
        {"$Name":"ClockRX","$Type":"Clock","$Version":"3","TimerEnabled":"False","TimerInterval":"100"},
        {"$Name":"BtnConnect","$Type":"ListPicker","$Version":"19","Text":"Verbinden","Width":"-2"},
        {"$Name":"LblStatus","$Type":"Label","$Version":"5","Text":"Nicht verbunden","Width":"-2"},
    ]
    for key,label,mn,mx,default,unit in PARAMS:
        comps.append({"$Name":f"Lbl_{key}","$Type":"Label","$Version":"5","FontBold":"True","Text":label,"Width":"-2"})
        comps.append({"$Name":f"HA_{key}","$Type":"HorizontalArrangement","$Version":"3","Width":"-2","$Components":[
            {"$Name":f"SL_{key}","$Type":"Slider","$Version":"1","MaxValue":str(float(mx)),"MinValue":str(float(mn)),"ThumbPosition":str(float(default)),"Width":"-2"},
            {"$Name":f"TB_{key}","$Type":"TextBox","$Version":"5","NumbersOnly":"True","Text":str(default),"Width":"80"},
            {"$Name":f"LblU_{key}","$Type":"Label","$Version":"5","Text":unit,"Width":"30"},
        ]})
    return json.dumps({"YaVersion":"2","Source":"Form","Properties":{"$Name":"Screen1","$Type":"Form","$Version":"17","AppName":"DSP Weiche","Scrollable":"True","Title":"DSP Weiche","$Components":comps}},ensure_ascii=True,indent=2)

def make_bky():
    return '<xml xmlns="https://developers.google.com/blockly/xml"></xml>'

buf = io.BytesIO()
with zipfile.ZipFile(buf,"w",zipfile.ZIP_DEFLATED) as z:
    pp = PKG.replace(".","/"
    z.writestr("youngandroidproject/project.properties",PROPS.encode("utf-8"))
    z.writestr(f"src/{pp}/Screen1.scm",make_scm().encode("utf-8"))
    z.writestr(f"src/{pp}/Screen1.bky",make_bky().encode("utf-8"))
with open(OUT,"wb") as f:
    f.write(buf.getvalue())
print(f"OK {OUT} ({os.path.getsize(OUT)} Bytes)")
