#pragma once
#include <stdint.h>

// ============================================================================
//  Sprach-/Audio-Clips als PCM im Flash (16-bit MONO @ 44.1 kHz)
// ----------------------------------------------------------------------------
//  So bindest du eigene Ansagen ("Verbunden" / "Getrennt") ein:
//
//  1) Audio erzeugen (z. B. Sprache):
//     - Online-TTS oder z. B. mit Windows-Bordmitteln / Balabolka / Piper.
//     - Oder eine vorhandene Melodie/Sprachaufnahme nehmen.
//
//  2) In das richtige Format bringen (WICHTIG: 44100 Hz, MONO, 16-bit PCM):
//     Mit ffmpeg:
//       ffmpeg -i quelle.mp3 -ac 1 -ar 44100 -sample_fmt s16 verbunden.wav
//
//  3) WAV in ein C-Array umwandeln. Zwei einfache Wege:
//
//     a) Mit ffmpeg direkt Roh-PCM erzeugen und mit xxd wandeln:
//          ffmpeg -i verbunden.wav -f s16le -ac 1 -ar 44100 verbunden.raw
//          xxd -i verbunden.raw > verbunden.inc
//        Das ergibt ein "unsigned char[]" – als int16_t interpretieren:
//        Besser die Python-Variante unten nutzen.
//
//     b) Mit einem kleinen Python-Skript (empfohlen, liefert int16_t):
//        ---------------------------------------------------------------
//        import wave, sys
//        w = wave.open("verbunden.wav", "rb")
//        assert w.getframerate() == 44100 and w.getnchannels() == 1 and w.getsampwidth() == 2
//        import struct
//        frames = w.readframes(w.getnframes())
//        vals = struct.unpack("<%dh" % (len(frames)//2), frames)
//        name = "snd_connected"
//        with open("snd_connected.h", "w") as f:
//            f.write('#pragma once\n#include <stdint.h>\n')
//            f.write('const int16_t %s[] PROGMEM = {\n' % name)
//            for i in range(0, len(vals), 16):
//                f.write(",".join(str(v) for v in vals[i:i+16]) + ",\n")
//            f.write('};\n')
//            f.write('const size_t %s_len = %d;\n' % (name, len(vals)))
//        ---------------------------------------------------------------
//
//  4) Die erzeugten Header (z. B. snd_connected.h / snd_disconnected.h) in
//     den Ordner include/ legen, hier einbinden und die HAS_*-Makros setzen.
//     Solange die Makros nicht definiert sind, werden die eingebauten
//     Melodien (Beeps) verwendet.
// ============================================================================

// --- Sprach-/Audio-Clips aktivieren -----------------------------------------
// Kommentiere die passende Zeile ein, sobald du die Header-Datei erzeugt hast:

// #include "snd_connected.h"      // definiert: const int16_t snd_connected[];    const size_t snd_connected_len;
// #define HAS_SND_CONNECTED

// #include "snd_disconnected.h"   // definiert: const int16_t snd_disconnected[]; const size_t snd_disconnected_len;
// #define HAS_SND_DISCONNECTED
