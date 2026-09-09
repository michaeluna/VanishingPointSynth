# VanishingPointSynth

A joystick-controlled, single-voice phase-distortion resonant synthesizer with a performance looper, built with Mozzi for an Arduino Nano (ATmega328P, old bootloader).

This is the latest saved `PD_resonant_v1` sketch used in the **Cloudburster working** task and successfully compiled and flashed there on August 26, 2026. The sketch and custom voice library are included without code changes.

## Build

1. Install the Arduino AVR Boards package and select **Arduino Nano**, processor **ATmega328P (Old Bootloader)**.
2. Install **Mozzi 2.0.4** and **FixMath 1.0.9** using Library Manager (the locally installed versions used for verification).
3. Copy `libraries/PDResonantCustom` into your Arduino sketchbook's `libraries` folder.
4. Open `PD_resonant_v1/PD_resonant_v1.ino`, select your board's port, and upload.

With Arduino CLI, from the repository root:

```sh
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old --libraries libraries PD_resonant_v1
arduino-cli upload --port COM3 --fqbn arduino:avr:nano:cpu=atmega328old PD_resonant_v1
```

Replace `COM3` with your board's port.

## Controls and audio

| Pin | Function |
| --- | --- |
| A0 | Pitch: D Kurd pitch collection, C1 to C5 |
| A1 | Phase-distortion depth: 0.01–0.025 over the first 75%, then 0.025–0.075 |
| A2 | Shared envelope time, 10–1500 ms; always live, never recorded |
| D3 | Active-low sound gate with internal pull-up |
| D4 | Active-low looper button: record → playback → normal |
| D5 | Status LED: on normally; quarter-note pulses while recording, half-note pulses during playback, at 120 BPM |
| D9 | High-order PWM audio through 3.9 kΩ |
| D10 | Low-order PWM audio through 499 kΩ |

Combine the D9 and D10 resistor outputs at the audio output node, with 4.7 nF from that node to ground. Connect the D3 and D4 buttons to ground.

The looper records pitch, depth, and gate changes. Its first playback preserves recorded timing; background quantization adjusts later playback to a sixteenth-note grid. Sudden joystick endpoint dropouts are guarded while the physical gate is held.

## Attribution

The sketch retains its Mozzi copyright and LGPL notice. The included `PDResonantCustom` library identifies itself as a project-specific fork of Mozzi's PDResonant voice. Mozzi and FixMath are external dependencies and are not bundled. The repository's existing `LICENSE` is preserved.
