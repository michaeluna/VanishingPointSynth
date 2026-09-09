# VanishingPointSynth

VanishingPointSynth is a playable, single-voice phase-distortion synthesizer for an Arduino Nano. A two-axis joystick selects pitch and resonance depth, a third analog control shapes the envelope, and a gate button articulates the sound. The instrument also includes a one-button performance looper that records pitch, resonance, and gate gestures, then gently quantizes later passes to a sixteenth-note grid.

The synth uses the [Mozzi](https://sensorium.github.io/Mozzi/) audio library and two-pin PWM output. Pitch is constrained to a D Kurd collection across C1–C5. The joystick inputs are smoothed and protected against sudden end-of-travel wiper dropouts while a note is held.

## Controls and pinout

| Arduino pin | Connect to | Function |
| --- | --- | --- |
| `A0` | Joystick X-axis output | Selects pitch from the D Kurd note collection, C1–C5 |
| `A1` | Joystick Y-axis output | Phase-distortion resonance depth; fine control through the first 75% of travel and a wider range in the final 25% |
| `A2` | 10 kΩ potentiometer wiper | Shared attack/decay and gate attack/release time, 10–1500 ms; always live and not recorded |
| `D3` | Momentary gate button to GND | Sound gate; active low using the Nano's internal pull-up |
| `D4` | Momentary looper button to GND | Cycles through record → playback → normal; active low using the internal pull-up |
| `D5` | LED anode through 220–1 kΩ resistor | Status LED; cathode connects to GND |
| `D9` | 3.9 kΩ resistor to audio node | High-order half of Mozzi's two-pin PWM output |
| `D10` | 499 kΩ resistor to audio node | Low-order half of Mozzi's two-pin PWM output |
| `5V` | Joystick VCC and A2 pot outer terminal | Control supply |
| `GND` | Joystick GND, other A2 pot outer terminal, buttons, LED, filter, and audio ground | Common ground |

## Wiring

Wire the joystick's two axis outputs to `A0` and `A1`, with its supply pins connected to `5V` and `GND`. Wire a 10 kΩ potentiometer across `5V` and `GND`, with its center wiper connected to `A2`. Reversing the two outer pot terminals reverses the direction of the envelope-time control.

The gate and looper controls are normally-open momentary buttons. Connect one side of each button to its digital pin (`D3` or `D4`) and the other side to `GND`; no external pull-up resistors are required. Connect the status LED from `D5` through a 220 Ω–1 kΩ current-limiting resistor to the LED anode, and connect its cathode to `GND`.

Build the Mozzi two-pin PWM output filter as follows:

```text
D9  ---- 3.9 kΩ ----+
                     +---- AUDIO OUT ---- amplifier/input
D10 --- 499 kΩ ------+
                     |
                    4.7 nF
                     |
                    GND
```

Keep all grounds common. Feed `AUDIO OUT` into a powered speaker, amplifier, mixer, or other line-level input; it is not intended to drive a passive speaker directly. Start with the receiving device's volume low.

## Looper behavior

`D4` advances through three modes:

1. **Record:** captures pitch, resonance depth, and gate changes. The LED pulses on quarter notes at 120 BPM.
2. **Playback:** repeats the performance. The first pass preserves the original timing while quantization is calculated; later passes move gate events to a sixteenth-note grid. The LED pulses on half notes at 120 BPM.
3. **Normal:** exits playback and returns all controls to live operation. The LED stays on.

The `A2` envelope-time control remains live in every mode and is never recorded.

## Build and upload

### Arduino IDE

1. Install the **Arduino AVR Boards** platform in Boards Manager.
2. Install **Mozzi 2.0.4** and **FixMath 1.0.9** in Library Manager.
3. Copy `libraries/PDResonantCustom` from this repository into your Arduino sketchbook's `libraries` folder, then restart the Arduino IDE.
4. Open `PD_resonant_v1/PD_resonant_v1.ino`.
5. Select **Arduino Nano** as the board and **ATmega328P (Old Bootloader)** as the processor.
6. Select the Nano's serial port and click **Upload**.

### Arduino CLI

From the repository root, with Arduino AVR Boards, Mozzi, and FixMath already installed:

```sh
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328old --libraries libraries PD_resonant_v1
arduino-cli upload --port COM3 --fqbn arduino:avr:nano:cpu=atmega328old PD_resonant_v1
```

Replace `COM3` with the port used by your Nano. The checked-in version has been compiled and uploaded successfully to an Arduino Nano using the old bootloader setting.

## Repository contents

- `PD_resonant_v1/PD_resonant_v1.ino` — main synthesizer and looper sketch
- `libraries/PDResonantCustom` — project-specific phase-distortion voice built on Mozzi

The sketch retains its Mozzi copyright and LGPL notice. Mozzi and FixMath remain external dependencies and are not bundled in this repository.
