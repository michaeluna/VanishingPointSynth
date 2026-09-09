/*  Single-voice phase-distortion resonant synthesizer using Mozzi.

    A0 selects D Kurd scale notes from C1 to C5.
    A1 controls phase-distortion resonance depth with a 75% breakpoint:
    0.01 to 0.025 over the lower 75%, then 0.025 to 0.075 over the upper 25%.
    While physical D3 is held, implausible A0/A1 jumps directly to ADC 0 or
    1023 are treated as joystick wiper dropouts. That control holds its last
    valid value until D3 is released or three good samples arrive; gradually
    reached endpoints still work.
    A2 controls one shared 10 to 1500 ms time for the internal phase-
    distortion attack and decay and for D3's external attack and release;
    unlike A0, A1, and D3, it is not recorded by the looper.
    An active-low button on D4 controls record, loop playback, and exit.
    Playback pass one preserves the performance. In the background, D3 note
    timing is fitted to a 60-160 BPM 4/4 tempo and a 1/16-note grid. The
    quantized timing starts at the next loop boundary; nearby A0 pitch and A1
    depth events move with their note-on event.
    D5 stays on in normal mode, pulses on quarter notes at 120 BPM while
    recording, and pulses on half notes at 120 BPM during playback.

    HiFi circuit: combine D9 through 3.9k and D10 through 499k at the
    audio output node, with 4.7nF from that node to ground.

   Mozzi documentation/API
   https://sensorium.github.io/Mozzi/doc/html/index.html

   Mozzi help/discussion/announcements:
   https://groups.google.com/forum/#!forum/mozzi-users

   Copyright 2012-2024 Tim Barrass and the Mozzi Team

   Mozzi is licensed under the GNU Lesser General Public Licence (LGPL) Version 2.1 or later.
*/

#include <MozziConfigValues.h>
#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_2PIN_PWM
#define MOZZI_CONTROL_RATE 128
#include <Mozzi.h>
#include <mozzi_midi.h>
#include <PDResonantCustom.h>

PDResonantCustom voice;

// D Kurd pitch collection across C-to-C endpoints: C, D, Eb, F, G, A, Bb.
const uint8_t D_KURD_OFFSETS_FROM_C[] PROGMEM = {0, 2, 3, 5, 7, 9, 10};

const uint8_t CARRIER_PITCH_PIN = A0;
const uint8_t PD_DEPTH_PIN = A1;
const uint8_t ENVELOPE_TIME_PIN = A2;
const uint8_t GATE_PIN = 3;
const uint8_t LOOPER_BUTTON_PIN = 4;
const uint8_t LOOPER_LED_PIN = 5;
const uint8_t LOWEST_MIDI_NOTE = 24;  // C1
const uint8_t HIGHEST_MIDI_NOTE = 72; // C5
const uint8_t SCALE_NOTES_PER_OCTAVE =
    sizeof(D_KURD_OFFSETS_FROM_C) / sizeof(D_KURD_OFFSETS_FROM_C[0]);
const uint8_t SCALE_OCTAVES =
    (HIGHEST_MIDI_NOTE - LOWEST_MIDI_NOTE) / 12;
const uint8_t SCALE_NOTE_COUNT =
    SCALE_OCTAVES * SCALE_NOTES_PER_OCTAVE + 1;
const uint16_t POT_CENTER = 512;
const uint16_t MIN_ENVELOPE_TIME_MS = 10;
const uint16_t MAX_ENVELOPE_TIME_MS = 1500;
const uint16_t PD_DEPTH_BREAKPOINT = 768;
const float MIN_PD_DEPTH = 0.01f;
const float PD_DEPTH_AT_BREAKPOINT = 0.025f;
const float MAX_PD_DEPTH = 0.075f;
const uint16_t AMPLITUDE_MAX = 32767;
const uint32_t AMPLITUDE_MAX_Q8 = (uint32_t)AMPLITUDE_MAX << 8;
const uint16_t NOTE_RAMP_SAMPLES =
    (MOZZI_AUDIO_RATE * 15UL + 999) / 1000;
const uint16_t NOTE_AMPLITUDE_STEP =
    (AMPLITUDE_MAX + NOTE_RAMP_SAMPLES - 1) / NOTE_RAMP_SAMPLES;
const uint8_t EVENT_CAPACITY = 128;
const uint8_t RECORD_INTERVAL_MS = 47;
const uint8_t LOOPER_DEBOUNCE_MS = 25;
const uint8_t PD_POT_HYSTERESIS = 4;
const uint8_t RECORD_PD_POT_THRESHOLD = 51; // 5% of 0-1023
const uint16_t RECORD_LED_BLINK_MS = 500;
const uint16_t PLAYBACK_LED_BLINK_MS = 1000;
const uint8_t LOOPER_LED_PULSE_MS = 100;
const uint8_t ANALOG_SMOOTHING_SHIFT = 2;
const uint8_t NOTE_STABILITY_SAMPLES = 4;
const uint8_t NOTE_CONTROL_ASSOCIATION_TICKS = 4;
const uint8_t ENDPOINT_APPROACH_WINDOW = 128;
const uint8_t ENDPOINT_RECOVERY_SAMPLES = 3;
const uint16_t QUANTIZER_NO_TARGET = 0xFFFF;

enum NoteTransitionState : uint8_t {
  NOTE_STEADY,
  NOTE_RAMPING_DOWN,
  NOTE_WAITING_FOR_CHANGE,
  NOTE_RAMPING_UP
};

enum LooperMode : uint8_t {
  LOOPER_NORMAL,
  LOOPER_RECORDING,
  LOOPER_PLAYBACK
};

enum RecordedControl : uint8_t {
  CONTROL_CARRIER_NOTE,
  CONTROL_PD_DEPTH,
  CONTROL_GATE
};

enum QuantizerState : uint8_t {
  QUANTIZER_IDLE,
  QUANTIZER_SCORING,
  QUANTIZER_ADJUSTING,
  QUANTIZER_READY
};

struct ControlEvent {
  uint16_t tick;
  uint16_t controlAndValue;
};

static_assert(sizeof(ControlEvent) == 4, "ControlEvent must remain 4 bytes");

struct AnalogSmoother {
  int32_t valueQ8;
  bool initialized;

  AnalogSmoother() : valueQ8(0), initialized(false) {}

  uint16_t next(uint16_t input) {
    const int32_t targetQ8 = (int32_t)input << 8;
    if (!initialized) {
      valueQ8 = targetQ8;
      initialized = true;
    } else {
      valueQ8 += (targetQ8 - valueQ8) >> ANALOG_SMOOTHING_SHIFT;
    }
    return (valueQ8 + 128) >> 8;
  }
};

struct EndpointDropoutGuard {
  uint16_t lastGoodValue;
  bool initialized;
  bool dropoutLatched;
  uint8_t recoverySamples;

  EndpointDropoutGuard()
      : lastGoodValue(0), initialized(false), dropoutLatched(false),
        recoverySamples(0) {}

  uint16_t next(uint16_t input, bool physicalGateHeld) {
    if (!initialized) {
      lastGoodValue = input;
      initialized = true;
      return input;
    }

    if (!physicalGateHeld) {
      dropoutLatched = false;
      recoverySamples = 0;
      lastGoodValue = input;
      return input;
    }

    if (dropoutLatched) {
      if (input > 0 && input < 1023) {
        if (++recoverySamples >= ENDPOINT_RECOVERY_SAMPLES) {
          dropoutLatched = false;
          recoverySamples = 0;
          lastGoodValue = input;
          return input;
        }
      } else {
        recoverySamples = 0;
      }
      return lastGoodValue;
    }

    const bool suddenLowEndpoint =
        input == 0 && lastGoodValue > ENDPOINT_APPROACH_WINDOW;
    const bool suddenHighEndpoint =
        input == 1023
        && lastGoodValue < 1023 - ENDPOINT_APPROACH_WINDOW;
    if (suddenLowEndpoint || suddenHighEndpoint) {
      dropoutLatched = true;
      recoverySamples = 0;
      return lastGoodValue;
    }

    lastGoodValue = input;
    return input;
  }
};

uint8_t currentCarrierMidiNote = LOWEST_MIDI_NOTE;
uint8_t pendingCarrierMidiNote = LOWEST_MIDI_NOTE;
uint16_t currentDepthPotValue = 0xFFFF;
uint16_t currentEnvelopePotValue = 0xFFFF;
uint16_t pdEnvelopeTimeMs =
    (MIN_ENVELOPE_TIME_MS + MAX_ENVELOPE_TIME_MS) / 2;
float pdDepth = 0.05f;
uint8_t lastLiveCarrierMidiNote = LOWEST_MIDI_NOTE;
uint8_t stableCarrierMidiNote = LOWEST_MIDI_NOTE;
uint8_t candidateCarrierMidiNote = LOWEST_MIDI_NOTE;
uint8_t candidateCarrierNoteSamples = 0;
bool lastLiveGate = false;
bool gateOpen = false;
bool pdVoiceGateOpen = false;
uint32_t amplitudeQ8 = 0;
uint32_t gateAmplitudeStepQ8 = 1;
uint16_t noteAmplitude = AMPLITUDE_MAX;
volatile NoteTransitionState noteTransitionState = NOTE_STEADY;
AnalogSmoother carrierPotSmoother;
AnalogSmoother depthPotSmoother;
AnalogSmoother envelopePotSmoother;
EndpointDropoutGuard carrierEndpointGuard;
EndpointDropoutGuard depthEndpointGuard;
ControlEvent events[EVENT_CAPACITY];
volatile LooperMode looperMode = LOOPER_NORMAL;
uint8_t eventCount = 0;
uint8_t playbackIndex = 0;
uint8_t lastRecordedCarrierMidiNote = LOWEST_MIDI_NOTE;
uint16_t lastRecordedDepthPotValue = POT_CENTER;
bool lastRecordedGate = false;
bool recordingFull = false;
uint16_t loopDurationTicks = 1;
uint32_t recordStartMs = 0;
uint32_t nextRecordSampleMs = 0;
uint32_t playbackStartMs = 0;
uint32_t playbackLoopEndMs = 0;
uint32_t nextPlaybackEventMs = 0;
bool lastLooperButtonReading = HIGH;
bool stableLooperButtonState = HIGH;
uint32_t looperDebounceStartMs = 0;
uint8_t gateDebounceHistory = 0x07;
bool debouncedLiveGate = false;
bool looperLedState = true;
uint32_t lastLooperLedToggleMs = 0;
QuantizerState quantizerState = QUANTIZER_IDLE;
bool quantizedPlaybackActive = false;
uint16_t quantizerCandidateBars = 1;
uint16_t quantizerMaximumBars = 1;
uint16_t quantizerBestBars = 1;
uint8_t quantizerScoreEventIndex = 0;
uint8_t quantizerScoreCount = 0;
uint32_t quantizerScoreSum = 0;
uint32_t quantizerBestScore = 0xFFFFFFFFUL;
int16_t quantizerAdjustIndex = -1;
uint16_t quantizerNextEffectiveTick = 0;
uint16_t quantizerNextNoteOnGrid = QUANTIZER_NO_TARGET;
uint16_t quantizerPendingGateOffGrid = QUANTIZER_NO_TARGET;
uint16_t quantizerPendingControlTarget = QUANTIZER_NO_TARGET;
uint16_t quantizerPendingGateTick = 0;
bool quantizerPendingPitch = false;
bool quantizerPendingDepth = false;

uint8_t midiNoteFromPot(uint16_t potValue) {
  const uint8_t scaleIndex =
      ((uint32_t)potValue * SCALE_NOTE_COUNT) >> 10;
  const uint8_t octave = scaleIndex / SCALE_NOTES_PER_OCTAVE;
  const uint8_t scaleDegree = scaleIndex % SCALE_NOTES_PER_OCTAVE;
  const uint8_t semitoneOffset =
      pgm_read_byte(&D_KURD_OFFSETS_FROM_C[scaleDegree]);
  return LOWEST_MIDI_NOTE + octave * 12 + semitoneOffset;
}

uint8_t stabilizeCarrierMidiNote(uint8_t midiNote) {
  if (midiNote != candidateCarrierMidiNote) {
    candidateCarrierMidiNote = midiNote;
    candidateCarrierNoteSamples = 1;
  } else if (candidateCarrierNoteSamples < NOTE_STABILITY_SAMPLES) {
    ++candidateCarrierNoteSamples;
  }

  if (candidateCarrierNoteSamples >= NOTE_STABILITY_SAMPLES) {
    stableCarrierMidiNote = candidateCarrierMidiNote;
  }
  return stableCarrierMidiNote;
}

float pdDepthFromPot(uint16_t potValue) {
  if (potValue <= PD_DEPTH_BREAKPOINT) {
    return MIN_PD_DEPTH
        + (float)potValue * (PD_DEPTH_AT_BREAKPOINT - MIN_PD_DEPTH)
          / PD_DEPTH_BREAKPOINT;
  }

  return PD_DEPTH_AT_BREAKPOINT
      + (float)(potValue - PD_DEPTH_BREAKPOINT)
        * (MAX_PD_DEPTH - PD_DEPTH_AT_BREAKPOINT)
        / (1023 - PD_DEPTH_BREAKPOINT);
}

uint16_t envelopeTimeFromPot(uint16_t potValue) {
  return MIN_ENVELOPE_TIME_MS
      + (uint32_t)(MAX_ENVELOPE_TIME_MS - MIN_ENVELOPE_TIME_MS)
        * potValue / 1023;
}

void setEnvelopeTimeFromPot(uint16_t potValue) {
  pdEnvelopeTimeMs = envelopeTimeFromPot(potValue);
  const uint16_t rampSamples =
      ((uint32_t)MOZZI_AUDIO_RATE * pdEnvelopeTimeMs + 999) / 1000;
  gateAmplitudeStepQ8 =
      (AMPLITUDE_MAX_Q8 + rampSamples - 1) / rampSamples;
}

void applyPdEnvelopeSettings() {
  voice.setPDEnv(pdEnvelopeTimeMs, pdEnvelopeTimeMs);
  voice.setPDDepth(pdDepth);
}

void triggerPdVoice() {
  voice.noteOn(1, currentCarrierMidiNote, 127);
  applyPdEnvelopeSettings();
}

void syncPdVoiceGate() {
  if (gateOpen == pdVoiceGateOpen) return;

  if (gateOpen) {
    triggerPdVoice();
  } else {
    voice.noteOff(1, currentCarrierMidiNote, 0);
  }
  pdVoiceGateOpen = gateOpen;
}

void requestCarrierNote(uint8_t midiNote) {
  if (midiNote != pendingCarrierMidiNote) {
    pendingCarrierMidiNote = midiNote;
    noteTransitionState = NOTE_RAMPING_DOWN;
  }
}

void setCarrierNoteImmediately(uint8_t midiNote) {
  currentCarrierMidiNote = midiNote;
  pendingCarrierMidiNote = midiNote;
  if (pdVoiceGateOpen) triggerPdVoice();
  noteAmplitude = AMPLITUDE_MAX;
  noteTransitionState = NOTE_STEADY;
}

void applyCarrierChangeAtSilence() {
  if (noteTransitionState == NOTE_WAITING_FOR_CHANGE) {
    currentCarrierMidiNote = pendingCarrierMidiNote;
    if (pdVoiceGateOpen) triggerPdVoice();
    noteTransitionState = NOTE_RAMPING_UP;
  }
}

uint16_t encodeControlEvent(RecordedControl control, uint16_t value) {
  return ((uint16_t)control << 10) | (value & 0x03FF);
}

RecordedControl eventControl(const ControlEvent &event) {
  return (RecordedControl)((event.controlAndValue >> 10) & 0x03);
}

int8_t eventTimingNudge(const ControlEvent &event) {
  int8_t nudge = (event.controlAndValue >> 12) & 0x0F;
  if (nudge >= 8) nudge -= 16;
  return nudge;
}

void setEventTimingNudge(ControlEvent &event, int8_t nudge) {
  if (nudge < -8) nudge = -8;
  if (nudge > 7) nudge = 7;
  event.controlAndValue = (event.controlAndValue & 0x0FFF)
      | ((uint16_t)(nudge & 0x0F) << 12);
}

uint16_t eventPlaybackTick(const ControlEvent &event) {
  int32_t tick = event.tick;
  if (quantizedPlaybackActive) tick += eventTimingNudge(event);
  if (tick < 0) return 0;
  if (tick >= loopDurationTicks) return loopDurationTicks - 1;
  return tick;
}

bool addControlEvent(uint16_t tick, RecordedControl control, uint16_t value) {
  if (eventCount >= EVENT_CAPACITY) {
    recordingFull = true;
    return false;
  }

  events[eventCount].tick = tick;
  events[eventCount].controlAndValue = encodeControlEvent(control, value);
  ++eventCount;
  return true;
}

void captureInitialControlState() {
  lastRecordedCarrierMidiNote = pendingCarrierMidiNote;
  lastRecordedDepthPotValue = currentDepthPotValue == 0xFFFF
      ? POT_CENTER
      : currentDepthPotValue;
  lastRecordedGate = gateOpen;

  addControlEvent(0, CONTROL_CARRIER_NOTE, lastRecordedCarrierMidiNote);
  addControlEvent(0, CONTROL_PD_DEPTH, lastRecordedDepthPotValue);
  addControlEvent(0, CONTROL_GATE, lastRecordedGate);
}

void startRecording() {
  eventCount = 0;
  recordingFull = false;
  recordStartMs = millis();
  nextRecordSampleMs = recordStartMs + RECORD_INTERVAL_MS;
  captureInitialControlState();
  looperMode = LOOPER_RECORDING;
  looperLedState = true;
  lastLooperLedToggleMs = recordStartMs;
  digitalWrite(LOOPER_LED_PIN, HIGH);
  quantizerState = QUANTIZER_IDLE;
  quantizedPlaybackActive = false;
}

void sampleControlChanges(uint32_t now) {
  if (recordingFull || (int32_t)(now - nextRecordSampleMs) < 0) {
    return;
  }

  nextRecordSampleMs += RECORD_INTERVAL_MS;
  if ((int32_t)(now - nextRecordSampleMs) >= 0) {
    nextRecordSampleMs = now + RECORD_INTERVAL_MS;
  }

  const uint32_t elapsedTicks = (now - recordStartMs) / RECORD_INTERVAL_MS;
  if (elapsedTicks > 0xFFFF) {
    recordingFull = true;
    return;
  }
  const uint16_t tick = elapsedTicks;

  if (pendingCarrierMidiNote != lastRecordedCarrierMidiNote) {
    lastRecordedCarrierMidiNote = pendingCarrierMidiNote;
    addControlEvent(tick, CONTROL_CARRIER_NOTE,
                    lastRecordedCarrierMidiNote);
  }
  const uint16_t recordedDepthPotDifference =
      currentDepthPotValue == 0xFFFF
      ? 0
      : abs((int)currentDepthPotValue
            - (int)lastRecordedDepthPotValue);
  if (recordedDepthPotDifference >= RECORD_PD_POT_THRESHOLD) {
    lastRecordedDepthPotValue = currentDepthPotValue;
    addControlEvent(tick, CONTROL_PD_DEPTH, lastRecordedDepthPotValue);
  }
  if (gateOpen != lastRecordedGate) {
    lastRecordedGate = gateOpen;
    addControlEvent(tick, CONTROL_GATE, lastRecordedGate);
  }
}

void applyControlEvent(const ControlEvent &event) {
  const RecordedControl control = eventControl(event);
  const uint16_t value = event.controlAndValue & 0x03FF;

  switch (control) {
    case CONTROL_CARRIER_NOTE:
      setCarrierNoteImmediately((uint8_t)value);
      break;
    case CONTROL_PD_DEPTH:
      pdDepth = pdDepthFromPot(value);
      if (pdVoiceGateOpen) applyPdEnvelopeSettings();
      break;
    case CONTROL_GATE:
      gateOpen = value != 0;
      break;
  }
}

uint16_t quantizerSubdivisions(uint16_t bars) {
  return bars * 16;
}

uint16_t nearestGridIndex(uint16_t tick, uint16_t bars) {
  const uint16_t subdivisions = quantizerSubdivisions(bars);
  uint16_t gridIndex = ((uint32_t)tick * subdivisions
      + loopDurationTicks / 2) / loopDurationTicks;
  if (gridIndex >= subdivisions) gridIndex = subdivisions - 1;
  return gridIndex;
}

uint16_t tickAtGridIndex(uint16_t gridIndex, uint16_t bars) {
  const uint16_t subdivisions = quantizerSubdivisions(bars);
  uint32_t tick = ((uint32_t)gridIndex * loopDurationTicks
      + subdivisions / 2) / subdivisions;
  if (tick >= loopDurationTicks) tick = loopDurationTicks - 1;
  return tick;
}

void beginLoopQuantization() {
  const uint32_t durationMs =
      (uint32_t)loopDurationTicks * RECORD_INTERVAL_MS;
  uint16_t minimumBars = (durationMs + 3999UL) / 4000UL;
  uint16_t maximumBars = durationMs / 1500UL;
  if (minimumBars < 1) minimumBars = 1;

  // Very short loops may not contain a whole 4/4 bar in the tempo range.
  // In that case, use the closest whole-bar interpretation to 120 BPM.
  if (maximumBars < minimumBars) {
    minimumBars = (durationMs + 1000UL) / 2000UL;
    if (minimumBars < 1) minimumBars = 1;
    maximumBars = minimumBars;
  }

  quantizerCandidateBars = minimumBars;
  quantizerMaximumBars = maximumBars;
  quantizerBestBars = minimumBars;
  quantizerScoreEventIndex = 0;
  quantizerScoreCount = 0;
  quantizerScoreSum = 0;
  quantizerBestScore = 0xFFFFFFFFUL;
  quantizerState = QUANTIZER_SCORING;
  quantizedPlaybackActive = false;
}

void finishQuantizerCandidate() {
  const uint32_t timingScore = quantizerScoreCount
      ? quantizerScoreSum / quantizerScoreCount
      : 0;
  const uint16_t bpm = ((uint32_t)quantizerCandidateBars * 240000UL
      + ((uint32_t)loopDurationTicks * RECORD_INTERVAL_MS) / 2)
      / ((uint32_t)loopDurationTicks * RECORD_INTERVAL_MS);
  const uint8_t tempoPrior =
      (abs((int)bpm - 120) + 7) / 8;
  const uint32_t score = timingScore + tempoPrior;

  if (score < quantizerBestScore) {
    quantizerBestScore = score;
    quantizerBestBars = quantizerCandidateBars;
  }

  if (quantizerCandidateBars < quantizerMaximumBars) {
    ++quantizerCandidateBars;
    quantizerScoreEventIndex = 0;
    quantizerScoreCount = 0;
    quantizerScoreSum = 0;
    return;
  }

  quantizerAdjustIndex = eventCount - 1;
  quantizerNextEffectiveTick = loopDurationTicks - 1;
  quantizerNextNoteOnGrid = QUANTIZER_NO_TARGET;
  quantizerPendingGateOffGrid = QUANTIZER_NO_TARGET;
  quantizerPendingControlTarget = QUANTIZER_NO_TARGET;
  quantizerPendingPitch = false;
  quantizerPendingDepth = false;
  quantizerState = QUANTIZER_ADJUSTING;
}

void scoreOneQuantizerEvent() {
  if (quantizerScoreEventIndex >= eventCount) {
    finishQuantizerCandidate();
    return;
  }

  const ControlEvent &event = events[quantizerScoreEventIndex++];
  if (event.tick == 0 || eventControl(event) != CONTROL_GATE) return;

  const uint16_t gridIndex =
      nearestGridIndex(event.tick, quantizerCandidateBars);
  const uint16_t targetTick =
      tickAtGridIndex(gridIndex, quantizerCandidateBars);
  const uint16_t error = abs((int)targetTick - (int)event.tick);
  const uint16_t subdivisions =
      quantizerSubdivisions(quantizerCandidateBars);
  quantizerScoreSum += ((uint32_t)error * subdivisions * 256UL)
      / loopDurationTicks;
  ++quantizerScoreCount;
}

void adjustOneQuantizerEvent() {
  if (quantizerAdjustIndex < 0) {
    quantizerState = QUANTIZER_READY;
    return;
  }

  ControlEvent &event = events[quantizerAdjustIndex];
  const RecordedControl control = eventControl(event);
  uint16_t desiredTick = event.tick;

  if (event.tick > 0 && control == CONTROL_GATE) {
    uint16_t gridIndex = nearestGridIndex(event.tick, quantizerBestBars);
    const bool gateOn = (event.controlAndValue & 0x03FF) != 0;

    if (gateOn) {
      if (quantizerPendingGateOffGrid != QUANTIZER_NO_TARGET
          && gridIndex >= quantizerPendingGateOffGrid) {
        gridIndex = quantizerPendingGateOffGrid == 0
            ? 0 : quantizerPendingGateOffGrid - 1;
      }
      if (quantizerNextNoteOnGrid != QUANTIZER_NO_TARGET
          && gridIndex >= quantizerNextNoteOnGrid) {
        gridIndex = quantizerNextNoteOnGrid == 0
            ? 0 : quantizerNextNoteOnGrid - 1;
      }
      quantizerNextNoteOnGrid = gridIndex;
      quantizerPendingGateOffGrid = QUANTIZER_NO_TARGET;
      quantizerPendingControlTarget =
          tickAtGridIndex(gridIndex, quantizerBestBars);
      quantizerPendingGateTick = event.tick;
      quantizerPendingPitch = true;
      quantizerPendingDepth = true;
    } else {
      quantizerPendingGateOffGrid = gridIndex;
    }
    desiredTick = tickAtGridIndex(gridIndex, quantizerBestBars);
  } else if (event.tick > 0
      && (control == CONTROL_CARRIER_NOTE || control == CONTROL_PD_DEPTH)) {
    bool &pending = control == CONTROL_CARRIER_NOTE
        ? quantizerPendingPitch : quantizerPendingDepth;
    if (pending) {
      if (quantizerPendingGateTick >= event.tick
          && quantizerPendingGateTick - event.tick
              <= NOTE_CONTROL_ASSOCIATION_TICKS) {
        desiredTick = quantizerPendingControlTarget;
      }
      pending = false;
    }
  }

  const uint16_t lowerBound = quantizerAdjustIndex > 0
      ? events[quantizerAdjustIndex - 1].tick : 0;
  if (desiredTick < lowerBound) desiredTick = lowerBound;
  if (desiredTick > quantizerNextEffectiveTick) {
    desiredTick = quantizerNextEffectiveTick;
  }

  int16_t nudge = (int16_t)desiredTick - event.tick;
  if (nudge < -8) nudge = -8;
  if (nudge > 7) nudge = 7;
  setEventTimingNudge(event, nudge);
  quantizerNextEffectiveTick = event.tick + nudge;
  --quantizerAdjustIndex;
}

void serviceLoopQuantizer() {
  // One bounded unit per 128 Hz control update keeps divisions and event
  // rewriting out of the audio callback and spreads the work over pass one.
  if (quantizerState == QUANTIZER_SCORING) {
    scoreOneQuantizerEvent();
  } else if (quantizerState == QUANTIZER_ADJUSTING) {
    adjustOneQuantizerEvent();
  }
}

void scheduleNextPlaybackEvent() {
  if (playbackIndex < eventCount) {
    nextPlaybackEventMs = playbackStartMs
        + (uint32_t)eventPlaybackTick(events[playbackIndex])
          * RECORD_INTERVAL_MS;
  } else {
    nextPlaybackEventMs = playbackLoopEndMs;
  }
}

void startPlayback() {
  uint32_t durationTicks =
      (millis() - recordStartMs) / RECORD_INTERVAL_MS;
  if (durationTicks < 1) durationTicks = 1;
  if (durationTicks > 0xFFFF) durationTicks = 0xFFFF;
  loopDurationTicks = durationTicks;
  playbackIndex = 0;
  playbackStartMs = millis();
  playbackLoopEndMs = playbackStartMs
      + (uint32_t)loopDurationTicks * RECORD_INTERVAL_MS;
  scheduleNextPlaybackEvent();
  looperMode = LOOPER_PLAYBACK;
  looperLedState = true;
  lastLooperLedToggleMs = playbackStartMs;
  digitalWrite(LOOPER_LED_PIN, HIGH);
  beginLoopQuantization();
}

void servicePlayback(uint32_t now) {
  const uint32_t durationMs =
      (uint32_t)loopDurationTicks * RECORD_INTERVAL_MS;

  // Playback is serviced at the control rate. Use cached absolute deadlines
  // so the AVR does not perform 32-bit division on every pass through loop().
  while ((int32_t)(now - playbackLoopEndMs) >= 0) {
    playbackStartMs = playbackLoopEndMs;
    playbackLoopEndMs += durationMs;
    if (!quantizedPlaybackActive && quantizerState == QUANTIZER_READY) {
      quantizedPlaybackActive = true;
    }
    playbackIndex = 0;
    scheduleNextPlaybackEvent();
  }

  while (playbackIndex < eventCount
         && (int32_t)(now - nextPlaybackEventMs) >= 0) {
    applyControlEvent(events[playbackIndex]);
    ++playbackIndex;
    scheduleNextPlaybackEvent();
  }
}

void stopPlayback() {
  looperMode = LOOPER_NORMAL;
  currentDepthPotValue = 0xFFFF;
  looperLedState = true;
  digitalWrite(LOOPER_LED_PIN, HIGH);
  quantizerState = QUANTIZER_IDLE;
  quantizedPlaybackActive = false;
}

void advanceLooperMode() {
  if (looperMode == LOOPER_NORMAL) {
    startRecording();
  } else if (looperMode == LOOPER_RECORDING) {
    startPlayback();
  } else {
    stopPlayback();
  }
}

void serviceLooperButton(uint32_t now) {
  const bool reading = digitalRead(LOOPER_BUTTON_PIN);
  if (reading != lastLooperButtonReading) {
    lastLooperButtonReading = reading;
    looperDebounceStartMs = now;
  }

  if ((uint32_t)(now - looperDebounceStartMs) >= LOOPER_DEBOUNCE_MS
      && reading != stableLooperButtonState) {
    stableLooperButtonState = reading;
    if (stableLooperButtonState == LOW) {
      advanceLooperMode();
    }
  }
}

void serviceLooperLed(uint32_t now) {
  if (looperMode == LOOPER_NORMAL) {
    if (!looperLedState) {
      looperLedState = true;
      digitalWrite(LOOPER_LED_PIN, HIGH);
    }
    return;
  }

  const uint16_t beatIntervalMs = looperMode == LOOPER_RECORDING
      ? RECORD_LED_BLINK_MS
      : PLAYBACK_LED_BLINK_MS;

  while ((uint32_t)(now - lastLooperLedToggleMs) >= beatIntervalMs) {
    lastLooperLedToggleMs += beatIntervalMs;
  }

  const bool pulseOn =
      (uint32_t)(now - lastLooperLedToggleMs) < LOOPER_LED_PULSE_MS;
  if (pulseOn != looperLedState) {
    looperLedState = pulseOn;
    digitalWrite(LOOPER_LED_PIN, looperLedState ? HIGH : LOW);
  }
}

void serviceLooper() {
  const uint32_t now = millis();
  serviceLooperButton(now);
  serviceLooperLed(now);
  if (looperMode == LOOPER_RECORDING) {
    sampleControlChanges(now);
  } else if (looperMode == LOOPER_PLAYBACK) {
    servicePlayback(now);
  }
}

void setup(){
  pinMode(GATE_PIN, INPUT_PULLUP);
  pinMode(LOOPER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LOOPER_LED_PIN, OUTPUT);
  digitalWrite(LOOPER_LED_PIN, HIGH);
  setEnvelopeTimeFromPot(POT_CENTER);
  startMozzi(); // :)
}


void updateControl(){
  applyCarrierChangeAtSilence();

  gateDebounceHistory =
      ((gateDebounceHistory << 1) | digitalRead(GATE_PIN)) & 0x07;
  if (gateDebounceHistory == 0) {
    debouncedLiveGate = true;
  } else if (gateDebounceHistory == 0x07) {
    debouncedLiveGate = false;
  }
  const bool liveGate = debouncedLiveGate;

  const uint16_t guardedCarrierPotValue = carrierEndpointGuard.next(
      mozziAnalogRead<10>(CARRIER_PITCH_PIN), liveGate);
  const uint16_t guardedDepthPotValue = depthEndpointGuard.next(
      mozziAnalogRead<10>(PD_DEPTH_PIN), liveGate);
  const uint16_t carrierPotValue =
      carrierPotSmoother.next(guardedCarrierPotValue);
  const uint16_t depthPotValue =
      depthPotSmoother.next(guardedDepthPotValue);
  const uint16_t envelopePotValue =
      envelopePotSmoother.next(mozziAnalogRead<10>(ENVELOPE_TIME_PIN));

  const uint8_t carrierMidiNote =
      stabilizeCarrierMidiNote(midiNoteFromPot(carrierPotValue));
  const bool carrierInputChanged =
      carrierMidiNote != lastLiveCarrierMidiNote;
  const bool gateInputChanged = liveGate != lastLiveGate;
  const uint16_t depthPotDifference =
      currentDepthPotValue == 0xFFFF
      ? 0xFFFF
      : abs((int)depthPotValue - (int)currentDepthPotValue);
  const bool depthInputChanged = depthPotDifference >= PD_POT_HYSTERESIS;
  const uint16_t envelopePotDifference =
      currentEnvelopePotValue == 0xFFFF
      ? 0xFFFF
      : abs((int)envelopePotValue - (int)currentEnvelopePotValue);
  const bool envelopeInputChanged =
      envelopePotDifference >= PD_POT_HYSTERESIS;

  lastLiveCarrierMidiNote = carrierMidiNote;
  lastLiveGate = liveGate;

  const bool acceptAllLiveControls = looperMode != LOOPER_PLAYBACK;

  if (acceptAllLiveControls || carrierInputChanged) {
    if (looperMode == LOOPER_PLAYBACK) {
      setCarrierNoteImmediately(carrierMidiNote);
    } else {
      requestCarrierNote(carrierMidiNote);
    }
  }

  bool pdSettingsChanged = false;
  if (depthInputChanged) {
    currentDepthPotValue = depthPotValue;
    pdDepth = pdDepthFromPot(depthPotValue);
    pdSettingsChanged = true;
  }

  // A2 always remains live and is never overridden by loop playback.
  if (envelopeInputChanged) {
    currentEnvelopePotValue = envelopePotValue;
    setEnvelopeTimeFromPot(envelopePotValue);
    pdSettingsChanged = true;
  }
  if (pdSettingsChanged && pdVoiceGateOpen) {
    applyPdEnvelopeSettings();
  }

  if (acceptAllLiveControls || gateInputChanged) {
    gateOpen = liveGate;
  }

  // Keep the main loop dedicated to filling Mozzi's audio buffer. Looper
  // timing at 128 Hz has at most 7.8 ms jitter, well below the 47 ms event grid.
  serviceLooper();
  serviceLoopQuantizer();
  syncPdVoiceGate();
  voice.update();
}


AudioOutput updateAudio(){
  if (gateOpen) {
    if (amplitudeQ8 < AMPLITUDE_MAX_Q8 - gateAmplitudeStepQ8) {
      amplitudeQ8 += gateAmplitudeStepQ8;
    } else {
      amplitudeQ8 = AMPLITUDE_MAX_Q8;
    }
  } else {
    if (amplitudeQ8 > gateAmplitudeStepQ8) {
      amplitudeQ8 -= gateAmplitudeStepQ8;
    } else {
      amplitudeQ8 = 0;
    }
  }

  if (noteTransitionState == NOTE_RAMPING_DOWN) {
    if (noteAmplitude > NOTE_AMPLITUDE_STEP) {
      noteAmplitude -= NOTE_AMPLITUDE_STEP;
    } else {
      noteAmplitude = 0;
      noteTransitionState = NOTE_WAITING_FOR_CHANGE;
    }
  } else if (noteTransitionState == NOTE_RAMPING_UP) {
    if (noteAmplitude < AMPLITUDE_MAX - NOTE_AMPLITUDE_STEP) {
      noteAmplitude += NOTE_AMPLITUDE_STEP;
    } else {
      noteAmplitude = AMPLITUDE_MAX;
      noteTransitionState = NOTE_STEADY;
    }
  }

  const int8_t pdOutput = voice.next();
  const uint16_t gateAmplitude = amplitudeQ8 >> 8;
  const uint16_t combinedAmplitude =
      ((uint32_t)gateAmplitude * noteAmplitude) >> 15;
  const int16_t envelopedOutput =
      ((int32_t)pdOutput * combinedAmplitude) >> 15;
  return MonoOutput::from8Bit(envelopedOutput);
}


void loop(){
  audioHook(); // required here
}
