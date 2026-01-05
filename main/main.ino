#include <Arduino.h>
#include <math.h>
#include <LiquidCrystal.h>

//  PIN DEFINITIONS 
// LCD pins
LiquidCrystal lcd(D7, D8, D9, D10, D11, D12);

// Button pins
#define BTN_MODE D2
#define BTN_KEY  D3

// LED pins
#define LED_OUT_OF_KEY D4
#define LED_IN_KEY     D5

//  PITCH DETECTION CONFIG 
#define FS 8000
#define N 1024
#define F_MIN 70
#define F_MAX 1000

volatile uint16_t buffer[N];
volatile uint16_t idx = 0;
volatile bool bufferReady = false;

HardwareTimer *timer = nullptr;

// KEY SIGNATURE DATA 
const char* keySignatures[] = {
  "C", "G", "D", "A", "E", "B", "F#", "C#",
  "F", "Bb", "Eb", "Ab", "Db", "Gb", "Cb"
};
const int numKeys = 15;

// Root note (as semitone 0-11) for each key signature
// C=0, C#=1, D=2, D#=3, E=4, F=5, F#=6, G=7, G#=8, A=9, A#=10, B=11
const int keyRoots[] = {
  0,  // C
  7,  // G
  2,  // D
  9,  // A
  4,  // E
  11, // B
  6,  // F#
  1,  // C#
  5,  // F
  10, // Bb
  3,  // Eb
  8,  // Ab
  1,  // Db (enharmonic to C#)
  6,  // Gb (enharmonic to F#)
  11  // Cb (enharmonic to B)
};

// Major scale intervals: W W H W W W H (0,2,4,5,7,9,11)
const int majorIntervals[] = {0, 2, 4, 5, 7, 9, 11};
// Minor scale intervals: W H W W H W W (0,2,3,5,7,8,10)
const int minorIntervals[] = {0, 2, 3, 5, 7, 8, 10};

int currentKey = 0;
bool isMajor = true;

// BUTTON STATE
bool lastModeState = HIGH;
bool lastKeyState = HIGH;
unsigned long lastDebounce = 0;
const unsigned long debounceDelay = 50;

// DETECTED NOTE STATE
int lastDisplayedMidi = -1;
bool lastInKey = false;

// HELPER FUNCTIONS
static inline float log2f_safe(float x) {
  return logf(x) / logf(2.0f);
}

int freqToMidi(float f) {
  return (int)lroundf(69.0f + 12.0f * log2f_safe(f / 440.0f));
}

float midiToFreq(int midi) {
  return 440.0f * powf(2.0f, (midi - 69) / 12.0f);
}

const char* noteName(int midi) {
  static const char* names[] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
  return names[midi % 12];
}

bool isNoteInKey(int midi) {
  int noteClass = midi % 12;  // 0-11
  int root = keyRoots[currentKey];
  const int* intervals = isMajor ? majorIntervals : minorIntervals;
  
  for (int i = 0; i < 7; i++) {
    int scaleNote = (root + intervals[i]) % 12;
    if (noteClass == scaleNote) return true;
  }
  return false;
}

// SAMPLING ISR
void sampleISR() {
  if (!bufferReady) {
    buffer[idx++] = analogRead(A0);
    if (idx >= N) {
      idx = 0;
      bufferReady = true;
    }
  }
}

// PITCH DETECTION
bool detectPitch(const uint16_t *x_u16, float &f_out, float &cents_out, int &midi_out) {
  long sum = 0;
  uint16_t minv = 4095, maxv = 0;
  for (int i = 0; i < N; i++) {
    uint16_t v = x_u16[i];
    sum += v;
    if (v < minv) minv = v;
    if (v > maxv) maxv = v;
  }
  float mean = (float)sum / (float)N;
  float p2p = (float)(maxv - minv);

  if (p2p < 120.0f) return false;

  int lagMin = (int)(FS / (float)F_MAX);
  int lagMax = (int)(FS / (float)F_MIN);
  if (lagMin < 2) lagMin = 2;
  if (lagMax > N / 2) lagMax = N / 2;

  const int start = (int)(0.015f * FS);
  int usable = N - start;
  if (usable <= lagMax + 1) return false;

  auto corrAt = [&](int lag) -> float {
    float score = 0.0f;
    for (int i = start; i < N - lag; i++) {
      float a = (float)x_u16[i] - mean;
      float b = (float)x_u16[i + lag] - mean;
      score += a * b;
    }
    return score;
  };

  float bestScore = -1e30f;
  int bestLag = -1;

  for (int lag = lagMin; lag <= lagMax; lag++) {
    float score = corrAt(lag);
    if (score > bestScore) {
      bestScore = score;
      bestLag = lag;
    }
  }

  // Octave error guard
  if (bestLag / 2 >= lagMin) {
    int lag2 = bestLag / 2;
    float score2 = corrAt(lag2);
    if (score2 > 0.90f * bestScore) {
      bestLag = lag2;
      bestScore = score2;
    }
  }

  if (bestLag <= 1 || bestLag >= lagMax) return false;

  // Parabolic interpolation
  float r_m1 = corrAt(bestLag - 1);
  float r_0  = corrAt(bestLag);
  float r_p1 = corrAt(bestLag + 1);

  float denom = (r_m1 - 2.0f * r_0 + r_p1);
  float delta = 0.0f;
  if (fabsf(denom) > 1e-12f) {
    delta = 0.5f * (r_m1 - r_p1) / denom;
    if (delta > 0.5f)  delta = 0.5f;
    if (delta < -0.5f) delta = -0.5f;
  }

  float refinedLag = (float)bestLag + delta;
  float f = (float)FS / refinedLag;

  int midi = freqToMidi(f);
  float f_ref = midiToFreq(midi);
  float cents = 1200.0f * log2f_safe(f / f_ref);

  f_out = f;
  cents_out = cents;
  midi_out = midi;
  return true;
}

// DISPLAY
void updateDisplay(int midi, bool inKey) {
  lcd.clear();
  
  // Line 1: Current key
  lcd.setCursor(0, 0);
  lcd.print("Key:");
  lcd.print(keySignatures[currentKey]);
  lcd.print(" ");
  lcd.print(isMajor ? "Maj" : "Min");
  
  // Line 2: Detected note
  lcd.setCursor(0, 1);
  if (midi >= 0) {
    int octave = (midi / 12) - 1;
    lcd.print("Note:");
    lcd.print(noteName(midi));
    lcd.print(octave);
    lcd.print(" ");
    lcd.print(inKey ? "OK" : "X");
  } else {
    lcd.print("Note: ---");
  }
}

void updateKeyDisplay() {
  updateDisplay(lastDisplayedMidi, lastInKey);
}

// LED CONTROL
void updateLEDs(bool inKey, bool noteDetected) {
  if (noteDetected) {
    digitalWrite(LED_IN_KEY, inKey ? HIGH : LOW);
    digitalWrite(LED_OUT_OF_KEY, inKey ? LOW : HIGH);
  } else {
    digitalWrite(LED_IN_KEY, LOW);
    digitalWrite(LED_OUT_OF_KEY, LOW);
  }
}

// SETUP
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // Initialize buttons
  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_KEY, INPUT_PULLUP);

  // Initialize LEDs
  pinMode(LED_OUT_OF_KEY, OUTPUT);
  pinMode(LED_IN_KEY, OUTPUT);
  digitalWrite(LED_OUT_OF_KEY, LOW);
  digitalWrite(LED_IN_KEY, LOW);

  // Initialize LCD
  lcd.begin(16, 2);
  updateDisplay(-1, false);

  // Initialize timer for sampling
  timer = new HardwareTimer(TIM2);
  timer->setOverflow(FS, HERTZ_FORMAT);
  timer->attachInterrupt(sampleISR);
  timer->resume();

  Serial.println("Guitar Key Checker Started");
}

// MAIN LOOP
void loop() {
  // Handle button presses
  bool modeState = digitalRead(BTN_MODE);
  bool keyState = digitalRead(BTN_KEY);

  if (modeState == LOW && lastModeState == HIGH && (millis() - lastDebounce) > debounceDelay) {
    isMajor = !isMajor;
    updateKeyDisplay();
    lastDebounce = millis();
  }

  if (keyState == LOW && lastKeyState == HIGH && (millis() - lastDebounce) > debounceDelay) {
    currentKey = (currentKey + 1) % numKeys;
    updateKeyDisplay();
    lastDebounce = millis();
  }

  lastModeState = modeState;
  lastKeyState = keyState;

  // Process pitch detection
  if (bufferReady) {
    static uint16_t x[N];
    noInterrupts();
    for (int i = 0; i < N; i++) x[i] = buffer[i];
    bufferReady = false;
    interrupts();

    float f, cents;
    int midi;

    if (detectPitch(x, f, cents, midi)) {
      bool inKey = isNoteInKey(midi);
      
      // Update display only if note changed
      if (midi != lastDisplayedMidi || inKey != lastInKey) {
        lastDisplayedMidi = midi;
        lastInKey = inKey;
        updateDisplay(midi, inKey);
      }
      
      // Update LEDs
      updateLEDs(inKey, true);

      // Serial debug output
      int octave = (midi / 12) - 1;
      Serial.print(noteName(midi));
      Serial.print(octave);
      Serial.print(" - ");
      Serial.println(inKey ? "IN KEY" : "OUT OF KEY");
    } else {
      // No note detected - turn off LEDs
      updateLEDs(false, false);
    }
  }
}