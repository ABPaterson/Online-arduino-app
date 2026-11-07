// SHERP N 1200 1:22 AirLoom demo model - bench test sketch (v0.1)
// Board: ESP32-C3 SuperMini  (Arduino core 3.x, "ESP32C3 Dev Module",
//        USB CDC On Boot: Enabled)
//
// On power-up it steps through every light once, runs the winch out/in
// briefly, then waits for Serial Monitor commands (115200 baud):
//   1 headlights  2 tail lights  3 light bar  4 spotlights  5 fridge
//   b amber beacon (blinks)      a all on     x all off
//   o winch OUT (1 s)   i winch IN (1 s)   s winch stop
//
// This is only for checking the wiring before the belly plate goes on.
// The real demo would run the AirLoom Node firmware instead.

const int PIN_HEAD   = 0;
const int PIN_TAIL   = 1;
const int PIN_BAR    = 3;
const int PIN_SPOT   = 4;
const int PIN_FRIDGE = 5;
const int PIN_AMBER  = 6;
const int PIN_WINCH  = 7;   // signal wire of the 360-degree winch servo

const int LIGHTS[] = {PIN_HEAD, PIN_TAIL, PIN_BAR, PIN_SPOT, PIN_FRIDGE};
const char *NAMES[] = {"headlights", "tail lights", "light bar", "spotlights", "fridge"};
const int N_LIGHTS = 5;

bool beaconOn = false;
uint32_t winchStopAt = 0;

// Continuous-rotation servo: 1500 us = stop, lower/higher = the two directions.
// If IN and OUT come out backwards, swap these two numbers.
const int US_STOP = 1500, US_OUT = 1000, US_IN = 2000;

void winchUs(int us) {
  ledcWrite(PIN_WINCH, (uint32_t)us * 16384UL / 20000UL);   // 50 Hz, 14-bit
}

void winchRun(int us, uint32_t ms) {
  winchUs(us);
  winchStopAt = millis() + ms;
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < N_LIGHTS; i++) pinMode(LIGHTS[i], OUTPUT);
  pinMode(PIN_AMBER, OUTPUT);
  ledcAttach(PIN_WINCH, 50, 14);
  winchUs(US_STOP);
  delay(500);

  for (int i = 0; i < N_LIGHTS; i++) {          // walk through every light
    Serial.printf("Testing %s\n", NAMES[i]);
    digitalWrite(LIGHTS[i], HIGH); delay(700); digitalWrite(LIGHTS[i], LOW);
  }
  Serial.println("Testing amber beacon");
  for (int k = 0; k < 4; k++) { digitalWrite(PIN_AMBER, HIGH); delay(250); digitalWrite(PIN_AMBER, LOW); delay(250); }
  Serial.println("Testing winch OUT then IN");
  winchUs(US_OUT); delay(700); winchUs(US_STOP); delay(300);
  winchUs(US_IN);  delay(700); winchUs(US_STOP);
  Serial.println("Ready. Commands: 1-5 b a x o i s");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c >= '1' && c <= '5') {
      int p = LIGHTS[c - '1'];
      digitalWrite(p, !digitalRead(p));
      Serial.printf("%s %s\n", NAMES[c - '1'], digitalRead(p) ? "ON" : "OFF");
    } else if (c == 'b') { beaconOn = !beaconOn; Serial.printf("beacon %s\n", beaconOn ? "ON" : "OFF"); }
    else if (c == 'a') { for (int i = 0; i < N_LIGHTS; i++) digitalWrite(LIGHTS[i], HIGH); beaconOn = true; }
    else if (c == 'x') { for (int i = 0; i < N_LIGHTS; i++) digitalWrite(LIGHTS[i], LOW); beaconOn = false; }
    else if (c == 'o') winchRun(US_OUT, 1000);
    else if (c == 'i') winchRun(US_IN, 1000);
    else if (c == 's') { winchUs(US_STOP); winchStopAt = 0; }
  }
  // amber beacon: double-flash pattern
  uint32_t t = millis() % 1000;
  digitalWrite(PIN_AMBER, beaconOn && (t < 80 || (t > 160 && t < 240)));
  // winch is hold-to-run style: stops by itself
  if (winchStopAt && (int32_t)(millis() - winchStopAt) >= 0) { winchUs(US_STOP); winchStopAt = 0; }
}
