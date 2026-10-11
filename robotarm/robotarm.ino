#include <Servo.h>

// ================= PINS (buzzer removed) =================
const int PIN_BASE = 9, PIN_ELBOW = 10, PIN_WRIST = 11;
const int PIN_FEED = 2, PIN_ESTOP = 3, PIN_RETRACT = 12;
const int PIN_TRIG = 4, PIN_ECHO = 5;
const int PIN_GREEN = 7, PIN_RED = 8;
const int PIN_POT = A0;

// =====================================================================
//   CALIBRATION SECTION: everything you tune lives in this block
//   All values are OFFSETS from 90 degrees. 0 = neutral (servo at 90).
// =====================================================================

// --- 1) Direction flip: if a joint moves the WRONG way, change +1 to -1
const int DIR_BASE  = +1;
const int DIR_ELBOW = +1;
const int DIR_WRIST = +1;

// --- 2) Base (bottom servo, swivel)
const int FACE_OFS = 0;     // base offset that points at the USER (0 = 90 deg)
// bowl angle is NOT set here: you set it with the pot at startup (see cal)

// --- 3) Elbow (middle servo, up/down)
const int SCOOP_E = +30;    // elbow while scooping in the bowl
const int CARRY_E = -10;    // elbow while carrying food
const int REACH_E = +20;    // elbow fully reached toward face

// --- 4) Wrist (top servo, spoon tilt)
const int DIP_W   = -30;    // spoon tilted down into the bowl
const int LIFT_W  = +20;    // spoon tilted up to scoop food
const int CARRY_W = +10;    // spoon level-ish while carrying
const int REACH_W = -20;    // spoon at the face

// --- 5) Face distance
const float TARGET_CM = 10.0;
const float TOL_CM    = 1.0;

// --- 6) Speed (BIGGER number = SLOWER)
const int STEP_MS     = 30;   // ms per degree, normal moves
const int APPROACH_MS = 70;   // ms per step, final approach to face
const int PAUSE_MS    = 300;  // pause between motor transitions

// --- 7) Safety limit: max offset from 90 (80 -> servo stays 10..170)
const int MAX_OFS = 80;
// =====================================================================

enum { BASE = 0, ELBOW = 1, WRIST = 2 };
Servo sv[3];
const int PINS[3] = { PIN_BASE, PIN_ELBOW, PIN_WRIST };
const int DIRS[3] = { DIR_BASE, DIR_ELBOW, DIR_WRIST };
int cur[3] = { 0, 0, 0 };        // current offsets from 90

int  bowlOfs   = 0;
int  bites     = 0;
bool aborted   = false;
bool ignoreEstop = false;

// ---------------- low-level helpers ----------------
void writeJoint(int j) {
  sv[j].write(90 + DIRS[j] * cur[j]);
}

bool pressed(int pin) {
  if (digitalRead(pin) == LOW) { delay(20); return digitalRead(pin) == LOW; }
  return false;
}

void blinkGreen(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(PIN_GREEN, LOW);  delay(120);
    digitalWrite(PIN_GREEN, HIGH); delay(120);
  }
}

void handleEstop();

void checkEstop() {
  if (!ignoreEstop && digitalRead(PIN_ESTOP) == LOW) handleEstop();
}

void slew(int j, int target, int ms = STEP_MS) {
  target = constrain(target, -MAX_OFS, MAX_OFS);
  while (cur[j] != target) {
    checkEstop();
    if (aborted) return;
    cur[j] += (target > cur[j]) ? 1 : -1;
    writeJoint(j);
    delay(ms);
  }
}

void safeWait(int ms) {
  for (int t = 0; t < ms; t += 10) { checkEstop(); if (aborted) return; delay(10); }
}

// move elbow+wrist together
void slewBoth(int e, int w, int ms = STEP_MS) {
  e = constrain(e, -MAX_OFS, MAX_OFS);
  w = constrain(w, -MAX_OFS, MAX_OFS);
  while (cur[ELBOW] != e || cur[WRIST] != w) {
    checkEstop();
    if (aborted) return;
    if (cur[ELBOW] != e) { cur[ELBOW] += (e > cur[ELBOW]) ? 1 : -1; writeJoint(ELBOW); }
    if (cur[WRIST] != w) { cur[WRIST] += (w > cur[WRIST]) ? 1 : -1; writeJoint(WRIST); }
    delay(ms);
  }
}

// ---------------- ultrasonic ----------------
float readCm() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(3);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long us = pulseIn(PIN_ECHO, HIGH, 25000);
  if (us == 0) return -1;
  return us * 0.0343 / 2.0;
}

float readCmFiltered() {           // median of 5
  float v[5];
  for (int i = 0; i < 5; i++) { v[i] = readCm(); delay(8); }
  for (int i = 0; i < 4; i++)
    for (int k = i + 1; k < 5; k++)
      if (v[k] < v[i]) { float t = v[i]; v[i] = v[k]; v[k] = t; }
  return v[2];
}

// ---------------- go to neutral (all servos at 90) ----------------
void goNeutral() {
  slewBoth(0, 0);
  slew(BASE, FACE_OFS);
}

// ---------------- E-STOP ----------------
void handleEstop() {
  digitalWrite(PIN_RED, HIGH);
  digitalWrite(PIN_GREEN, LOW);
  Serial.println("EVENT,ESTOP");
  // servos hold position. Wait for release, then hold FEED 3 s (or serial "reset")
  while (digitalRead(PIN_ESTOP) == LOW) delay(10);
  unsigned long held = 0;
  while (true) {
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n'); s.trim();
      if (s == "reset") break;
    }
    if (digitalRead(PIN_FEED) == LOW) {
      if (held == 0) held = millis();
      if (millis() - held > 3000) break;
    } else held = 0;
    delay(10);
  }
  digitalWrite(PIN_RED, LOW);
  ignoreEstop = true;
  goNeutral();                     // slow, safe return to 90/90/90
  ignoreEstop = false;
  aborted = true;                  // tell running routines to bail out
  Serial.println("EVENT,ESTOP_CLEARED");
}

// ---------------- calibration (pot used ONLY here) ----------------
void calibrate() {
  Serial.println("CAL: turn pot to aim at bowl, press FEED to lock");
  slewBoth(0, 0);
  slew(BASE, FACE_OFS);
  digitalWrite(PIN_GREEN, HIGH);
  while (true) {
    int raw = analogRead(PIN_POT);                       // 0..1023
    int target = map(raw, 0, 1023, -MAX_OFS, MAX_OFS);   // pot center = 90
    target = constrain(target, -MAX_OFS, MAX_OFS);
    if (target != cur[BASE]) {
      cur[BASE] += (target > cur[BASE]) ? 1 : -1;
      writeJoint(BASE);
    }
    delay(15);
    if (pressed(PIN_FEED)) {
      bowlOfs = cur[BASE];
      while (digitalRead(PIN_FEED) == LOW) delay(10);
      break;
    }
  }
  blinkGreen(2);
  Serial.print("CAL_DONE,bowlOfs="); Serial.println(bowlOfs);
  slew(BASE, FACE_OFS);            // return to face user
}

// ---------------- approach face ----------------
bool approachFace() {
  const int N = 100;
  for (int i = 0; i <= N; i++) {
    checkEstop(); if (aborted) return false;
    int e = CARRY_E + (REACH_E - CARRY_E) * i / N;
    int w = CARRY_W + (REACH_W - CARRY_W) * i / N;
    cur[ELBOW] = constrain(e, -MAX_OFS, MAX_OFS); writeJoint(ELBOW);
    cur[WRIST] = constrain(w, -MAX_OFS, MAX_OFS); writeJoint(WRIST);
    delay(APPROACH_MS);

    float d = readCmFiltered();
    if (d > 0 && d <= TARGET_CM + TOL_CM) {
      // too close? back off along the same path
      while (d > 0 && d < TARGET_CM - TOL_CM && i > 0) {
        i--;
        cur[ELBOW] = CARRY_E + (REACH_E - CARRY_E) * i / N; writeJoint(ELBOW);
        cur[WRIST] = CARRY_W + (REACH_W - CARRY_W) * i / N; writeJoint(WRIST);
        delay(APPROACH_MS);
        d = readCmFiltered();
      }
      Serial.print("EVENT,FACE_FOUND,cm="); Serial.println(d);
      return true;
    }
  }
  return false;
}

// ---------------- feed cycle ----------------
void retractToBowl() {
  slewBoth(CARRY_E, CARRY_W);
  if (aborted) return;
  safeWait(PAUSE_MS);
  slew(BASE, bowlOfs);
  bites++;
  Serial.print("EVENT,BITE,count="); Serial.println(bites);
}

void feedCycle() {
  aborted = false;
  digitalWrite(PIN_GREEN, HIGH);
  Serial.println("EVENT,FEED_START");

  // 1) go to bowl
  slew(BASE, bowlOfs);                         if (aborted) return;
  safeWait(PAUSE_MS);

  // 2) scoop
  slewBoth(SCOOP_E, DIP_W);                    if (aborted) return;
  safeWait(PAUSE_MS);
  slew(WRIST, LIFT_W);                         if (aborted) return;
  safeWait(PAUSE_MS);
  slewBoth(CARRY_E, CARRY_W);                  if (aborted) return;
  safeWait(PAUSE_MS);

  // 3) swing base fully to user BEFORE elbow/wrist move
  slew(BASE, FACE_OFS);                        if (aborted) return;
  safeWait(PAUSE_MS);

  // 4) glide to face
  bool found = approachFace();                 if (aborted) return;
  if (!found) {
    Serial.println("EVENT,NO_FACE");
    blinkGreen(3);
    retractToBowl();
    return;
  }

  // 5) hold at face until RETRACT pressed
  blinkGreen(1);
  while (!pressed(PIN_RETRACT)) {
    checkEstop(); if (aborted) return;
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n'); s.trim();
      if (s == "retract") break;
    }
    delay(10);
  }
  while (digitalRead(PIN_RETRACT) == LOW) delay(10);
  retractToBowl();
}

// ---------------- serial commands ----------------
void handleSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n'); s.trim();
  if (s == "feed")         feedCycle();
  else if (s == "retract") retractToBowl();
  else if (s == "home")    { ignoreEstop = true; goNeutral(); ignoreEstop = false; }
  else if (s == "cal")     calibrate();
  else if (s == "p") {     // print current offsets
    Serial.print("OFFSETS base="); Serial.print(cur[BASE]);
    Serial.print(" elbow=");       Serial.print(cur[ELBOW]);
    Serial.print(" wrist=");       Serial.println(cur[WRIST]);
  }
  else if (s.length() > 1 && (s[0]=='b' || s[0]=='e' || s[0]=='w')) {
    int v = s.substring(1).toInt();            // OFFSET from 90, e.g. e-20, w15
    int j = (s[0]=='b') ? BASE : (s[0]=='e') ? ELBOW : WRIST;
    slew(j, v);
    Serial.print("OK offset="); Serial.println(cur[j]);
  }
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  analogReadResolution(10);

  pinMode(PIN_FEED, INPUT_PULLUP);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_RETRACT, INPUT_PULLUP);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_RED, OUTPUT);

  // Attach and snap ALL servos to 90 immediately
  for (int j = 0; j < 3; j++) {
    sv[j].attach(PINS[j], 500, 2400);
    cur[j] = 0;
    writeJoint(j);
  }
  delay(1000);

  calibrate();                     // aim at bowl once, lock with FEED
  Serial.println("READY");
}

void loop() {
  handleSerial();
  checkEstop();

  if (pressed(PIN_FEED)) {
    while (digitalRead(PIN_FEED) == LOW) delay(10);
    feedCycle();
  }

  // hold RETRACT 3 s while idle -> recalibrate
  if (digitalRead(PIN_RETRACT) == LOW) {
    unsigned long t = millis();
    while (digitalRead(PIN_RETRACT) == LOW) {
      if (millis() - t > 3000) { calibrate(); break; }
    }
  }
  aborted = false;
}
