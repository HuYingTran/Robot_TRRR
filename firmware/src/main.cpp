#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <stdlib.h>

// =====================================================================
//  Robot TRR - firmware Arduino Nano cho Digital Twin (digital_twin/app.py)
//  3 stepper (A4988, shield CNC V4 cho Nano) + servo MG995 (J3) + SG90 (kep)
//
//  Pin map (cung kieu project tham khao, da chay duoc):
//      J1 (vai)    : STEP=D5, DIR=D2   (stepper 4238, dai 20->60)
//      J2 (khuyu)  : STEP=D6, DIR=D3   (stepper 4239, dai 20->36)
//      Z  (vit me) : STEP=D7, DIR=D4   (stepper 4237, vit me 8 mm/vong)
//      EN chung    : D8 (LOW = bat driver)
//      MG995 (J3)  : D10      SG90 (kep): D11
//
//  Giao thuc (115200 baud, moi lenh 1 dong '\n', don vi mm / do, so voi home):
//    J z j1 j2 j3 g   setpoint khop (twin gui ~20 Hz). Khong tra loi (tru ERR).
//    H                ve home (0 0 0 0 g_home)
//    S                dung em (giam toc toi dung)
//    ?                bao vi tri ngay
//    O                lay vi tri hien tai lam home (dat lai goc toa do)
//    E 0|1            tat / bat driver stepper
//    P                ping -> OK
//  Firmware gui:  P z j1 j2 j3 g   ~20 Hz (z, j1, j2 dem tu so buoc da phat;
//                 j3, g = goc servo dang ra lenh - servo khong co phan hoi).
//
//  CHU Y: chua co cong tac hanh trinh. Vi tri luc cap nguon / reset = home.
//  Dat robot ve tu the home (canh tay duoi thang theo -Y, Z o vi tri home) truoc khi cap nguon,
//  hoac dua tay ve home roi gui lenh "O".
// =====================================================================

// ------------------------------ phan cung -----------------------------
#define EN_PIN 8
#define J3_SERVO_PIN 10
#define GRIP_SERVO_PIN 11

constexpr float STEPS_PER_REV = 200.0f * 16.0f;     // 1.8 do x vi buoc 1/16

// Truc stepper: thu tu trong mang = Z, J1, J2 (giong vector khop cua twin).
struct AxisCfg {
  const char *name;
  uint8_t stepBit;      // bit trong PORTD (D5 = PD5 ...)
  uint8_t dirPin;
  bool invert;          // dao chieu neu motor quay nguoc
  float stepsPerUnit;   // buoc / mm hoac buoc / do
  float minPos, maxPos; // gioi han mem (mm / do)
  float vmax;           // don vi / s
  float accel;          // don vi / s^2
};

AxisCfg CFG[3] = {
  // name   STEP DIR  inv    buoc/don vi                      min   max   vmax  accel
  { "Z",   7,   4,   false, STEPS_PER_REV / 8.0f,             -12,  150,  25,   100 },
  { "J1",  5,   2,   false, STEPS_PER_REV * 3.0f / 360.0f,    -135, 135,  90,   360 },
  { "J2",  6,   3,   false, STEPS_PER_REV * 1.8f / 360.0f,    -150, 150,  120,  480 },
};

// Servo: goc servo = offset + sign * goc khop.
constexpr float J3_SERVO_CENTER = 90.0f;   // j3 = 0  -> servo 90
constexpr float J3_SIGN = 1.0f;
constexpr float J3_MIN = -90.0f, J3_MAX = 90.0f;
constexpr float GRIP_SERVO_OPEN = 120.0f;  // g = 0 (ngon thang) -> servo 120 (nhu project tham khao)
constexpr float GRIP_SIGN = -1.0f;         // g tang -> servo giam (cuon cuoc)
constexpr float G_MIN = 0.0f, G_MAX = 135.0f, G_HOME = 0.0f;

// ------------------------------ phat xung (Timer2) --------------------
// Ngat 25 kHz: moi truc co bo tich luy pha 16 bit; tran -> 1 buoc. Toc do toi da 12 500 buoc/s.
constexpr uint32_t ISR_HZ = 25000;
constexpr float MAX_STEP_RATE = 12000.0f;
constexpr uint8_t STEP_MASK = _BV(5) | _BV(6) | _BV(7);

volatile uint16_t phaseInc[3] = {0, 0, 0};
volatile uint16_t phase[3] = {0, 0, 0};
volatile int8_t dirSign[3] = {1, 1, 1};
volatile int32_t posSteps[3] = {0, 0, 0};
uint8_t stepBits[3];                 // mat na bit STEP tren PORTD (tinh tu CFG trong setup)

ISR(TIMER2_COMPA_vect) {
  PORTD &= ~STEP_MASK;                 // ket thuc xung tu lan ngat truoc (rong 40 us)
  uint8_t set = 0;
  for (uint8_t i = 0; i < 3; i++) {
    const uint16_t inc = phaseInc[i];
    if (!inc) continue;
    const uint16_t p = phase[i] + inc;
    if (p < phase[i]) {                // tran -> phat 1 buoc
      set |= stepBits[i];
      posSteps[i] += dirSign[i];
    }
    phase[i] = p;
  }
  PORTD |= set;
}

static void timerBegin() {
  cli();
  TCCR2A = _BV(WGM21);                 // CTC
  TCCR2B = _BV(CS21);                  // chia 8 -> 2 MHz
  OCR2A = (F_CPU / 8 / ISR_HZ) - 1;    // 79 -> 25 kHz
  TCNT2 = 0;
  TIMSK2 = _BV(OCIE2A);
  sei();
}

// ------------------------------ dieu khien vi tri ----------------------
float targetSteps[3] = {0, 0, 0};
float velSteps[3] = {0, 0, 0};         // buoc/s, co dau
float j3Cmd = 0, gCmd = G_HOME;
bool driversOn = true;

Servo servoJ3, servoGrip;

static int32_t readPos(uint8_t i) {
  cli();
  const int32_t p = posSteps[i];
  sei();
  return p;
}

static void writeRate(uint8_t i, float rate) {
  if (rate > MAX_STEP_RATE) rate = MAX_STEP_RATE;
  const uint16_t inc = (uint16_t)(rate * 65536.0f / ISR_HZ);
  cli();
  phaseInc[i] = inc;
  sei();
}

static void setDir(uint8_t i, int8_t s) {
  if (dirSign[i] == s) return;
  const bool level = (s > 0) != CFG[i].invert;
  cli();
  digitalWrite(CFG[i].dirPin, level ? HIGH : LOW);
  dirSign[i] = s;
  sei();
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void setTarget(uint8_t i, float units) {
  targetSteps[i] = clampf(units, CFG[i].minPos, CFG[i].maxPos) * CFG[i].stepsPerUnit;
}

// Bam vi tri voi gioi han van toc + gia toc (cang gan dich cang cham: v = sqrt(2 a d)).
static void updateAxes(float dt) {
  for (uint8_t i = 0; i < 3; i++) {
    const float a = CFG[i].accel * CFG[i].stepsPerUnit;
    const float vmax = CFG[i].vmax * CFG[i].stepsPerUnit;
    const float err = targetSteps[i] - (float)readPos(i);
    float vdes = 0;
    if (fabsf(err) >= 1.0f) {
      vdes = sqrtf(2.0f * a * fabsf(err));
      if (vdes > vmax) vdes = vmax;
      if (err < 0) vdes = -vdes;
    }
    const float dv = a * dt;
    float v = velSteps[i];
    if (vdes > v + dv) v += dv;
    else if (vdes < v - dv) v -= dv;
    else v = vdes;
    if (!driversOn) v = 0;
    velSteps[i] = v;
    if (v > 0.5f) setDir(i, 1);
    else if (v < -0.5f) setDir(i, -1);
    writeRate(i, fabsf(v));
  }
}

static void writeServos() {
  const float s3 = clampf(J3_SERVO_CENTER + J3_SIGN * j3Cmd, 0, 180);
  const float sg = clampf(GRIP_SERVO_OPEN + GRIP_SIGN * gCmd, 0, 180);
  // writeMicroseconds de min hon write(do): 544..2400 us ~ 0..180 do
  servoJ3.writeMicroseconds(544 + (int)(s3 * (2400 - 544) / 180.0f));
  servoGrip.writeMicroseconds(544 + (int)(sg * (2400 - 544) / 180.0f));
}

static void reportPos() {
  Serial.print('P');
  for (uint8_t i = 0; i < 3; i++) {
    Serial.print(' ');
    Serial.print((float)readPos(i) / CFG[i].stepsPerUnit, 3);
  }
  Serial.print(' ');
  Serial.print(j3Cmd, 2);
  Serial.print(' ');
  Serial.println(gCmd, 2);
}

static void stopAll() {
  // dich = vi tri hien tai + quang duong phanh -> giam toc deu
  for (uint8_t i = 0; i < 3; i++) {
    const float a = CFG[i].accel * CFG[i].stepsPerUnit;
    const float v = velSteps[i];
    const float brake = v * fabsf(v) / (2.0f * a);
    targetSteps[i] = (float)readPos(i) + brake;
  }
}

// ------------------------------ lenh serial ----------------------------
static char lineBuf[64];
static uint8_t lineLen = 0;

static bool parseFloats(char *s, float *out, uint8_t n) {
  for (uint8_t k = 0; k < n; k++) {
    char *end;
    out[k] = (float)strtod(s, &end);
    if (end == s) return false;
    s = end;
  }
  return true;
}

// Tra ve: 0 = im lang, 1 = OK, -1 = ERR
static int8_t handleLine(char *line) {
  while (*line == ' ') line++;
  const char cmd = *line++;
  switch (cmd) {
    case 'J': {
      float v[5];
      if (!parseFloats(line, v, 5)) return -1;
      setTarget(0, v[0]);
      setTarget(1, v[1]);
      setTarget(2, v[2]);
      j3Cmd = clampf(v[3], J3_MIN, J3_MAX);
      gCmd = clampf(v[4], G_MIN, G_MAX);
      writeServos();
      return 0;
    }
    case 'H':
      for (uint8_t i = 0; i < 3; i++) setTarget(i, 0);
      j3Cmd = 0;
      gCmd = G_HOME;
      writeServos();
      return 1;
    case 'S':
      stopAll();
      return 1;
    case '?':
      reportPos();
      return 0;
    case 'O':
      cli();
      for (uint8_t i = 0; i < 3; i++) posSteps[i] = 0;
      sei();
      for (uint8_t i = 0; i < 3; i++) { targetSteps[i] = 0; velSteps[i] = 0; writeRate(i, 0); }
      return 1;
    case 'E': {
      while (*line == ' ') line++;
      driversOn = (*line != '0');
      if (!driversOn) stopAll();
      digitalWrite(EN_PIN, driversOn ? LOW : HIGH);
      return 1;
    }
    case 'P':
      return 1;
    default:
      return -1;
  }
}

static void pollSerial() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        const int8_t r = handleLine(lineBuf);
        if (r > 0) Serial.println("OK");
        else if (r < 0) { Serial.print("ERR "); Serial.println(lineBuf); }
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;                     // dong qua dai -> bo
    }
  }
}

// ------------------------------ setup / loop ---------------------------
constexpr uint32_t CTRL_PERIOD_US = 2000;   // 500 Hz
constexpr uint32_t REPORT_PERIOD_MS = 50;   // 20 Hz
static uint32_t lastCtrlUs = 0, lastReportMs = 0;

void setup() {
  Serial.begin(115200);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);          // tat driver trong luc khoi dong
  DDRD |= STEP_MASK;
  PORTD &= ~STEP_MASK;
  for (uint8_t i = 0; i < 3; i++) {
    stepBits[i] = _BV(CFG[i].stepBit);
    pinMode(CFG[i].dirPin, OUTPUT);
    digitalWrite(CFG[i].dirPin, CFG[i].invert ? LOW : HIGH);   // dirSign = +1
  }
  servoJ3.attach(J3_SERVO_PIN);
  servoGrip.attach(GRIP_SERVO_PIN);
  writeServos();
  timerBegin();
  digitalWrite(EN_PIN, LOW);
  lastCtrlUs = micros();
  Serial.println("READY Robot TRR Nano - digital twin (J z j1 j2 j3 g)");
}

void loop() {
  pollSerial();
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastCtrlUs) >= CTRL_PERIOD_US) {
    lastCtrlUs += CTRL_PERIOD_US;
    updateAxes(CTRL_PERIOD_US * 1e-6f);
  }
  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastReportMs) >= REPORT_PERIOD_MS) {
    lastReportMs = nowMs;
    reportPos();
  }
}
