/*
  ESP32-C3 Super Mini - Quadcopter na tugma sa ESP-Drone app
  (CRTP over UDP, tulad ng opisyal na ESP-Drone / Crazyflie protocol)

  Drone : 192.168.43.42  UDP port 2390
  Packet = CRTP header + payload + checksum (sum ng lahat mod 256)
  Commander (port 3): float roll, float pitch, float yaw, uint16 thrust

  WIRING
  MPU6050: VCC->3V3, GND->GND, SDA->GPIO6, SCL->GPIO7
  Motors (logic-level N-MOSFET + flyback diode + 100k pulldown sa gate):
    M1 Front-Right (CCW) -> GPIO0
    M2 Rear-Right  (CW)  -> GPIO1
    M3 Rear-Left   (CCW) -> GPIO3
    M4 Front-Left  (CW)  -> GPIO4

  BABALA: TEST MUNA WALANG PROPELLER!
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

// ---------- Network ----------
const char* AP_SSID = "ESP-DRONE_C3";
const char* AP_PASS = "12345678";
IPAddress DRONE_IP(192, 168, 43, 42);
IPAddress NETMASK(255, 255, 255, 0);
const uint16_t UDP_PORT = 2390;
WiFiUDP udp;

// ---------- Pins ----------
#define SDA_PIN 6
#define SCL_PIN 7
const int MOTOR_PIN[4] = {0, 1, 3, 4};   // FR, RR, RL, FL
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;

// ---------- Tuning / Signs ----------
float ANGLE_P   = 4.0;
float RATE_P    = 0.0020;
float RATE_I    = 0.0015;
float RATE_D    = 0.00005;
float YAW_P     = 0.0030;
float MAX_ANGLE = 30.0;
float IDLE      = 0.06;
float I_LIMIT   = 0.15;
int ROLL_SIGN = 1, PITCH_SIGN = 1, YAW_SIGN = 1;
int ROLL_CMD  = 1, PITCH_CMD = -1, YAW_CMD = -1;

// ---------- State ----------
float roll = 0, pitch = 0;
float gxo = 0, gyo = 0, gzo = 0;
float iRoll = 0, iPitch = 0, prevGx = 0, prevGy = 0;

float cRoll = 0, cPitch = 0, cYaw = 0, cThr = 0;
bool armed = false;
uint32_t lastCmd = 0;
uint32_t lastLoop = 0;
uint32_t lastDbg = 0;

// ---------- IMU ----------
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool readIMU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(0x68, 14) != 14) return false;
  int16_t v[7];
  for (int i = 0; i < 7; i++) {
    uint8_t h = Wire.read();
    uint8_t l = Wire.read();
    v[i] = (h << 8) | l;
  }
  ax = v[0] / 8192.0f;
  ay = v[1] / 8192.0f;
  az = v[2] / 8192.0f;
  gx = v[4] / 65.5f;
  gy = v[5] / 65.5f;
  gz = v[6] / 65.5f;
  return true;
}

void initIMU() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  mpuWrite(0x6B, 0x00);
  mpuWrite(0x1A, 0x03);
  mpuWrite(0x1B, 0x08);  // gyro +-500 dps
  mpuWrite(0x1C, 0x08);  // accel +-4g
  delay(100);
}

void calibrateGyro() {
  float ax, ay, az, gx, gy, gz;
  double sx = 0, sy = 0, sz = 0;
  int n = 0;
  for (int i = 0; i < 500; i++) {
    if (readIMU(ax, ay, az, gx, gy, gz)) { sx += gx; sy += gy; sz += gz; n++; }
    delay(3);
  }
  if (n) { gxo = sx / n; gyo = sy / n; gzo = sz / n; }
}

// ---------- Motors ----------
void setMotors(float m0, float m1, float m2, float m3) {
  float m[4] = {m0, m1, m2, m3};
  for (int i = 0; i < 4; i++) {
    m[i] = constrain(m[i], 0.0f, 1.0f);
    ledcWrite(MOTOR_PIN[i], (int)(m[i] * 255));
  }
}
void motorsOff() { setMotors(0, 0, 0, 0); }

// ---------- ESP-Drone / CRTP UDP ----------
void handleUDP() {
  int n = udp.parsePacket();
  while (n > 0) {
    uint8_t buf[64];
    int len = udp.read(buf, sizeof(buf));

    if (len >= 2) {
      uint8_t sum = 0;
      for (int i = 0; i < len - 1; i++) sum += buf[i];

      if (sum == buf[len - 1]) {
        uint8_t port = buf[0] >> 4;

        if (port == 3 && len >= 16) {
          float r, p, y;
          uint16_t th;
          memcpy(&r,  &buf[1], 4);
          memcpy(&p,  &buf[5], 4);
          memcpy(&y,  &buf[9], 4);
          memcpy(&th, &buf[13], 2);

          cRoll  = constrain(r, -MAX_ANGLE, MAX_ANGLE);
          cPitch = constrain(p, -MAX_ANGLE, MAX_ANGLE);
          cYaw   = constrain(y, -200.0f, 200.0f);
          cThr   = th / 65535.0f;
          lastCmd = millis();

          if (th == 0) armed = true;
        } else if (millis() - lastDbg > 2000) {
          lastDbg = millis();
          Serial.printf("Other CRTP pkt: header=0x%02X len=%d\n", buf[0], len);
        }
      }
    }
    n = udp.parsePacket();
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    ledcAttach(MOTOR_PIN[i], PWM_FREQ, PWM_RES);
    ledcWrite(MOTOR_PIN[i], 0);
  }

  initIMU();
  Serial.println("Calibrating gyro - wag galawin!");
  calibrateGyro();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(DRONE_IP, DRONE_IP, NETMASK);
  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 2);
  WiFi.setTxPower(WIFI_POWER_11dBm);
  udp.begin(UDP_PORT);

  Serial.printf("Ready. Connect to %s, UDP %s:%u\n", AP_SSID, DRONE_IP.toString().c_str(), UDP_PORT);
  lastLoop = micros();
}

// ---------- Main loop (250 Hz) ----------
void loop() {
  handleUDP();

  uint32_t now = micros();
  if (now - lastLoop < 4000) return;
  float dt = (now - lastLoop) * 1e-6f;
  lastLoop = now;

  float ax, ay, az, gx, gy, gz;
  if (!readIMU(ax, ay, az, gx, gy, gz)) { armed = false; motorsOff(); return; }
  gx -= gxo; gy -= gyo; gz -= gzo;

  float accRoll  = atan2f(ay, az) * 57.2958f;
  float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2958f;
  roll  = 0.98f * (roll  + gx * dt) + 0.02f * accRoll;
  pitch = 0.98f * (pitch + gy * dt) + 0.02f * accPitch;

  // Failsafe: walang commander packet sa loob ng 500 ms
  if (millis() - lastCmd > 500) { armed = false; cThr = 0; }

  // Auto-disarm kapag tumaob
  if (fabsf(roll) > 70 || fabsf(pitch) > 70) armed = false;

  if (!armed || cThr < 0.02f) {
    iRoll = iPitch = 0;
    motorsOff();
    return;
  }

  float rollSet  = ROLL_SIGN  * ROLL_CMD  * cRoll;
  float pitchSet = PITCH_SIGN * PITCH_CMD * cPitch;
  float rateYawT = YAW_CMD * cYaw;

  float rateRollT  = ANGLE_P * (rollSet  - roll);
  float ratePitchT = ANGLE_P * (pitchSet - pitch);

  float eR = rateRollT - gx;
  iRoll = constrain(iRoll + eR * RATE_I * dt, -I_LIMIT, I_LIMIT);
  float outRoll = RATE_P * eR + iRoll - RATE_D * (gx - prevGx) / dt;
  prevGx = gx;

  float eP = ratePitchT - gy;
  iPitch = constrain(iPitch + eP * RATE_I * dt, -I_LIMIT, I_LIMIT);
  float outPitch = RATE_P * eP + iPitch - RATE_D * (gy - prevGy) / dt;
  prevGy = gy;

  float outYaw = YAW_SIGN * YAW_P * (rateYawT - gz);

  // X-config mixer: FR, RR, RL, FL
  float thr = IDLE + cThr * (1.0f - IDLE);
  float mFR = thr + outRoll + outPitch + outYaw;
  float mRR = thr + outRoll - outPitch - outYaw;
  float mRL = thr - outRoll - outPitch + outYaw;
  float mFL = thr - outRoll + outPitch - outYaw;

  setMotors(mFR, mRR, mRL, mFL);
}
