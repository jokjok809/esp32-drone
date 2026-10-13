/*
  ESP32-C3 Super Mini - MOTOR TEST (WALANG GYRO)
  Kontrolado ng ESP-Drone app via WiFi/UDP (CRTP)

  LAYUNIN: Para lang tingnan kung gumagalaw ang 4 motors at tama ang
  direksyon nila. WALA itong stabilization - sunod lang nang diretso
  ang bawat motor sa throttle ng app. HINDI ito pang-aktwal na lipad.

  Throttle (mula sa app) -> parehong lakas ang apat na motors.
  Yaw stick (kaliwa-kanan) -> ginagamit para pumili kung aling motor
  lang ang susubukan nang isa-isa (tingnan sa ibaba).

  Drone : 192.168.43.42  UDP port 2390
  Commander (port 3): float roll, float pitch, float yaw, uint16 thrust

  WIRING
  Motors (logic-level N-MOSFET + flyback diode + 100k pulldown sa gate):
    M1 Front-Right -> GPIO0
    M2 Rear-Right  -> GPIO1
    M3 Rear-Left   -> GPIO3
    M4 Front-Left  -> GPIO4

  BABALA: TANGGALIN ANG PROPELLER BAGO SUBUKAN!
*/

#include <WiFi.h>
#include <WiFiUdp.h>

// ---------- Network ----------
const char* AP_SSID = "ESP-DRONE_C3";
const char* AP_PASS = "12345678";
IPAddress DRONE_IP(192, 168, 43, 42);
IPAddress NETMASK(255, 255, 255, 0);
const uint16_t UDP_PORT = 2390;
WiFiUDP udp;

// ---------- Pins ----------
const int MOTOR_PIN[4] = {0, 1, 3, 4};   // FR, RR, RL, FL
const int PWM_FREQ = 20000;
const int PWM_RES  = 8;

// Itaas ang limit na ito para sa test lang (ligtas na mababang lakas)
float MAX_TEST_POWER = 0.35;   // 0.0 - 1.0 (35% max habang tine-test)

// ---------- State ----------
float cThr = 0, cYaw = 0;
bool armed = false;
uint32_t lastCmd = 0;

void setMotors(float m0, float m1, float m2, float m3) {
  float m[4] = {m0, m1, m2, m3};
  for (int i = 0; i < 4; i++) {
    m[i] = constrain(m[i], 0.0f, 1.0f);
    ledcWrite(MOTOR_PIN[i], (int)(m[i] * 255));
  }
}
void motorsOff() { setMotors(0, 0, 0, 0); }

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
          float y;
          uint16_t th;
          memcpy(&y,  &buf[9], 4);
          memcpy(&th, &buf[13], 2);

          cYaw = y;
          cThr = th / 65535.0f;
          lastCmd = millis();

          if (th == 0) armed = true;
        }
      }
    }
    n = udp.parsePacket();
  }
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    ledcAttach(MOTOR_PIN[i], PWM_FREQ, PWM_RES);
    ledcWrite(MOTOR_PIN[i], 0);
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(DRONE_IP, DRONE_IP, NETMASK);
  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 2);
  WiFi.setTxPower(WIFI_POWER_11dBm);
  udp.begin(UDP_PORT);

  Serial.println("MOTOR TEST MODE - walang gyro, walang stabilization");
  Serial.printf("WiFi: %s  UDP: %s:%u\n", AP_SSID, DRONE_IP.toString().c_str(), UDP_PORT);
}

void loop() {
  handleUDP();

  // Failsafe: mawala ang app > 500ms
  if (millis() - lastCmd > 500) { armed = false; cThr = 0; }

  if (!armed || cThr < 0.03f) {
    motorsOff();
    delay(10);
    return;
  }

  float power = cThr * MAX_TEST_POWER;

  if (cYaw > 50) {
    setMotors(power, 0, 0, 0);      // M1 FR lang
    Serial.println("Testing M1 (Front-Right)");
  } else if (cYaw < -50) {
    setMotors(0, 0, power, 0);      // M3 RL lang
    Serial.println("Testing M3 (Rear-Left)");
  } else {
    setMotors(power, power, power, power);  // lahat
    Serial.printf("All motors @ %.0f%%\n", power * 100);
  }

  delay(20);
}
