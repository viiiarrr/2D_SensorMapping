#include "ServoMotor.h"
#include "imu.h"
#include "ultrasonik.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <math.h>

// =====================
// KONFIGURASI WIFI & UDP
// =====================
const char *ssid = "rytz";
const char *password = "admin12345";
const char *laptop_ip = "192.168.137.1";
const int udp_port = 5005;
WiFiUDP udp;

// =====================
// OBJEK SENSOR & AKTUATOR
// =====================
Servo360Motor servo;
Sensor ultrasonicArray;
ImuSensor imu;

// =====================
// HOMING
// =====================
#define HOMING_PIN 15
volatile unsigned long sweepStartTime = 0;

// =====================
// STATE ARAH SERVO
// =====================
enum MotorState { DIR_CW, DIR_CCW };
MotorState currentState = DIR_CW;
float sweepStartYaw = 0; // Yaw saat mulai sweep
#define SWEEP_DEG 720.0  // 2 putaran penuh = 720°


void IRAM_ATTR homingISR() {
  if (millis() - sweepStartTime > 500) {
    sweepStartTime = millis();
  }
}

void bacaDanKirim() {
  imu.update();
  ultrasonicArray.readAll();

  float currentAngle = imu.getYaw();

  // Format: sudut_platform,jarak1,...,jarak8
  String out = String(currentAngle, 1) + ",";
  for (int i = 0; i < NUM_SENSOR; i++) {
    float d = ultrasonicArray.getDistance(i);
    if (d == -1.0)
      d = 0.0;
    out += String(d, 1);
    if (i < NUM_SENSOR - 1)
      out += ",";
  }

  if (WiFi.status() == WL_CONNECTED) {
    udp.beginPacket(laptop_ip, udp_port);
    udp.print(out);
    udp.endPacket();
  }

  Serial.print("YAW: ");
  Serial.print(currentAngle, 2);
  Serial.print(" deg | ");
  Serial.println(out);
}

void setup() {
  Serial.begin(115200);
  
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  int att = 0;
  while (WiFi.status() != WL_CONNECTED && att < 20) {
    delay(500);
    Serial.print(".");
    att++;
  }
  Serial.println(WiFi.status() == WL_CONNECTED
                     ? "\nWi-Fi OK: " + WiFi.localIP().toString()
                     : "\nWi-Fi GAGAL. Lanjut tanpa WiFi...");

  ultrasonicArray.begin();
  pinMode(HOMING_PIN, INPUT_PULLUP);// Homing switch
  attachInterrupt(digitalPinToInterrupt(HOMING_PIN), homingISR, FALLING);
  imu.begin();
  
  servo.begin();
  delay(1500);
  servo.resetAngle();

  Serial.println("=== MULAI PEMETAAN — SERVO BERPUTAR ===");
  sweepStartTime = millis();
  servo.rotateCW();
}

void loop() {

  imu.update();
  float currentAngle = imu.getYaw();

  // Pakai abs agar berlaku untuk CW (yaw negatif) maupun CCW (yaw positif)
  float yawTraveled = abs(currentAngle - sweepStartYaw);

  if (yawTraveled >= SWEEP_DEG) {
    if (currentState == DIR_CW) {
      Serial.println(">> 2 Putaran CW selesai — Berbalik ke CCW");
      servo.stop();
      delay(500);
      sweepStartYaw =
          currentAngle; // Catat titik awal sweep baru (TIDAK reset yaw)
      currentState = DIR_CCW;
      servo.rotateCCW();
    } else {
      Serial.println(">> 2 Putaran CCW selesai — Berbalik ke CW");
      servo.stop();
      delay(500);
      sweepStartYaw =
          currentAngle; // Catat titik awal sweep baru (TIDAK reset yaw)
      currentState = DIR_CW;
      servo.rotateCW();
    }
    return;
  }

  ultrasonicArray.readAll();

  // Format: sudut_platform,jarak1,...,jarak8
  String out = String(currentAngle, 1) + ",";
  for (int i = 0; i < NUM_SENSOR; i++) {
    float d = ultrasonicArray.getDistance(i);
    if (d == -1.0)
      d = 0.0;
    out += String(d, 1);
    if (i < NUM_SENSOR - 1)
      out += ",";
  }

  if (WiFi.status() == WL_CONNECTED) {
    udp.beginPacket(laptop_ip, udp_port);
    udp.print(out);
    udp.endPacket();
  }

  Serial.print(currentState == DIR_CW ? "[CW] " : "[CCW] ");
  Serial.print("YAW: ");
  Serial.print(currentAngle, 1);
  Serial.print(" | Traveled: ");
  Serial.print(yawTraveled, 1);
  Serial.print(" | ");
  Serial.println(out);

  delay(20);
}
