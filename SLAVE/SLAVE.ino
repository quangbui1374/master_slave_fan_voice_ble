#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ====== L298N (1 kênh) ======
#define ENA_PIN  17       // PWM pin
#define IN1_PIN  16
#define IN2_PIN  4

// PWM (LEDC) - ESP32 core 3.3.3
#define FAN_PWM_FREQ    25000
#define FAN_PWM_RES     8        // duty 0..255

static uint8_t currentDuty = 0;

static void motorStop() {
  ledcWrite(ENA_PIN, 0);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  currentDuty = 0;
  Serial.println(">>> FAN OFF");
}

static void motorForwardPWM(uint8_t duty, const char* statusText) {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);

  currentDuty = duty;
  ledcWrite(ENA_PIN, duty);

  Serial.printf(">>> FAN duty = %u/255 | %s\n", duty, statusText);
}

static void handleCmd(const String& cmd) {
  if (cmd == "tat") {
    motorStop();
  } else if (cmd == "bat") {
    motorForwardPWM(150, "On");
  } else if (cmd == "so_hai") {
    motorForwardPWM(200, "So 2");
  } else if (cmd == "so_ba") {
    motorForwardPWM(250, "So 3");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // L298N pins
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);

  // PWM init (ENA) - core 3.3.3
  ledcAttach(ENA_PIN, FAN_PWM_FREQ, FAN_PWM_RES);
  ledcWrite(ENA_PIN, 0);

  // Bluetooth SPP SLAVE
  SerialBT.begin("ESP32_FAN_SLAVE");
  SerialBT.setPin("1234", 4);  // optional

  motorStop();
  Serial.println("=== SLAVE OK - BT name: ESP32_FAN_SLAVE ===");
}

void loop() {
  static String line = "";

  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c == '\r') continue;

    if (c == '\n') {
      line.trim();
      if (line.length()) {
        Serial.printf(">>> CMD = %s\n", line.c_str());
        handleCmd(line);
      }
      line = "";
    } else {
      line += c;
      if (line.length() > 32) line = "";
    }
  }
}
