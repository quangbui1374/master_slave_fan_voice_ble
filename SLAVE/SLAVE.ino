#include <Arduino.h>
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

#define ENA_PIN    17      
#define IN1_PIN    16
#define IN2_PIN    4

// Cấu hình PWM cho ESP32 Core v2.x
#define FAN_PWM_FREQ    25000
#define FAN_PWM_RES     8      
#define FAN_PWM_CHANNEL 0       

static uint8_t currentDuty = 0;

static void motorStop() {
  ledcWrite(FAN_PWM_CHANNEL, 0);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  currentDuty = 0;
  Serial.println(">>> FAN OFF");
}

static void motorForwardPWM(uint8_t duty, const char* statusText) {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);

  currentDuty = duty;
  // Với Core v2, ta write vào CHANNEL
  ledcWrite(FAN_PWM_CHANNEL, duty);

  Serial.printf(">>> FAN duty = %u/255 | %s\n", duty, statusText);
}

static void handleCmd(const String& cmd) {
  if (cmd == "tat") {
    motorStop();
  } else if (cmd == "bat") {
    motorForwardPWM(150, "On");
  } else if (cmd == "mot") {
    motorForwardPWM(150, "So 1");
  } else if (cmd == "hai") {
    motorForwardPWM(200, "So 2");
  } 
}

void setup() {
  Serial.begin(921600);
  delay(300);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);


  ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQ, FAN_PWM_RES);
  ledcAttachPin(ENA_PIN, FAN_PWM_CHANNEL);
  ledcWrite(FAN_PWM_CHANNEL, 0);

  // Bluetooth SPP SLAVE
  SerialBT.begin("ESP32_FAN_SLAVE");
  SerialBT.setPin("1234");
  motorStop();
  Serial.println("=== SLAVE OK - BT name: ESP32_FAN_SLAVE ===");
}

void loop() {
  if (!SerialBT.connected()) {
    if (currentDuty > 0) {
      Serial.println("!!! MAT KET NOI -> DUNG KHAN CAP !!!");
      motorStop();
    }
    delay(100);
    return;
  }

  // --- XỬ LÝ LỆNH KHI CÓ KẾT NỐI ---
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