#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <quang_test_inferencing.h>
#include "driver/i2s.h"

#include "BluetoothSerial.h"

// ====== I2S MIC (INMP441) ======
#define I2S_WS  16
#define I2S_SCK 4
#define I2S_SD  17

// ====== LCD I2C ======
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2
#define I2C_SDA  23
#define I2C_SCL  22
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// ====== Bluetooth ======
BluetoothSerial SerialBT;
static const char* SLAVE_NAME = "ESP32_FAN_SLAVE";
static uint8_t targetMac[6];
static bool haveTargetMac = false;

// ====== THRESHOLD & FILTER ======
static const float THRESH = 0.65f;
static const uint32_t CMD_COOLDOWN_MS = 700;

static uint8_t currentDuty = 0;
static String lastCmd = "";
static uint32_t lastCmdMs = 0;

// Buffer đọc I2S
static int32_t i2s_buf[1024];

static void lcdSetStatus(const char* status) {
  lcd.setCursor(0, 0);
  lcd.print("Trang thai quat");
  lcd.setCursor(0, 1);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print(status);
}

static void printMac(const uint8_t mac[6]) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void reverseMac(const uint8_t in[6], uint8_t out[6]) {
  for (int i = 0; i < 6; i++) out[i] = in[5 - i];
}

static void btScanPickSlave() {
  haveTargetMac = false;

  Serial.println("[BT] Scanning 6s...");
  SerialBT.discoverAsync([](BTAdvertisedDevice* d) {
    String name = d->getName().c_str();
    String macs = d->getAddress().toString().c_str();
    Serial.printf("  Found: %s  MAC=%s\n", name.c_str(), macs.c_str());

    if (!haveTargetMac && name == SLAVE_NAME) {
      const uint8_t (*m)[6] = d->getAddress().getNative(); // core 3.3.3 kiểu này
      for (int i = 0; i < 6; i++) targetMac[i] = (*m)[i];
      haveTargetMac = true;

      Serial.print("[BT] Picked slave MAC = ");
      printMac(targetMac);
      Serial.println();
    }
  });

  delay(6000);
  SerialBT.discoverAsyncStop();
  Serial.println("[BT] Scan stop.");
}

static bool btConnectSlave() {
  if (!haveTargetMac) return false;

  Serial.print("[BT] Connect MAC normal: ");
  printMac(targetMac);
  Serial.println();
  if (SerialBT.connect(targetMac)) return true;

  uint8_t rev[6];
  reverseMac(targetMac, rev);
  Serial.print("[BT] Normal failed. Try reversed: ");
  printMac(rev);
  Serial.println();
  return SerialBT.connect(rev);
}

static void btEnsureConnected() {
  if (SerialBT.connected()) return;

  btScanPickSlave();
  if (!haveTargetMac) {
    Serial.println("[BT] KHONG TIM THAY ESP32_FAN_SLAVE");
    return;
  }

  bool ok = btConnectSlave();
  Serial.println(ok ? "[BT] Connected!" : "[BT] Connect FAILED!");
}

static void btSendCmd(const char* cmd) {
  if (!SerialBT.connected()) return;
  SerialBT.print(cmd);
  SerialBT.print('\n'); // Slave đọc theo dòng
}

// ====== giữ tên hàm y như code bạn gửi ======
static void motorStop() {
  currentDuty = 0;
  lcdSetStatus("Off");
  btSendCmd("tat");
  Serial.println(">>> FAN OFF");
}

static void motorForwardPWM(uint8_t duty, const char* statusText) {
  currentDuty = duty;
  lcdSetStatus(statusText);

  if (duty == 150) btSendCmd("bat");
  else if (duty == 200) btSendCmd("so_hai");
  else if (duty == 250) btSendCmd("so_ba");

  Serial.printf(">>> FAN duty = %u/255 | %s\n", duty, statusText);
}

static String decodeCommandFromLabel(const String& lbl) {
  if (lbl.indexOf("tat") != -1)     return "tat";
  if (lbl.indexOf("so_hai") != -1)  return "so_hai";
  if (lbl.indexOf("so_ba") != -1)   return "so_ba";
  if (lbl.indexOf("bat") != -1)     return "bat";
  return "";
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // LCD init
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();
  lcdSetStatus("Off");

  // BT MASTER
  SerialBT.begin("ESP32_VOICE_MASTER", true);
  SerialBT.setPin("1234", 4); // optional
  btEnsureConnected();

  // I2S config cho INMP441 (GIỮ NGUYÊN)
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // nếu nhiễu 1.00 -> thử ONLY_LEFT
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  Serial.println("=== MIC OK - NOI: bat / so_hai / so_ba / tat ===");
}

void loop() {
  if (!SerialBT.connected()) {
    btEnsureConnected();
  }

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;

  signal.get_data = [](size_t offset, size_t length, float *out) -> int {
    (void)offset;

    size_t done = 0;
    while (done < length) {
      size_t chunk = min((size_t)1024, length - done);
      size_t bytes_read = 0;

      i2s_read(I2S_NUM_0, (void*)i2s_buf, chunk * sizeof(int32_t), &bytes_read, portMAX_DELAY);

      size_t samples = bytes_read / sizeof(int32_t);
      for (size_t i = 0; i < samples; i++) {
        out[done + i] = (float)(i2s_buf[i] >> 16);
      }
      done += samples;
    }
    return 0;
  };

  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR rc = run_classifier(&signal, &result, false);
  if (rc != EI_IMPULSE_OK) {
    Serial.printf("run_classifier error: %d\n", rc);
    delay(50);
    return;
  }

  float max_val = 0.0f;
  int max_idx = 0;
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > max_val) {
      max_val = result.classification[i].value;
      max_idx = (int)i;
    }
  }

  String lbl = String(result.classification[max_idx].label);
  Serial.printf("→ %s: %.2f | duty=%u\n", lbl.c_str(), max_val, currentDuty);

  if (max_val >= THRESH) {
    String cmd = decodeCommandFromLabel(lbl);
    if (cmd.length()) {
      uint32_t now = millis();
      if (cmd != lastCmd && (now - lastCmdMs) > CMD_COOLDOWN_MS) {
        lastCmd = cmd;
        lastCmdMs = now;

        if (cmd == "tat") {
          motorStop();
        } else if (cmd == "bat") {
          motorForwardPWM(150, "On");
        } else if (cmd == "so_hai") {
          motorForwardPWM(200, "So 2");
        } else if (cmd == "so_ba") {
          motorForwardPWM(250, "So 3");
        }

        Serial.printf(">>> CMD = %s\n", cmd.c_str());
      }
    }
  }

  delay(80);
}
