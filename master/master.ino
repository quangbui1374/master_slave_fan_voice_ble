#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <quang_test_inferencing.h>
#include "driver/i2s.h"  

#include "BluetoothSerial.h"

// ====== CẤU HÌNH ======
#define SOFTWARE_GAIN       1.0f  
#define CONFIDENCE_THRESHOLD 0.7f 

// ====== PIN DEFINITIONS ======
#define I2S_WS  16
#define I2S_SCK 4
#define I2S_SD  17
#define I2S_PORT I2S_NUM_0

#define LCD_ADDR 0x27
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

BluetoothSerial SerialBT;
static const char* SLAVE_NAME = "ESP32_FAN_SLAVE";
static uint8_t targetMac[6];
static bool haveTargetMac = false;

static const uint32_t CMD_COOLDOWN_MS = 1500; 
static uint32_t lastCmdMs = 0;
int32_t *i2s_buf; 

static void lcdSetStatus(const char* status) {
  lcd.setCursor(0, 0);
  lcd.print(" Trang thai quat");
  lcd.setCursor(0, 1);
  lcd.print("                "); 
  lcd.setCursor(0, 1);
  lcd.print(status);
}

// --- BLUETOOTH HELPER ---
static void reverseMac(const uint8_t in[6], uint8_t out[6]) {
  for (int i = 0; i < 6; i++) out[i] = in[5 - i];
}

static void btSendCmd(const char* cmd) {
  if (!SerialBT.connected()) return;
  SerialBT.print(cmd);
  SerialBT.print('\n'); 
}

static void btScanPickSlave() {
  haveTargetMac = false;
  Serial.println("[BT] Scanning...");
  SerialBT.discoverAsync([](BTAdvertisedDevice* d) {
    if (!haveTargetMac && d->getName() == SLAVE_NAME) {
      const uint8_t (*m)[6] = d->getAddress().getNative();
      for (int i = 0; i < 6; i++) targetMac[i] = (*m)[i];
      haveTargetMac = true;
      Serial.println("[BT] Found Slave!");
    }
  });
  delay(6000);
  SerialBT.discoverAsyncStop();
}

static bool btConnectSlave() {
  if (!haveTargetMac) return false;
  if (SerialBT.connect(targetMac)) return true;
  uint8_t rev[6];
  reverseMac(targetMac, rev);
  return SerialBT.connect(rev);
}

static void btEnsureConnected() {
  if (SerialBT.connected()) return;
  
  // 1. Scan tìm thiết bị
  btScanPickSlave();
  if (!haveTargetMac) {
    Serial.println("[BT] Not found target.");
    return;
  }

  // 2. Thử kết nối
  bool ok = btConnectSlave();
  if (ok) {
     Serial.println("[BT] Connected!");
     delay(500);
     btSendCmd("tat"); 
  } else {
     Serial.println("[BT] Connect Fail.");
  }
}

static void motorStop() {
  lcdSetStatus("TAT QUAT");
  btSendCmd("tat");
}

static void motorForwardPWM(uint8_t duty, const char* statusText) {
  lcdSetStatus(statusText);
  if (duty == 150) btSendCmd("bat"); 
  else if (duty == 130) btSendCmd("mot");
  else if (duty == 190) btSendCmd("hai");
}

void setup() {
  Serial.begin(921600);

  Wire.begin(23, 22);
  lcd.init();
  lcd.backlight();
  lcdSetStatus("Dang ket noi...");
  // Setup I2S Buffer
  i2s_buf = (int32_t *)malloc(1024 * sizeof(int32_t));
  // Setup Bluetooth
  SerialBT.begin("ESP32_MASTER", true);
  btEnsureConnected(); 
  // Setup Mic I2S
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = -1
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);

  if (SerialBT.connected()) lcdSetStatus("San sang nghe");
}

void loop() {
  if (!SerialBT.connected()) {
    lcdSetStatus("Dang ket noi..."); 
    btEnsureConnected();             
    if (SerialBT.connected()) lcdSetStatus("San sang nghe");
    return;
  }
  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
  
  signal.get_data = [](size_t offset, size_t length, float *out) -> int {
    size_t done = 0;
    while (done < length) {
      size_t chunk = (length - done > 1024) ? 1024 : (length - done);
      size_t bytes_read = 0;
      i2s_read(I2S_PORT, (void*)i2s_buf, chunk * sizeof(int32_t), &bytes_read, 100);
      size_t samples = bytes_read / sizeof(int32_t);
      if (samples == 0) { delay(1); continue; }

      int64_t sum = 0;
      for (size_t i = 0; i < samples; i++) sum += (int16_t)(i2s_buf[i] >> 16);
      int16_t average = (int16_t)(sum / samples);

      for (size_t i = 0; i < samples; i++) {
        int16_t s16 = (int16_t)(i2s_buf[i] >> 16);
        float clean = (float)(s16 - average);
        out[done + i] = (clean * SOFTWARE_GAIN) / 32768.0f; 
        if (out[done + i] > 1.0f) out[done + i] = 1.0f;
        if (out[done + i] < -1.0f) out[done + i] = -1.0f;
      }
      done += samples;
    }
    return 0;
  };

  ei_impulse_result_t result = {0};
  if (run_classifier(&signal, &result, false) != EI_IMPULSE_OK) return;

  float max_val = 0.0f;
  int max_idx = -1;
  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > max_val) {
      max_val = result.classification[i].value;
      max_idx = (int)i;
    }
  }

  if (max_idx != -1 && max_val >= CONFIDENCE_THRESHOLD) {
    String label = result.classification[max_idx].label;
    if (label != "noise" && label != "unknown") {
      if (millis() - lastCmdMs > CMD_COOLDOWN_MS) {
        lastCmdMs = millis();
        if (label.indexOf("tat") != -1) motorStop();
        else if ((label.indexOf("bat") != -1) || (label.indexOf("mot") != -1)) motorForwardPWM(150, "BAT - SO 1");
        else if (label.indexOf("hai") != -1) motorForwardPWM(190, "SO 2");
      }
    }
  }
}