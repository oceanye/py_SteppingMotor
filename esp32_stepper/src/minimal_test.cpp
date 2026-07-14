// esp32_stepper/src/minimal_test.cpp
// 最小测试 sketch：只在 [env:esp32s3_minimal] 下编译。
// 目的：验证 ESP32 板子硬件 + 烧录链路是否健全。
// 期望行为：每秒打印一行 "tick N"，闪 LED（如果板载 LED 在 GPIO 48）。
//
// 编译并烧录：
//   pio run -e esp32s3_minimal -t upload
#ifdef MINIMAL_TEST_BUILD
#include <Arduino.h>

#ifndef LED_PIN
#define LED_PIN 48   // ESP32-S3-DevKitC-1 板载 RGB LED 的 data pin（部分版本）
#endif

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== MINIMAL TEST BOOT ===");
  Serial.print("Build: "); Serial.print(__DATE__); Serial.print(" "); Serial.println(__TIME__);
  pinMode(LED_PIN, OUTPUT);
}

static uint32_t s_tick = 0;
void loop() {
  digitalWrite(LED_PIN, (s_tick & 1) ? HIGH : LOW);
  Serial.print("tick "); Serial.println(s_tick++);
  delay(1000);
}
#endif
