#include "serial_tx.h"

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {

constexpr size_t kLineBufferSize = 256;
SemaphoreHandle_t s_tx_mutex = nullptr;

class TxLock {
 public:
  TxLock() : locked_(s_tx_mutex != nullptr &&
                     xSemaphoreTake(s_tx_mutex, portMAX_DELAY) == pdTRUE) {}
  ~TxLock() {
    if (locked_) xSemaphoreGive(s_tx_mutex);
  }

  TxLock(const TxLock&) = delete;
  TxLock& operator=(const TxLock&) = delete;

 private:
  bool locked_;
};

}  // namespace

bool serial_tx_init() {
  if (s_tx_mutex == nullptr) s_tx_mutex = xSemaphoreCreateMutex();
  return s_tx_mutex != nullptr;
}

void serial_tx_line(const char* line) {
  if (line == nullptr) line = "";
  const TxLock lock;
  // Keep both the payload and CRLF inside the same critical section. Arduino's
  // HardwareSerial preserves the order of bytes already placed in its TX
  // buffer, so flushing here would only block motion/control tasks needlessly.
  Serial.println(line);
}

void serial_tx_printf(const char* format, ...) {
  char line[kLineBufferSize];
  va_list args;
  va_start(args, format);
  const int written = vsnprintf(line, sizeof(line), format, args);
  va_end(args);

  if (written < 0) {
    serial_tx_line("ERR:serial format");
    return;
  }
  if ((size_t)written >= sizeof(line)) {
    serial_tx_line("ERR:serial line too long");
    return;
  }
  line[sizeof(line) - 1] = '\0';
  serial_tx_line(line);
}
