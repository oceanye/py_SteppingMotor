#include "stepper_encoders.h"
#include "config.h"
#include <Arduino.h>

#if defined(DRIVE_MODE_GEAR)
#include <Wire.h>

// This module is diagnostic only. Nothing here calls stepper_move/abort or
// changes PUL/DIR. Installing TCA9548A/AS5600 hardware therefore cannot switch
// a DM442 axis into closed-loop control.
struct EncoderSnapshot {
  bool sensor_online;
  bool magnet_detected;
  bool initialized;
  uint16_t raw_angle;
  uint16_t previous_raw;
  int64_t multi_turn_counts;
  uint32_t last_success_ms;
  uint32_t error_count;
};

static EncoderSnapshot s_encoders[NUM_STEPPER_ENCODERS] = {};
static portMUX_TYPE s_snapshot_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_poll_task = nullptr;
static bool s_mux_online = false;       // guarded by s_snapshot_mux
static bool s_task_started = false;     // initialized before protocol becomes available

static bool probe_device(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

static bool select_mux_channel(uint8_t channel) {
  Wire.beginTransmission(TCA9548A_I2C_ADDR);
  Wire.write((uint8_t)(1U << channel));
  return Wire.endTransmission() == 0;
}

// Read STATUS + RAW ANGLE high/low in one transaction. STATUS.MD (bit 5)
// reports whether AS5600 currently detects a magnet.
static bool read_as5600(uint16_t& raw_angle, bool& magnet_detected) {
  Wire.beginTransmission(AS5600_I2C_ADDR);
  Wire.write((uint8_t)AS5600_STATUS_REG);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)AS5600_I2C_ADDR, (uint8_t)3) != 3) return false;
  const uint8_t sensor_status = Wire.read();
  const uint8_t high = Wire.read();
  const uint8_t low = Wire.read();
  raw_angle = (uint16_t)(((uint16_t)(high & 0x0F) << 8) | low);
  magnet_detected = (sensor_status & 0x20U) != 0;
  return true;
}

static void mark_mux_state(bool online) {
  portENTER_CRITICAL(&s_snapshot_mux);
  s_mux_online = online;
  if (!online) {
    for (int axis = 0; axis < NUM_STEPPER_ENCODERS; ++axis) {
      s_encoders[axis].sensor_online = false;
      s_encoders[axis].magnet_detected = false;
      ++s_encoders[axis].error_count;
    }
  }
  portEXIT_CRITICAL(&s_snapshot_mux);
}

static void record_sample(int axis, bool online, uint16_t raw, bool magnet) {
  portENTER_CRITICAL(&s_snapshot_mux);
  EncoderSnapshot& snapshot = s_encoders[axis];
  snapshot.sensor_online = online;
  snapshot.magnet_detected = online && magnet;
  if (!online) {
    ++snapshot.error_count;
  } else {
    if (!snapshot.initialized) {
      snapshot.multi_turn_counts = raw;
      snapshot.initialized = true;
    } else {
      int32_t delta = (int32_t)raw - (int32_t)snapshot.previous_raw;
      if (delta > 2048) delta -= 4096;
      else if (delta < -2048) delta += 4096;
      snapshot.multi_turn_counts += delta;
    }
    snapshot.previous_raw = raw;
    snapshot.raw_angle = raw;
    snapshot.last_success_ms = millis();
  }
  portEXIT_CRITICAL(&s_snapshot_mux);
}

static void encoder_poll_task(void*) {
  for (;;) {
    if (!probe_device(TCA9548A_I2C_ADDR)) {
      mark_mux_state(false);
      vTaskDelay(pdMS_TO_TICKS(STEPPER_ENCODER_OFFLINE_RETRY_MS));
      continue;
    }
    mark_mux_state(true);
    for (int axis = 0; axis < NUM_STEPPER_ENCODERS; ++axis) {
      uint16_t raw = 0;
      bool magnet = false;
      const bool online = select_mux_channel((uint8_t)axis) && read_as5600(raw, magnet);
      record_sample(axis, online, raw, magnet);
    }
    vTaskDelay(pdMS_TO_TICKS(STEPPER_ENCODER_POLL_MS));
  }
}

void stepper_encoders_init() {
#if STEPPER_ENCODER_DIAGNOSTICS_ENABLED
  static_assert(NUM_STEPPER_ENCODERS <= 8, "TCA9548A only has channels 0..7");
  static_assert(STEPPER_ENCODER_CLOSED_LOOP_ENABLED == 0,
                "Closed-loop DM442 control is intentionally not implemented");
  Wire.begin(PIN_STEPPER_ENCODER_I2C_SDA, PIN_STEPPER_ENCODER_I2C_SCL);
  Wire.setClock(STEPPER_ENCODER_I2C_HZ);
  s_task_started = xTaskCreatePinnedToCore(
      encoder_poll_task, "stepEnc", STEPPER_ENCODER_TASK_STACK, nullptr,
      STEPPER_ENCODER_TASK_PRIORITY, &s_poll_task,
      STEPPER_ENCODER_TASK_CORE) == pdPASS;
  if (!s_task_started) Serial.println("ERR:optional encoder diagnostic task init failed");
  else Serial.println("[ENC] optional TCA9548A/AS5600 diagnostics enabled; open-loop step control unchanged");
#else
  s_task_started = false;
#endif
}

bool stepper_encoder_get_status(int axis, StepperEncoderStatus& status) {
  if (axis < 0 || axis >= NUM_STEPPER_ENCODERS) return false;
  status = {};
  status.diagnostics_enabled = STEPPER_ENCODER_DIAGNOSTICS_ENABLED && s_task_started;
  portENTER_CRITICAL(&s_snapshot_mux);
  const EncoderSnapshot snapshot = s_encoders[axis];
  status.mux_online = s_mux_online;
  portEXIT_CRITICAL(&s_snapshot_mux);
  status.sensor_online = snapshot.sensor_online;
  status.magnet_detected = snapshot.magnet_detected;
  status.raw_angle = snapshot.raw_angle;
  status.multi_turn_counts = snapshot.multi_turn_counts;
  status.age_ms = snapshot.initialized ? millis() - snapshot.last_success_ms : UINT32_MAX;
  status.error_count = snapshot.error_count;
  return true;
}

#endif  // DRIVE_MODE_GEAR
