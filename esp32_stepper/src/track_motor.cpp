#include "track_motor.h"
#include "config.h"
#include <Arduino.h>

#if defined(DRIVE_MODE_GEAR)

static SemaphoreHandle_t s_track_mutex = nullptr;
static TrackDirection s_direction = TRACK_STOPPED;
static uint8_t s_duty_pct = 0;
static uint32_t s_expires_ms = 0;
static TaskHandle_t s_track_task = nullptr;
static bool s_ready = false;

static void write_outputs(TrackDirection direction, uint8_t duty_pct) {
  const uint32_t pwm = (uint32_t)duty_pct * TRACK_PWM_MAX / 100U;
  if (direction == TRACK_FORWARD) {
    ledcWrite(TRACK_D_LEDC_CH_IN1, pwm);
    ledcWrite(TRACK_D_LEDC_CH_IN2, 0);
  } else if (direction == TRACK_REVERSE) {
    ledcWrite(TRACK_D_LEDC_CH_IN1, 0);
    ledcWrite(TRACK_D_LEDC_CH_IN2, pwm);
  } else {
    ledcWrite(TRACK_D_LEDC_CH_IN1, 0);
    ledcWrite(TRACK_D_LEDC_CH_IN2, 0);
  }
}

static void safety_task(void*) {
  for (;;) {
    bool timed_out = false;
    xSemaphoreTake(s_track_mutex, portMAX_DELAY);
    if (s_direction != TRACK_STOPPED &&
        (int32_t)(millis() - s_expires_ms) >= 0) {
      s_direction = TRACK_STOPPED;
      s_duty_pct = 0;
      timed_out = true;
    }
    const TrackDirection direction = s_direction;
    const uint8_t duty_pct = s_duty_pct;
    write_outputs(direction, duty_pct);
    xSemaphoreGive(s_track_mutex);
    if (timed_out) Serial.println("TRACK,D,TIMEOUT");
    vTaskDelay(pdMS_TO_TICKS(TRACK_SAFETY_TICK_MS));
  }
}

void track_motor_init() {
  s_track_mutex = xSemaphoreCreateMutex();
  ledcSetup(TRACK_D_LEDC_CH_IN1, TRACK_PWM_FREQ_HZ, TRACK_PWM_RES_BITS);
  ledcSetup(TRACK_D_LEDC_CH_IN2, TRACK_PWM_FREQ_HZ, TRACK_PWM_RES_BITS);
  ledcAttachPin(PIN_TRACK_D_IN1, TRACK_D_LEDC_CH_IN1);
  ledcAttachPin(PIN_TRACK_D_IN2, TRACK_D_LEDC_CH_IN2);
  write_outputs(TRACK_STOPPED, 0);
  if (s_track_mutex == nullptr) {
    Serial.println("ERR:track mutex init failed");
    return;
  }
  s_ready = xTaskCreatePinnedToCore(safety_task, "trackD", 2048, nullptr,
                                    3, &s_track_task, 0) == pdPASS;
  if (!s_ready) Serial.println("ERR:track safety task init failed");
}

bool track_motor_drive(TrackDirection direction, uint8_t duty_pct, uint32_t lease_ms) {
  if (!s_ready || (direction != TRACK_FORWARD && direction != TRACK_REVERSE) ||
      duty_pct < 1 || duty_pct > 100 || lease_ms < TRACK_MIN_LEASE_MS ||
      lease_ms > TRACK_MAX_LEASE_MS) return false;
  xSemaphoreTake(s_track_mutex, portMAX_DELAY);
  s_duty_pct = duty_pct;
  s_expires_ms = millis() + lease_ms;
  s_direction = direction;
  write_outputs(s_direction, s_duty_pct);
  xSemaphoreGive(s_track_mutex);
  return true;
}

void track_motor_stop() {
  if (s_track_mutex == nullptr) { write_outputs(TRACK_STOPPED, 0); return; }
  xSemaphoreTake(s_track_mutex, portMAX_DELAY);
  s_direction = TRACK_STOPPED;
  s_duty_pct = 0;
  s_expires_ms = 0;
  write_outputs(TRACK_STOPPED, 0);
  xSemaphoreGive(s_track_mutex);
}

TrackStatus track_motor_get_status() {
  TrackStatus status = {TRACK_STOPPED, 0, 0};
  if (!s_ready) return status;
  xSemaphoreTake(s_track_mutex, portMAX_DELAY);
  status.direction = s_direction;
  status.duty_pct = s_duty_pct;
  const int32_t remaining = (s_direction == TRACK_STOPPED)
      ? 0 : (int32_t)(s_expires_ms - millis());
  status.remaining_ms = remaining > 0 ? (uint32_t)remaining : 0;
  xSemaphoreGive(s_track_mutex);
  return status;
}

#endif
