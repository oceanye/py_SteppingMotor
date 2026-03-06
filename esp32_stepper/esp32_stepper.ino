#include <Adafruit_NeoPixel.h>

#define PUL_PIN  4
#define DIR_PIN  5
#define LED_PIN  48  // ESP32-S3 Dev 板载 WS2812（LOLIN S3 Mini 改 47）
#define LED_NUM  1

Adafruit_NeoPixel led(LED_NUM, LED_PIN, NEO_GRB + NEO_KHZ800);

void setLED(uint8_t r, uint8_t g, uint8_t b) {
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

void setup() {
  Serial.begin(115200);
  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(PUL_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  led.begin();
  setLED(0, 30, 0);  // 绿色 = 待机就绪

  Serial.println("ESP32 Stepper Ready");
}

// steps: 步数, direction: 1正转/0反转, delay_ms: 每步间隔(ms)
void moveMotor(int steps, int direction, int delay_ms) {
  setLED(0, 0, 40);  // 蓝色 = 运动中
  digitalWrite(DIR_PIN, direction);
  delayMicroseconds(100);
  for (int i = 0; i < steps; i++) {
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(50);
    digitalWrite(PUL_PIN, LOW);
    delay(delay_ms);
  }
  setLED(0, 30, 0);  // 绿色 = 运动完成
  Serial.println("OK");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // 格式: MOVE,steps,direction,delay_ms
    // 示例: MOVE,200,1,10
    if (cmd.startsWith("MOVE,")) {
      int p1 = cmd.indexOf(',');
      int p2 = cmd.indexOf(',', p1 + 1);
      int p3 = cmd.indexOf(',', p2 + 1);

      if (p1 > 0 && p2 > 0 && p3 > 0) {
        int steps     = cmd.substring(p1 + 1, p2).toInt();
        int direction = cmd.substring(p2 + 1, p3).toInt();
        int delay_ms  = cmd.substring(p3 + 1).toInt();
        moveMotor(steps, direction, delay_ms);
      } else {
        Serial.println("ERR:bad format");
      }
    } else {
      Serial.println("ERR:unknown command");
    }
  }
}
