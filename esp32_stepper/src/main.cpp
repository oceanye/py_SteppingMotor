#include <Arduino.h>

#define PUL_PIN  5   // PUL+ 脉冲
#define DIR_PIN  6   // DIR+ 方向

// ====================================================================
// 诊断固件 v1 —— 用于定位"指令OK但电机不转"的故障层
// 启动后先自动跑两段诊断，然后进入正常 Serial 指令循环
// ====================================================================

void moveMotor(int steps, int direction, int delay_ms) {
  digitalWrite(DIR_PIN, direction);
  delayMicroseconds(100);
  for (int i = 0; i < steps; i++) {
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(50);
    digitalWrite(PUL_PIN, LOW);
    delay(delay_ms);
  }
  Serial.println("OK");
}

void runDiagnostics() {
  // ---- DIAG 1: 静态电平测试 ----
  Serial.println();
  Serial.println("===== DIAG 1: 静态电平测试 =====");
  Serial.println(">>> PUL+ 即将置 HIGH 持续 10 秒");
  Serial.println(">>> 请用万用表直流电压档 (20V) 黑笔夹 ESP32 GND，红笔依次测:");
  Serial.println("    a) ESP32 GPIO5 焊盘        期望 ~3.3V");
  Serial.println("    b) DM422 PUL+ 端子         期望 ~3.3V");
  Serial.println("    c) DM422 PUL+ 对 PUL-      期望 ~3.3V");
  delay(2000);
  digitalWrite(PUL_PIN, HIGH);
  Serial.println(">>> [PUL=HIGH] 开始测量 ...");
  delay(10000);

  Serial.println(">>> PUL+ 置 LOW 持续 3 秒 (应读到 ~0V)");
  digitalWrite(PUL_PIN, LOW);
  delay(3000);

  // ---- DIAG 2: DIR 引脚电平测试 ----
  Serial.println();
  Serial.println("===== DIAG 2: DIR 静态电平测试 =====");
  Serial.println(">>> DIR+ 即将置 HIGH 持续 5 秒，请测 DM422 DIR+ 对 GND 期望 ~3.3V");
  delay(1000);
  digitalWrite(DIR_PIN, HIGH);
  delay(5000);
  digitalWrite(DIR_PIN, LOW);

  // ---- DIAG 3: 超慢速大宽度脉冲（正反对称，净位移=0） ----
  Serial.println();
  Serial.println("===== DIAG 3: 慢速大脉冲测试 (正转5 + 反转5) =====");
  Serial.println(">>> 每脉冲 HIGH 20ms + LOW 80ms，应能听到咔声");
  delay(2000);

  Serial.println("  [正转 5 步]");
  digitalWrite(DIR_PIN, HIGH);
  delay(1);
  for (int i = 0; i < 5; i++) {
    digitalWrite(PUL_PIN, HIGH);
    delay(20);
    digitalWrite(PUL_PIN, LOW);
    delay(80);
  }

  delay(500);
  Serial.println("  [反转 5 步]");
  digitalWrite(DIR_PIN, LOW);
  delay(1);
  for (int i = 0; i < 5; i++) {
    digitalWrite(PUL_PIN, HIGH);
    delay(20);
    digitalWrite(PUL_PIN, LOW);
    delay(80);
  }

  // ---- DIAG 4: 中速脉冲（正反对称，幅度 1/10，净位移=0） ----
  Serial.println();
  Serial.println("===== DIAG 4: 中速脉冲测试 (正转500 + 反转500) =====");
  Serial.println(">>> 每步 2ms，单向总时长约 1 秒，观察转动是否明显");
  delay(1000);

  Serial.println("  [正转 500 步]");
  digitalWrite(DIR_PIN, HIGH);
  delay(1);
  for (int i = 0; i < 500; i++) {
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(50);
    digitalWrite(PUL_PIN, LOW);
    delay(2);
  }

  delay(800);
  Serial.println("  [反转 500 步，回到起点]");
  digitalWrite(DIR_PIN, LOW);
  delay(1);
  for (int i = 0; i < 500; i++) {
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(50);
    digitalWrite(PUL_PIN, LOW);
    delay(2);
  }

  Serial.println();
  Serial.println("===== 诊断结束，进入正常 MOVE 指令循环 =====");
  Serial.println("可发送: MOVE,steps,dir,delay_ms  例如 MOVE,400,1,20");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(PUL_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  delay(500);
  Serial.println();
  Serial.println("ESP32 Stepper Ready");
  Serial.print("PUL_PIN = GPIO"); Serial.println(PUL_PIN);
  Serial.print("DIR_PIN = GPIO"); Serial.println(DIR_PIN);
  Serial.println("发送 MOVE,steps,dir,delay_ms  或  DIAG");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // 格式: MOVE,steps,direction,delay_ms
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
    } else if (cmd == "DIAG") {
      runDiagnostics();
    } else if (cmd.length() > 0) {
      Serial.println("ERR:unknown command");
    }
  }
}
