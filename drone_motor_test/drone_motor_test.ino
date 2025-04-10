#include <Arduino.h>
#include <ESP32Servo.h>

#define ESC_PIN 16
Servo esc;

void setup() {
  Serial.begin(115200);
  esc.attach(ESC_PIN, 1000, 2000);
  
  Serial.println("ESC 캘리브레이션: 최대값 설정 (2000us)");
  esc.writeMicroseconds(2000);  // 최대값 먼저 설정
  delay(5000);                  // 최대값 유지 (ESC 캘리브레이션 진입)
  
  Serial.println("ESC 캘리브레이션: 최소값 설정 (1000us)");
  esc.writeMicroseconds(1000);  // 최소값 설정
  delay(5000);                  // 캘리브레이션 종료
}

void loop() {
  Serial.println("캘리브레이션 후 테스트 중속 (1500us)");
  esc.writeMicroseconds(2000);  // 중간 속도로 회전
  delay(5000);

  Serial.println("캘리브레이션 후 테스트 중속 (1500us)");
  esc.writeMicroseconds(1500);  // 중간 속도로 회전
  delay(5000);

  Serial.println("캘리브레이션 후 테스트 중속 (1500us)");
  esc.writeMicroseconds(1200);  // 중간 속도로 회전
  delay(5000);

  Serial.println("모터 정지 (1000us)");
  esc.writeMicroseconds(1000);  // 모터 정지
  delay(5000);
}
