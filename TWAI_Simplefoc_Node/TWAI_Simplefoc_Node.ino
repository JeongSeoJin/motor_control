#include <SimpleFOC.h>
#include <ESP32-TWAI-CAN.hpp>
#include <math.h>

// ----- 노드 ID 설정 -----
// 각 노드는 고유의 NODE_ID를 가져야 함 (예: 1, 2, 3, ...)
#define NODE_ID 4

// ----- 핀 매핑 -----
// DRV8302 관련 핀
#define INH_A    14
#define INH_B    27
#define INH_C    26
#define EN_GATE  13
#define M_PWM    33
#define M_OC     25
#define OC_ADJ   32
#define OC_GAIN  5
// 전류 센싱 핀 (ESP32 ADC)
#define IOUTA    4
#define IOUTB    12
#define IOUTC    15
// CAN 통신 핀 (TWAI)
#define CAN_TX   17
#define CAN_RX   16

// ----- 모터, 드라이버, 센서 객체 생성 -----
BLDCMotor motor = BLDCMotor(11);           // 예: 22극 모터 → 11 pole pairs
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C, EN_GATE);
LowsideCurrentSense cs = LowsideCurrentSense(0.005f, 12.22f, IOUTA, IOUTB, IOUTC);
MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);

// 시리얼 커맨드 인터페이스 (모든 노드에서 활성)
Commander command = Commander(Serial);

// ----- 제어용 전역 변수 -----
float theta_desired = 0.0f;  
float Kp = 2.0f;       // [A/rad]
float Cd = 0.2f;       // [A/(rad/s)]
float Kt = 60 / (2 * PI * 380);
float angle_offset = 0.0;

// ----- CAN 메시지 정의 -----
// 메시지 포맷: data[0]: target ID, data[1]: 파라미터 ID, data[2..5]: float 값 (little-endian)
#define PARAM_CAN_ID      0x100
#define PARAM_ID_REF_ANGLE 0x01  // 'r'
#define PARAM_ID_KP        0x02  // 'k'
#define PARAM_ID_CD        0x03  // 'c'

union FloatUnion {
  float f;
  uint8_t b[4];
};

CanFrame rxFrame;

// ----- 시리얼 커맨드 콜백 함수 -----
// Commander를 통해 로컬 명령을 처리할 때 사용 (로컬 업데이트)
void doRefAngle(char* cmd) { command.scalar(&theta_desired, cmd); }
void doKp(char* cmd)       { command.scalar(&Kp, cmd); }
void doCd(char* cmd)       { command.scalar(&Cd, cmd); }

// ----- CAN 메시지 전송 함수 -----
// 대상 노드(target), 파라미터 ID, 값을 전송
void sendParam(uint8_t target, uint8_t paramID, float value) {
  CanFrame tx;
  tx.identifier = PARAM_CAN_ID;
  tx.extd = false;
  tx.data_length_code = 6;
  tx.data[0] = target;
  tx.data[1] = paramID;
  FloatUnion fu;
  fu.f = value;
  for (int i = 0; i < 4; i++) {
    tx.data[i + 2] = fu.b[i];
  }
  ESP32Can.writeFrame(tx);
}

// ----- 시리얼 커맨드 파서 -----
// "ID param value" 형식 (예: "2 r 10")
void processSerialCommandSimple() {
  if (Serial.available()) {
    String cmdLine = Serial.readStringUntil('\n');
    cmdLine.trim();
    if (cmdLine.length() > 0) {
      int target;
      char param;
      float value;
      if (sscanf(cmdLine.c_str(), "%d %c %f", &target, &param, &value) == 3) {
        uint8_t paramID;
        if (param == 'r' || param == 'R') {
          paramID = PARAM_ID_REF_ANGLE;
        } else if (param == 'k' || param == 'K') {
          paramID = PARAM_ID_KP;
        } else if (param == 'c' || param == 'C') {
          paramID = PARAM_ID_CD;
        } else {
          Serial.println("Unknown parameter");
          return;
        }
        if (target == NODE_ID) {
          // 로컬 업데이트
          if (paramID == PARAM_ID_REF_ANGLE) {
            theta_desired = value;
            Serial.print("Node ");
            Serial.print(NODE_ID);
            Serial.print(" updated theta_desired = ");
            Serial.println(theta_desired);
          } else if (paramID == PARAM_ID_KP) {
            Kp = value;
            Serial.print("Node ");
            Serial.print(NODE_ID);
            Serial.print(" updated Kp = ");
            Serial.println(Kp);
          } else if (paramID == PARAM_ID_CD) {
            Cd = value;
            Serial.print("Node ");
            Serial.print(NODE_ID);
            Serial.print(" updated Cd = ");
            Serial.println(Cd);
          }
        } else {
          // 대상이 다른 노드이면 CAN 메시지 전송
          sendParam((uint8_t)target, paramID, value);
          Serial.print("Sent command to node ");
          Serial.print(target);
          Serial.print(": ");
          Serial.print(param);
          Serial.print(" ");
          Serial.println(value);
        }
      } else {
        Serial.println("Invalid command. Use: <ID> <param> <value>");
      }
    }
  }
}

// ----- CAN 메시지 수신 처리 -----
// 수신된 CAN 메시지 중 대상이 자신의 NODE_ID와 일치하면 파라미터를 업데이트
void processCAN() {
  while (ESP32Can.readFrame(rxFrame, 0)) {
    if (rxFrame.identifier == PARAM_CAN_ID && rxFrame.data_length_code == 6) {
      uint8_t target = rxFrame.data[0];
      FloatUnion fu;
      for (int i = 0; i < 4; i++) {
        fu.b[i] = rxFrame.data[i + 2];
      }
      float newVal = fu.f;
      if (target == NODE_ID) {
        uint8_t pid = rxFrame.data[1];
        if (pid == PARAM_ID_REF_ANGLE) {
          theta_desired = newVal;
          Serial.print("Node ");
          Serial.print(NODE_ID);
          Serial.print(" received theta_desired = ");
          Serial.println(theta_desired);
        } else if (pid == PARAM_ID_KP) {
          Kp = newVal;
          Serial.print("Node ");
          Serial.print(NODE_ID);
          Serial.print(" received Kp = ");
          Serial.println(Kp);
        } else if (pid == PARAM_ID_CD) {
          Cd = newVal;
          Serial.print("Node ");
          Serial.print(NODE_ID);
          Serial.print(" received Cd = ");
          Serial.println(Cd);
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // Commander로 로컬 시리얼 커맨드 등록 (필요시)
  command.add('R', doRefAngle, "Set theta_desired [rad]");
  command.add('K', doKp, "Set Kp [A/rad]");
  command.add('C', doCd, "Set Cd [A/(rad/s)]");

  // ----- 센서 및 모터 초기화 -----
  encoder.init();
  motor.linkSensor(&encoder);

  // ----- DRV8302 핀 설정 -----
  pinMode(M_OC, OUTPUT); digitalWrite(M_OC, LOW);
  pinMode(M_PWM, OUTPUT); digitalWrite(M_PWM, HIGH);
  pinMode(OC_ADJ, OUTPUT); digitalWrite(OC_ADJ, HIGH);
  pinMode(OC_GAIN, OUTPUT); digitalWrite(OC_GAIN, LOW);

  // ----- 드라이버 초기화 및 연결 -----
  driver.voltage_power_supply = 30;
  driver.pwm_frequency = 15000;
  driver.init();
  motor.linkDriver(&driver);
  cs.linkDriver(&driver);
  motor.voltage_sensor_align = 0.5f;

  // ----- FOC 설정 -----
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;
  motor.motion_downsample = 0.0;
  motor.PID_current_q.P  = 0.05f;
  motor.PID_current_q.I  = 0.6f;
  motor.LPF_current_q.Tf = 0.04f;
  motor.PID_current_d.P  = 0.05f;
  motor.PID_current_d.I  = 0.6f;
  motor.LPF_current_d.Tf = 0.04f;
  motor.velocity_limit = 50.0f;
  motor.voltage_limit  = 20.0f;
  motor.current_limit  = 13.0f;
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D;
  motor.monitor_downsample = 100;
  
  motor.init();
  cs.init();
  cs.gain_a *= -1; cs.gain_b *= -1; cs.gain_c *= -1;
  motor.linkCurrentSense(&cs);
  
  motor.initFOC();

  delay(1000); // 모터 초기화 안정화 후
  
  // ----- TWAI CAN 초기화 -----
  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(500));
  if (ESP32Can.begin(ESP32Can.convertSpeed(500), CAN_TX, CAN_RX, 10, 10)) {
    Serial.println("CAN bus started");
  } else {
    Serial.println("CAN bus failed");
    while (true) {}
  }
}

void loop() {
  motor.loopFOC();

  float theta = motor.shaft_angle;
  float dtheta = motor.shaft_velocity;
  float Iq_ref = Kp * (theta_desired - theta) - Cd * dtheta;
  motor.target = Iq_ref;
  motor.move();

  processSerialCommandSimple();
  processCAN();
}
