/**
 * 위치 기반 토크 제어 (전류제어 FOC) 예시 코드
 * 
 * [개념]
 *  - 모터의 현재 각도(shaft_angle, [rad])와 목표 각도(theta_desired, [rad])를 비교해
 *    오차(angleError)를 구한다.
 *  - 오차 * Kp ([A/rad]) = 토크 지령 전류(I_q_ref, [A])를 계산한다.
 *  - SimpleFOC는 FOC Current 모드(TorqueControlType::foc_current)에서 이 전류를
 *    실제로 모터의 q축 전류(I_q)로 만들기 위해 PWM을 제어한다.
 * 
 *  => motor.target = I_q_ref (단위: [A])
 *  => 실제 토크 T [N·m] = Kt [N·m/A] × I_q [A]
 */

// This source code is for a proprioceptive actuator v2. torque sensing using q-axis current sensing

#include <SimpleFOC.h>  // SimpleFOC 라이브러리
#include <math.h>

// ============================ 핀 매핑 설정 ============================
// DRV8302 보드 핀 연결 (기본 예시)
#define INH_A    14     // 하이사이드 PWM A
#define INH_B    27     // 하이사이드 PWM B
#define INH_C    26     // 하이사이드 PWM C
#define EN_GATE  13     // 드라이버 활성화
#define M_PWM    33     // 3pwm 모드 설정
#define M_OC     25     // 오버커런트 보호
#define OC_ADJ   32     // 오버커런트 조정
#define OC_GAIN  5      // DRV8302 내부 증폭기 게인 선택 핀

// 로사이드 전류 센싱 핀 (ESP32 ADC 핀)
#define IOUTA 4         // ADC2_CH0
#define IOUTB 12        // ADC2_CH5
#define IOUTC 15        // ADC2_CH3

// ============================= 모터, 드라이버, 센서 =============================

// 1) 모터 객체
//  - BLDCMotor( pole_pairs ): pole_pairs = 극수(pole) / 2
//  - 예: 22극 모터 -> pole_pairs=11
BLDCMotor motor = BLDCMotor(11);

// 2) 드라이버 객체
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C, EN_GATE);

// 3) 로사이드 전류센싱
//  - 샨트저항: 0.005Ω
//  - 증폭기 게인: 12.22 V/V (DRV8302 내부 증폭 + 외부 회로에 따라 측정)
LowsideCurrentSense cs = LowsideCurrentSense(0.005f, 12.22f, IOUTA, IOUTB, IOUTC);

// 4) 엔코더 (AS5600 I2C 예시)
MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);

// 5) 시리얼 커맨드 인터페이스
Commander command = Commander(Serial);
// void onMotor(char* cmd) { command.motor(&motor, cmd); }

// ============================= 제어용 전역 변수 =============================
// 목표 각도 [rad]
float theta_desired = 0.0f;  
// 위치 오차 → 전류로 바꿀 때 사용하는 비례이득 Kp [A/rad]
// 예) Kp=1.0 -> 오차 1rad당 q축전류 1A
float Kp = 2.0f; // max : 5.0
float Cd = 0.2f; // max : 0.2
float max_speed = 60.0f;

float Kt = 60 / (2 * PI * 380);

// Commander에서 theta_desired, Kp를 설정하기 위한 함수
void doRefAngle(char* cmd) { command.scalar(&theta_desired, cmd); }
void doKp(char* cmd)       { command.scalar(&Kp, cmd); }
void doCd(char* cmd)       { command.scalar(&Cd, cmd); }

void setup() {
  // ========== 시리얼 초기화 ==========
  Serial.begin(115200);
  // SimpleFOC 디버그 활성화(옵션)
  SimpleFOCDebug::enable(&Serial);

  // ========== 센서 초기화 ==========
  encoder.init();           // I2C 기반 AS5600 초기화
  motor.linkSensor(&encoder);

  // ========== DRV8302 관련 핀 설정 ==========
  pinMode(M_OC, OUTPUT);
  digitalWrite(M_OC, LOW);
  pinMode(M_PWM, OUTPUT);
  digitalWrite(M_PWM, HIGH);
  pinMode(OC_ADJ, OUTPUT);
  digitalWrite(OC_ADJ, HIGH);
  pinMode(OC_GAIN, OUTPUT);
  digitalWrite(OC_GAIN, LOW); // LOW 상태에서 특정 게인 (약 10~12배)

  // ========== 드라이버 설정 ==========
  driver.voltage_power_supply = 30;   // 드라이버 공급전압 [V]
  driver.pwm_frequency = 15000;       // PWM 주파수 [Hz]
  driver.init();
  motor.linkDriver(&driver);

  // ========== 전류센서 설정 ==========
  cs.linkDriver(&driver);
  
  // ========== 모터 기본 파라미터 ==========
  // FOC 정렬 시 사용하는 전압(각도 캘리브레이션)
  motor.voltage_sensor_align = 0.5f; // [V]
  
  // ========== 핵심: 전류제어(FOC) 설정 ==========
  // 1) 토크제어 모드: foc_current => q축 전류를 직접 제어함
  motor.torque_controller = TorqueControlType::foc_current;
  
  // 2) MotionControlType::torque => motor.target = q축전류(A)
  motor.controller = MotionControlType::torque;
  
  // 3) motion_downsample: 속도/각도 루프를 생략하거나 간소화 (여기서는 0.0)
  motor.motion_downsample = 0.0;

  // ========== PID / LPF 설정 (주로 내부 PI 전류 루프) ==========
  // 전류 i_q 제어
  motor.PID_current_q.P  = 0.05f;   // [V/A] (내부 환산 게인)
  motor.PID_current_q.I  = 0.6f; // [V/(A·s)]
  motor.LPF_current_q.Tf = 0.04f;  // [s]

  // 전류 i_d 제어
  motor.PID_current_d.P  = 0.05f;   // [V/A]
  motor.PID_current_d.I  = 0.6f; // [V/(A·s)]
  motor.LPF_current_d.Tf = 0.04f;  // [s]

  // 속도/각도 제어(지금 안씀)도 일단 기본값 설정
  motor.PID_velocity.P  = 0.1f; 
  motor.PID_velocity.I  = 1.7f;  
  motor.LPF_velocity.Tf = 0.001f;
  motor.P_angle.P       = 10.0f;  
  motor.LPF_angle.Tf    = 0.0f;

  // ========== 제한값 ==========
  // 최대 속도 [rad/s] (토크모드이지만 안전 차원)
  motor.velocity_limit = 50.0f; 
  // FOC에서 인가할 전압의 최대치 [V]
  motor.voltage_limit  = 20.0f;  
  // 최대 전류 제한 [A]
  // -> motor.target이 이 값을 초과 못하도록 내부 보호
  motor.current_limit  = 13.0f;   

  // ========== 모니터링 설정 ==========
  motor.useMonitoring(Serial);
  // D축, Q축 전류 모니터링
  motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D; // q가 토크에 관여, D는 0에 가까워져 있어야함. 
  // 출력 주기 : loopFOC()가 n번 실행될 때마다 한번씩 출력
  motor.monitor_downsample = 100;

  // ========== 모터 초기화 ==========
  motor.init();   // 드라이버, 센서 설정에 맞춰 모터구조 초기 설정
  cs.init();      // 전류센싱 초기화
  // DRV8302는 내부적으로 게인이 반전돼있는 경우가 있어 -1 곱
  cs.gain_a *= -1;
  cs.gain_b *= -1;
  cs.gain_c *= -1;
  motor.linkCurrentSense(&cs);

  // ========== FOC align & start ==========
  // 실제 모터에 전압을 인가하여 엔코더 0점 찾고, d/q 축 정렬
  motor.initFOC();

  // ========== Commander (시리얼 명령) ==========
  // command.add('M', onMotor, "motor");
  command.add('R', doRefAngle, "ref angle [rad]");
  command.add('K', doKp, "Kp [A/rad]");
  command.add('C', doCd, "Cd [A/rad]");

  Serial.println(F("FOC Current-based Torque Control (Position-based)"));
  Serial.println(F(" - R: set target angle [rad]"));
  Serial.println(F(" - K: set Kp [A/rad] (proportional gain)"));
  Serial.println(F(" motor.target will be I_q [A] in torque mode.\n"));

  _delay(1000);
}

void loop() {
  // ========== 1) FOC 알고리즘 업데이트 ==========
  //   - 센서 위치 갱신, d/q 전류 측정 및 PID 계산 등
  motor.loopFOC();

  // ========== 2) 위치 오차 계산 ==========
  //   - 엔코더 각도 [rad]
  float theta = motor.shaft_angle;
  float dtheta = motor.shaft_velocity;
  //   - theta_desired [rad], currentAngle [rad]
  // float angleError = theta_desired - currentAngle;  // [rad]

  // ========== 3) 토크 전류 지령 (Iq_ref) 계산 ==========
  //   - Kp [A/rad] * angleError [rad] = Iq_ref [A]
  float Iq_ref = Kp * (theta_desired - theta) -Cd * dtheta;

  // ========== 4) 모터에 q축 전류 지령 전송 ==========
  //   - torque 모드이므로 motor.target = Iq_ref
  //   - motor.target 단위: [A]

  float output_torque = 10 * Kt * motor.current.q;
  // Serial.println(output_torque);
  // Serial.println(motor.current.q); // 실제 q축 전류

  motor.target = Iq_ref; // [A] foc_current제어이기 때문에 target값이 q축 전류이다.

  // ========== 5) FOC 업데이트 (PWM 출력) ==========
  motor.move();

  // ========== 6) 모니터링 ==========
  motor.monitor();

  // ========== 7) 시리얼 명령 처리 ==========
  command.run();
}
