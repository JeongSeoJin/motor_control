#include <SimpleFOC.h>
#include <math.h>

// 1) 센서 설정: AS5600 I2C
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// 2) BLDC 모터 & 드라이버 설정
//    - 모터: BLDCMotor(pole_pairs) -> 5010 모터라면 극쌍수(pole_pairs) 7 등
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// ========== 임피던스 제어 파라미터 ==========
float K_spring = 3.0;       // 스프링 상수
float C_damper = 0.1;       // 감쇠 계수
float M_mass   = 0.0;      // 가상 질량
float ref_angle = 0.0;      // 평형 각도 (rad)

// 가속도(각가속도) 추정용
static float last_velocity = 0.0f;
static float ddtheta_est   = 0.0f;

// LPF 계수 (노이즈 억제용)
static float alpha = 0.9f;  
// 예: 0.9 -> 새 데이터 10%만 반영, 90%는 이전값 유지 (필요에 따라 조정)

// 루프 주기(초)
// 실제론 micros() 등을 사용해 매 회 루프 사이의 dt를 구하는 것이 좋음
// 여기서는 예시로 고정 1ms(=0.001초)라 가정
const float dt = 0.001f;  

// Commander 인스턴스
Commander command = Commander(Serial);

// 커맨드 파서 함수들
void doRefAngle(char* cmd) { command.scalar(&ref_angle, cmd); }
void doKspring(char* cmd)  { command.scalar(&K_spring, cmd); }
void doCdamper(char* cmd)  { command.scalar(&C_damper, cmd); }
void doMass(char* cmd)     { command.scalar(&M_mass, cmd); }

void setup() {
  Serial.begin(115200);
  while(!Serial);

  // 센서 초기화
  sensor.init();
  // 모터에 센서 연결
  motor.linkSensor(&sensor);

  // 드라이버 초기화
  driver.voltage_power_supply = 12;  // 전원 전압 (예: 12V)
  driver.init();
  motor.linkDriver(&driver);

  // FOC 모듈레이션
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // *** 토크 제어 모드로 설정 ***
  motor.controller = MotionControlType::torque;

  // 전압 제한 (토크 제어 시 전압 제한이 곧 토크 제한에 영향)
  motor.voltage_limit = 6; // 예시값 (필요 시 조정)

  // PID 설정 (내부 속도 루프/전류 루프용) 
  motor.PID_velocity.P = 0.1f;
  motor.PID_velocity.I = 1.7f;
  motor.PID_velocity.D = 0;
  motor.LPF_velocity.Tf = 0.01;

  // 속도 제한 (rad/s) - 토크 모드에서는 직접적 제한 효과 없음
  motor.velocity_limit = 10;

  // 모니터링 (Serial Plotter)
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_VEL | _MON_ANGLE | _MON_VOLT_Q;
  motor.monitor_downsample = 10;
  
  // 모터 초기화 + 센서 얼라인
  motor.init();
  motor.initFOC();

  // Commander 명령 추가
  command.add('R', doRefAngle,  "Reference angle [rad]");
  command.add('K', doKspring,   "K_spring");
  command.add('C', doCdamper,   "C_damper");
  command.add('M', doMass,      "M_mass");

  Serial.println("Impedance Control (Spring+Damper+Mass) Ready.");
  Serial.println("Commands:");
  Serial.println("  R <val> : set reference angle (rad)");
  Serial.println("  K <val> : set spring constant");
  Serial.println("  C <val> : set damping constant");
  Serial.println("  M <val> : set virtual mass");
  _delay(1000);
}

void loop() {
  // 1) FOC 실행 (전류 루프 등 내부 제어)
  motor.loopFOC();

  // ========== 2) 각가속도 추정 (유한차분 + LPF) ==========
  float dtheta = motor.shaft_velocity;  
  float raw_acc = (dtheta - last_velocity) / dt;
  // LPF 적용
  ddtheta_est = alpha * ddtheta_est + (1.0f - alpha) * raw_acc;
  last_velocity = dtheta;

  // ========== 3) 임피던스 제어 식 (토크 계산) ==========
  float theta = motor.shaft_angle;

  // 스프링(K) + 댐퍼(C) + 질량(M) 항
  //   \tau = -K(θ - θ0) - C·dθ - M·ddθ
  float torque_cmd = 
       -K_spring * (theta - ref_angle)
       -C_damper * dtheta
       -M_mass   * ddtheta_est;

  // 4) 토크 명령 설정 (토크 제어 모드)
  motor.move(torque_cmd);

  // 5) 모니터링 & 시리얼 명령 파싱
  motor.monitor();
  command.run();
}
