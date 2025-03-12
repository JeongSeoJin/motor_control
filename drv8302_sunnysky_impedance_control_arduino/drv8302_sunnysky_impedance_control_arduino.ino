#include <SimpleFOC.h>

// DRV8302 pins connections
// don't forget to connect the common ground pin
#define   INH_A 9
#define   INH_B 10
#define   INH_C 11
#define   EN_GATE 8
#define   M_PWM 6 
#define   M_OC 5
#define   OC_ADJ 7

float target_position = 0;

// ========== 임피던스 제어 파라미터 ==========
float K_spring = 1.0;       // 스프링 상수
float C_damper = 0.0;       // 감쇠 계수
float M_mass   = 0.0;      // 가상 질량
float ref_angle = 0.0;      // 평형 각도 (rad)

// motor instance
BLDCMotor motor = BLDCMotor(11);

// driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C, EN_GATE);

// encoder instance
// Encoder encoder = Encoder(2, 3, 8192);
MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);

// commander interface
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }
// 속도 PID P 값 설정 함수
void setVelocityP(char* cmd) { command.scalar(&motor.PID_velocity.P, cmd); }
// 속도 PID I 값 설정 함수
void setVelocityI(char* cmd) { command.scalar(&motor.PID_velocity.I, cmd); }

void pos(char* cmd) { command.scalar(&target_position, cmd); }

void doRefAngle(char* cmd) { command.scalar(&ref_angle, cmd); }
void doKspring(char* cmd)  { command.scalar(&K_spring, cmd); }
void doCdamper(char* cmd)  { command.scalar(&C_damper, cmd); }
void doMass(char* cmd)     { command.scalar(&M_mass, cmd); }

void setup() {

  // initialize encoder sensor hardware
  encoder.init();

  // encoder.direction = Direction::CW;  // 시계 방향으로 설정

  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // DRV8302 specific code
  // M_OC  - enable over-current protection
  pinMode(M_OC,OUTPUT);
  digitalWrite(M_OC,LOW);
  // M_PWM  - enable 3pwm mode
  pinMode(M_PWM,OUTPUT);
  digitalWrite(M_PWM,HIGH);
  // OD_ADJ - set the maximum over-current limit possible
  // Better option would be to use voltage divisor to set exact value
  pinMode(OC_ADJ,OUTPUT);
  digitalWrite(OC_ADJ,HIGH);

  // configure driver
  driver.voltage_power_supply = 24;
  driver.init();
  motor.linkDriver(&driver);

  // choose FOC modulation
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set control loop type to be used
  motor.controller = MotionControlType::torque;

  // controller configuration based on the control type 
  motor.PID_velocity.P = 0.01;
  motor.PID_velocity.I = 1.7;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.001;

  // angle loop controller
  motor.P_angle.P = 10;
  
  // angle loop velocity limit
  motor.velocity_limit = 20; // 진짜 max값.
  // default voltage_power_supply
  motor.voltage_limit = 6;

  // use monitoring with serial for motor init
  // monitoring port
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // set the initial target value
  motor.target = 0;

  // define the motor id
  command.add('M', onMotor, "motor");
  command.add('V', setVelocityP, "Velocity PID P Gain");
  command.add('I', setVelocityI, "Velocity PID I Gain");
  command.add('R', doRefAngle,  "Reference angle [rad]");
  command.add('K', doKspring,   "K_spring");
  command.add('C', doCdamper,   "C_damper");

  _delay(1000);
}


void loop() {
  // iterative setting FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outer loop target
  // velocity, position or voltage
  // if target not set in parameter uses motor.target variable
  // motor.move(target_position);
  // Serial.println()

  // ========== 3) 임피던스 제어 식 (토크 계산) ==========
  float dtheta = motor.shaft_velocity;
  float theta = motor.shaft_angle;

  // 스프링(K) + 댐퍼(C) + 질량(M) 항
  //   \tau = -K(θ - θ0) - C·dθ - M·ddθ
  float torque_cmd = 
       -K_spring * (theta - ref_angle)
       -C_damper * dtheta;

  // 4) 토크 명령 설정 (토크 제어 모드)
  motor.move(torque_cmd);


  // user communication
  command.run();
}