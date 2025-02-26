/********************************************************
 * 1) 매크로 설정: 마스터 / 슬레이브 선택
 ********************************************************/
#define IS_MASTER  // <- 주석 해제 시, 이 보드는 마스터(Commander로 파라미터 입력 + CAN 전송)
//#define IS_MASTER  // 주석 처리 시, 슬레이브(직렬 Commander 없음, CAN 수신만)


/********************************************************
 * 2) 헤더 선언
 ********************************************************/
#include <ACAN_ESP32.h>      // CAN 통신 라이브러리
#include <SimpleFOC.h>       // 모터 FOC 라이브러리
#include <math.h>


/********************************************************
 * 3) CAN 통신 관련 상수
 ********************************************************/
// 원하는 CAN 속도 (예시: 500 kbps)
static const uint32_t DESIRED_BIT_RATE = 500UL * 1000UL;

// CAN 메시지 전송 주기를 제어할 수도 있지만
// 여기서는 "파라미터가 바뀔 때" 즉시 전송하는 방식을 시연

// LED 핀 (ESP32 일부 보드는 2번이 내장 LED)
static const int LED_PIN = 2;

/********************************************************
 * 4) SimpleFOC 모터, 드라이버, 센서 설정
 ********************************************************/
// drv8302 + esp-wroom-32
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

BLDCMotor motor(11);                            // Pole pairs=11 예시
BLDCDriver3PWM driver(INH_A, INH_B, INH_C, EN_GATE);
MagneticSensorI2C encoder(AS5600_I2C);
LowsideCurrentSense cs = LowsideCurrentSense(0.005f, 12.22f, IOUTA, IOUTB, IOUTC);

float theta_desired = 0.0f;  
float Kp = 2.0f; // max : 5.0
float Cd = 0.2f; // max : 0.2
float max_speed = 60.0f;
float Kt = 60 / (2 * PI * 380);


/********************************************************
 * 5) Commander: 직렬 명령 파싱 (마스터에서만 사용)
 ********************************************************/
#ifdef IS_MASTER
// #include <SimpleFOCCommander.h>
Commander command = Commander(Serial);

// FOC 모터에 대한 콜백
// void onMotor(char* cmd) { command.motor(&motor, cmd); }
void setVelocityP(char* cmd) { command.scalar(&motor.PID_velocity.P, cmd); }
void setVelocityI(char* cmd) { command.scalar(&motor.PID_velocity.I, cmd); }
#endif

/********************************************************
 * 6) 임피던스 제어용 파라미터
 ********************************************************/
// (필요하면 M_mass, target_position 등 추가 가능)

/********************************************************
 * 7) 전역 변수 / 자료형
 ********************************************************/
// float ↔ byte 변환용 union
union FloatUnion {
  float f;
  uint8_t b[4];
};

// 파라미터 식별자
static const uint8_t PARAM_ID_REF_ANGLE = 0x01;
static const uint8_t PARAM_ID_KSPRING   = 0x02;
static const uint8_t PARAM_ID_CDAMPER   = 0x03;

// CAN ID (파라미터 브로드캐스트용)
static const uint32_t PARAM_CAN_ID = 0x100;

/********************************************************
 * 8) 파라미터 변경 시 CAN 전송 함수 (마스터 전용)
 ********************************************************/
#ifdef IS_MASTER
void sendParamOverCAN(uint8_t paramID, float value) {
  CANMessage txFrame;
  txFrame.id = PARAM_CAN_ID;
  txFrame.len = 5; // [1바이트 paramID + 4바이트 float]

  txFrame.data[0] = paramID; // 어떤 파라미터인지 구분

  FloatUnion fu;
  fu.f = value;
  for (int i = 0; i < 4; i++) {
    txFrame.data[i+1] = fu.b[i]; // ???????????? 갑자기 fu.b는 왜 나온 거지? garbage 값이어야하는데.
    //                               --> 이에 대한 답변 : union은 공용체 멤버들이 동일한 메모리 영역을 공유하기 때문이다.
    //fu.f = value; -> union의 4바이트 메모리 영역이 value의 부동소수점 비트로 채워짐. 
    //그리고 fu.b[i]가 같은 4바이트 구역을 바이트 배열로 해석해 읽어옴.
    //이렇게 사용하는 것 방법은 C언어에서 부동소수점 값을 바이트로 분해하는 전형적인 기법이며, CAN메시지나 시리얼 패킷에
    //float를 담기 위해 이 방식을 사용할 수 있다.
  }

  if (ACAN_ESP32::can.tryToSend(txFrame)) {
    Serial.print("[CAN] Param sent. ID=");
    Serial.print(paramID, HEX); //??? what is HEX
    Serial.print(", value=");
    Serial.println(value);
  } else {
    Serial.println("[CAN] Send error!");
  }

  //paramID를 가진 값이 Commander를 통해 바뀔 때, 즉시 CAN으로 브로드 캐스트 하는 방식이다.
}
#endif

/********************************************************
 * 9) Commander 콜백: ref_angle, K_spring, C_damper
 ********************************************************/
#ifdef IS_MASTER
void doRefAngle(char* cmd) {
  command.scalar(&theta_desired, cmd); // 기존 Commander 방식
  // 변경됐으니 CAN 전송
  sendParamOverCAN(PARAM_ID_REF_ANGLE, theta_desired);
}
void doKp(char* cmd) {
  command.scalar(&Kp, cmd);
  sendParamOverCAN(PARAM_ID_KSPRING, Kp);
}
void doCd(char* cmd) {
  command.scalar(&Cd, cmd);
  sendParamOverCAN(PARAM_ID_CDAMPER, Cd);
}
#endif

/********************************************************
 * 10) setup()
 ********************************************************/
void setup() {
  // 시리얼 초기화 (Commander or 디버그)
  Serial.begin(115200);
  delay(100);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // --- CAN 설정 ---
  Serial.println("Configure ESP32 CAN in Normal mode");
  {
    ACAN_ESP32_Settings settings(DESIRED_BIT_RATE);
    settings.mRequestedCANMode = ACAN_ESP32_Settings::NormalMode;

    // 예시 TX=17, RX=16
    settings.mTxPin = (gpio_num_t)17;
    settings.mRxPin = (gpio_num_t)16;

    // 필터 acceptAll
    const ACAN_ESP32_Filter filter = ACAN_ESP32_Filter::acceptAll();
    const uint32_t errorCode = ACAN_ESP32::can.begin(settings, filter);
    if (errorCode == 0) {
      Serial.println("CAN configuration OK!");
    } else {
      Serial.print("CAN configuration error: 0x");
      Serial.println(errorCode, HEX);
      while(true){}
    }
  }

  // --- SimpleFOC 설정 ---
  // 1) 센서 초기화
  encoder.init();
  motor.linkSensor(&encoder);

  // 2) 드라이버 세팅
  pinMode(M_OC, OUTPUT);
  digitalWrite(M_OC, LOW);
  pinMode(M_PWM, OUTPUT);
  digitalWrite(M_PWM, HIGH);
  pinMode(OC_ADJ, OUTPUT);
  digitalWrite(OC_ADJ, HIGH);
  pinMode(OC_GAIN, OUTPUT);
  digitalWrite(OC_GAIN, LOW); // LOW 상태에서 특정 게인 (약 10~12배)

  driver.voltage_power_supply = 30;
  driver.pwm_frequency = 15000;       // PWM 주파수 [Hz]
  driver.init();
  motor.linkDriver(&driver);
  cs.linkDriver(&driver);
  motor.voltage_sensor_align = 0.5f; // [V]


  // 3) FOC 설정
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::torque;  
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
#ifdef IS_MASTER
  command.add('R', doRefAngle, "ref angle [rad]");
  command.add('K', doKp, "Kp [A/rad]");
  command.add('C', doCd, "Cd [A/rad]");

  Serial.println(F("FOC Current-based Torque Control (Position-based)"));
  Serial.println(F(" - R: set target angle [rad]"));
  Serial.println(F(" - K: set Kp [A/rad] (proportional gain)"));
  Serial.println(F(" motor.target will be I_q [A] in torque mode.\n"));
  Serial.println("Commander (MASTER) ready.");

#else
  Serial.println("SLAVE mode: no Commander.");
#endif

  _delay(1000);

  // --- Commander 등록 (마스터만) ---
  Serial.println("Setup complete!");
}

/********************************************************
 * 11) loop()
 ********************************************************/
void loop() {
  // (A) FOC 루프
  motor.loopFOC();

  // 간단 임피던스 식: τ = -K(θ - ref) - C·dθ
  float theta  = motor.shaft_angle;
  float dtheta = motor.shaft_velocity;
  float Iq_ref = Kp * (theta_desired - theta) -Cd * dtheta;
  float output_torque = 10 * Kt * motor.current.q;

  motor.target = Iq_ref; // [A] foc_current제어이기 때문에 target값이 q축 전류이다.

  // ========== 5) FOC 업데이트 (PWM 출력) ==========
  motor.move();

  // ========== 6) 모니터링 ==========
  // motor.monitor();

  // (B) Commander (마스터만)
#ifdef IS_MASTER
  command.run();
#endif

  // (C) CAN 수신 (마스터/슬레이브 모두)
  {
    CANMessage rxFrame;
    while (ACAN_ESP32::can.receive(rxFrame)) {
      // Param update message?
      if ((rxFrame.id == PARAM_CAN_ID) && (rxFrame.len == 5)) {
        uint8_t paramID = rxFrame.data[0];
        FloatUnion fu;
        fu.b[0] = rxFrame.data[1];
        fu.b[1] = rxFrame.data[2];
        fu.b[2] = rxFrame.data[3];
        fu.b[3] = rxFrame.data[4];

        float newVal = fu.f; 
        // 4byte 배열로 저장했던 것을 다시 float로 복원하는 과정

        switch (paramID) {
          case PARAM_ID_REF_ANGLE:
            theta_desired = newVal;
            Serial.print("Updated theta_desired via CAN: ");
            Serial.println(theta_desired);
            break;

          case PARAM_ID_KSPRING:
            Kp = newVal;
            Serial.print("Updated K_spring via CAN: ");
            Serial.println(Kp);
            break;

          case PARAM_ID_CDAMPER:
            Cd = newVal;
            Serial.print("Updated C_damper via CAN: ");
            Serial.println(Cd);
            break;

          default:
            break;
        }
      } else {
        // 다른 메시지이면 로그만 찍거나 무시
        Serial.print("Received other CAN frame: ID=0x");
        Serial.print(rxFrame.id, HEX);
        Serial.print(", DLC=");
        Serial.print(rxFrame.len);
        Serial.print(", Data=[");
        for (uint8_t i=0; i<rxFrame.len; i++){
          Serial.print(rxFrame.data[i], HEX);
          if(i<rxFrame.len-1) Serial.print(" ");
        }
        Serial.println("]");
      }
    }
  }
}
