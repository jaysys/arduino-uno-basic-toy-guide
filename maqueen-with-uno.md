말씀하신 Maqueen V3.0 보드 중앙 근처에 있는 포트에:

```
[ D ] [ C ] [ - ] [ + ]
```

이 표기가 되어 있고, 근처에 **"liC" 또는 "IIC"** 라고 적혀 있다면, **바로 그 포트가 I2C (IIC) 통신 포트**입니다.

---

### 🔍 해당 포트의 핀 의미

| 표기 | 의미 | 아두이노 우노 연결 |
| ---- | ---- | ------------------ |
| D    | SDA  | A4                 |
| C    | SCL  | A5                 |
| -    | GND  | GND                |
| +    | VCC  | 5V                 |

---

### 🔌 다시 정리: 아두이노 우노 연결 대응

| Maqueen 핀 | 연결 대상 (Arduino Uno) |
| ---------- | ----------------------- |
| D (SDA)    | A4                      |
| C (SCL)    | A5                      |
| - (GND)    | GND                     |
| + (VCC)    | 5V                      |

📌 즉, 해당 포트는 **Grove 포트 없이도 점퍼 케이블로 직접 연결할 수 있는 I2C 포트**입니다. 일반적인 2.54mm 핀 헤더이기 때문에 **Dupont 케이블**로 바로 연결하면 됩니다.

---

### 📸 참고용 (추가 확인 원할 시)

원하신다면 해당 위치에 핀 명칭을 표시한 **이미지로 확인**해드릴 수 있어요. 필요하시면 말씀해주세요 — 바로 시각적으로 안내해드리겠습니다.

---

✅ 결론:
**"D / C / - / +"로 표기된 그 포트가 Maqueen V3.0의 I2C 통신용 포트**가 맞습니다!

```cpp
// Maqueen V3.0의 내장 모터를 아두이노 우노로 제어
// I2C 통신을 통해 Maqueen의 모터 드라이버와 통신

#include <Wire.h>

// Maqueen I2C 주소 및 명령어
#define MAQUEEN_I2C_ADDRESS 0x10   // Maqueen의 I2C 주소

// 모터 제어 명령어 (Maqueen 프로토콜)
#define MOTOR_LEFT  0x00
#define MOTOR_RIGHT 0x02
#define MOTOR_STOP  0x00
#define MOTOR_FORWARD  0x01
#define MOTOR_BACKWARD 0x02

// 아두이노 우노와 Maqueen V3.0 연결:
// 아두이노 A4 (SDA) -> Maqueen SDA (I2C 데이터)
// 아두이노 A5 (SCL) -> Maqueen SCL (I2C 클럭)
// 아두이노 GND -> Maqueen GND
// 아두이노 5V -> Maqueen VCC (또는 별도 전원)

#define LED_BUILTIN 13

// 속도 설정 (0-255)
#define FAST_SPEED 200
#define MEDIUM_SPEED 150
#define SLOW_SPEED 100

void setup() {
 Serial.begin(9600);
 Wire.begin();  // I2C 마스터로 초기화

 pinMode(LED_BUILTIN, OUTPUT);

 // 시작 신호 - LED 3번 깜빡임
 for(int i = 0; i < 3; i++) {
   digitalWrite(LED_BUILTIN, HIGH);
   delay(200);
   digitalWrite(LED_BUILTIN, LOW);
   delay(200);
 }

 Serial.println("Maqueen V3.0 모터 제어 시작!");

 // 초기에 모터 정지
 stopMotors();
 delay(1000);
}

void loop() {
 // 전진
 Serial.println("전진 시작");
 moveForwardWithLED(MEDIUM_SPEED, 2000);

 // 정지
 Serial.println("정지");
 stopWithLED(1000);

 // 우회전 (제자리 회전)
 Serial.println("우회전");
 turnRightWithLED(SLOW_SPEED, 1000);

 // 정지
 stopWithLED(500);

 // 후진
 Serial.println("후진");
 moveBackwardWithLED(SLOW_SPEED, 1500);

 // 정지
 stopWithLED(500);

 // 좌회전 (제자리 회전)
 Serial.println("좌회전");
 turnLeftWithLED(SLOW_SPEED, 1000);

 // 정지
 stopWithLED(1000);
}

// I2C로 Maqueen 모터 제어
void controlMotor(uint8_t motor, uint8_t direction, uint8_t speed) {
 Wire.beginTransmission(MAQUEEN_I2C_ADDRESS);
 Wire.write(motor);      // 모터 선택 (왼쪽/오른쪽)
 Wire.write(direction);  // 방향
 Wire.write(speed);      // 속도 (0-255)
 uint8_t result = Wire.endTransmission();

 if(result != 0) {
   Serial.print("I2C 통신 오류: ");
   Serial.println(result);
 }
}

// 전진
void moveForward(uint8_t speed) {
 controlMotor(MOTOR_LEFT, MOTOR_FORWARD, speed);
 controlMotor(MOTOR_RIGHT, MOTOR_FORWARD, speed);
}

// 후진
void moveBackward(uint8_t speed) {
 controlMotor(MOTOR_LEFT, MOTOR_BACKWARD, speed);
 controlMotor(MOTOR_RIGHT, MOTOR_BACKWARD, speed);
}

// 좌회전 (제자리)
void turnLeft(uint8_t speed) {
 controlMotor(MOTOR_LEFT, MOTOR_BACKWARD, speed);
 controlMotor(MOTOR_RIGHT, MOTOR_FORWARD, speed);
}

// 우회전 (제자리)
void turnRight(uint8_t speed) {
 controlMotor(MOTOR_LEFT, MOTOR_FORWARD, speed);
 controlMotor(MOTOR_RIGHT, MOTOR_BACKWARD, speed);
}

// 모터 정지
void stopMotors() {
 controlMotor(MOTOR_LEFT, MOTOR_STOP, 0);
 controlMotor(MOTOR_RIGHT, MOTOR_STOP, 0);
}

// 좌측 곡선 주행
void curveLeft(uint8_t leftSpeed, uint8_t rightSpeed) {
 controlMotor(MOTOR_LEFT, MOTOR_FORWARD, leftSpeed);
 controlMotor(MOTOR_RIGHT, MOTOR_FORWARD, rightSpeed);
}

// 우측 곡선 주행
void curveRight(uint8_t leftSpeed, uint8_t rightSpeed) {
 controlMotor(MOTOR_LEFT, MOTOR_FORWARD, leftSpeed);
 controlMotor(MOTOR_RIGHT, MOTOR_FORWARD, rightSpeed);
}

// LED와 함께 전진
void moveForwardWithLED(uint8_t speed, int duration) {
 moveForward(speed);
 blinkLED(duration, 200);  // 200ms 간격으로 깜빡임
}

// LED와 함께 후진
void moveBackwardWithLED(uint8_t speed, int duration) {
 moveBackward(speed);
 blinkLED(duration, 150);  // 150ms 간격으로 빠르게 깜빡임
}

// LED와 함께 좌회전
void turnLeftWithLED(uint8_t speed, int duration) {
 turnLeft(speed);
 blinkLED(duration, 100);  // 100ms 간격으로 매우 빠르게 깜빡임
}

// LED와 함께 우회전
void turnRightWithLED(uint8_t speed, int duration) {
 turnRight(speed);
 blinkLED(duration, 100);  // 100ms 간격으로 매우 빠르게 깜빡임
}

// LED와 함께 정지
void stopWithLED(int duration) {
 stopMotors();
 blinkLED(duration, 500);  // 500ms 간격으로 천천히 깜빡임
}

// LED 깜빡임 함수 (논블로킹)
void blinkLED(int duration, int interval) {
 unsigned long startTime = millis();
 unsigned long lastBlink = 0;
 bool ledState = false;

 while(millis() - startTime < duration) {
   if(millis() - lastBlink >= interval) {
     ledState = !ledState;
     digitalWrite(LED_BUILTIN, ledState);
     lastBlink = millis();
   }
 }

 digitalWrite(LED_BUILTIN, LOW);  // LED 끄기
}

// I2C 장치 스캔 함수 (디버깅용)
void scanI2C() {
 Serial.println("I2C 장치 스캔 중...");
 byte error, address;
 int nDevices = 0;

 for(address = 1; address < 127; address++) {
   Wire.beginTransmission(address);
   error = Wire.endTransmission();

   if(error == 0) {
     Serial.print("I2C 장치 발견: 0x");
     if(address < 16) Serial.print("0");
     Serial.println(address, HEX);
     nDevices++;
   }
 }

 if(nDevices == 0) {
   Serial.println("I2C 장치를 찾을 수 없습니다.");
 } else {
   Serial.print("총 ");
   Serial.print(nDevices);
   Serial.println("개의 I2C 장치 발견");
 }
}
```

---

### 초음파 거리측정기 HC-SR04 추가해서 앞에 장애물 인지되면 멈추게 한다.

```cpp

#include <Wire.h>

// Maqueen I2C 관련 상수
#define MAQUEEN_I2C_ADDRESS 0x10
#define MOTOR_LEFT  0x00
#define MOTOR_RIGHT 0x02
#define MOTOR_STOP  0x00
#define MOTOR_FORWARD  0x02
#define MOTOR_BACKWARD 0x01

// 속도 설정
#define MEDIUM_SPEED 150

// LED
#define LED_BUILTIN 13

// HC-SR04 핀
#define TRIG_PIN 9
#define ECHO_PIN 10

// 거리 한계 (cm)
#define OBSTACLE_THRESHOLD 15

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // 시작 LED 깜빡임
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }

  Serial.println("Maqueen + HC-SR04 시작");
  stopMotors();
  delay(1000);
}

void loop() {
  long distance = measureDistance();
  Serial.print("거리: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > 0 && distance < OBSTACLE_THRESHOLD) {
    Serial.println("⚠️ 장애물 감지! 정지");
    stopWithLED(1000);
  } else {
    Serial.println("➡️ 전진 중...");
    moveForwardWithLED(MEDIUM_SPEED, 1000);
  }

  delay(200); // 측정 간 딜레이
}

// 초음파 거리 측정 (cm)
long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 20000); // 최대 20ms 대기
  if (duration == 0) return -1; // 측정 실패

  return duration * 0.034 / 2;
}

// Maqueen I2C 모터 제어
void controlMotor(uint8_t motor, uint8_t direction, uint8_t speed) {
  Wire.beginTransmission(MAQUEEN_I2C_ADDRESS);
  Wire.write(motor);
  Wire.write(direction);
  Wire.write(speed);
  uint8_t result = Wire.endTransmission();

  if (result != 0) {
    Serial.print("I2C 오류: ");
    Serial.println(result);
  }
}

void moveForward(uint8_t speed) {
  controlMotor(MOTOR_LEFT, MOTOR_FORWARD, speed);
  controlMotor(MOTOR_RIGHT, MOTOR_FORWARD, speed);
}

void stopMotors() {
  controlMotor(MOTOR_LEFT, MOTOR_STOP, 0);
  controlMotor(MOTOR_RIGHT, MOTOR_STOP, 0);
}

void moveForwardWithLED(uint8_t speed, int duration) {
  moveForward(speed);
  blinkLED(duration, 200);
}

void stopWithLED(int duration) {
  stopMotors();
  blinkLED(duration, 500);
}

void blinkLED(int duration, int interval) {
  unsigned long startTime = millis();
  unsigned long lastBlink = 0;
  bool ledState = false;

  while (millis() - startTime < duration) {
    if (millis() - lastBlink >= interval) {
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);
      lastBlink = millis();
    }
  }

  digitalWrite(LED_BUILTIN, LOW);
}


```
