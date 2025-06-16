초음파 거리 센서 **HC-SR04** (HR-sR04라고 부르신 것은 HC-SR04의 오타로 보입니다)의 **핀 설정과 샘플 코드**

---

## 🔌 HC-SR04 핀 구성

| HC-SR04 핀 | 기능               | Arduino 연결 |
| ---------- | ------------------ | ------------ |
| VCC        | 전원 입력 (+5V)    | 5V           |
| GND        | 그라운드           | GND          |
| TRIG       | 초음파 발사 (출력) | D9           |
| ECHO       | 초음파 수신 (입력) | D10          |

> 전원은 반드시 5V에 연결하세요. 3.3V는 감지 거리나 작동에 문제가 생길 수 있어요.

---

## ✅ 샘플 코드: HC-SR04로 거리 측정하기

```cpp
const int trigPin = 9;   // 초음파 발사 핀
const int echoPin = 10;  // 초음파 수신 핀

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // 초음파 발사
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 초음파 수신 시간 측정 (μs)
  long duration = pulseIn(echoPin, HIGH);

  // 거리 계산 (단위: cm)
  long distance = duration * 0.034 / 2;

  // 출력
  Serial.print("거리: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(500);  // 0.5초 대기
}
```

---

## 📏 거리 계산 공식

- 초음파의 속도는 약 **340m/s = 0.034 cm/μs**
- 왕복 시간이므로 거리 계산 시 **2로 나눔**

```
거리(cm) = (duration * 0.034) / 2
```

---

## ✅ 요약 연결

| HC-SR04 | Arduino Uno |
| ------- | ----------- |
| VCC     | 5V          |
| GND     | GND         |
| TRIG    | D9          |
| ECHO    | D10         |

---
