Arduino Uno에 **CNT1 3색 RGB LED 모듈**을 연결하는 방법과 샘플 코드

---

## 🔌 CNT1 3색 LED (RGB LED 모듈) 핀 구성

CNT1 모듈은 **공통 음극(common cathode)** 방식의 RGB LED입니다.

| CNT1 핀 번호 | 기능        | 연결 대상            |
| ------------ | ----------- | -------------------- |
| 1 (GND)      | 공통 GND    | Arduino GND          |
| 2 (R)        | 빨간색 제어 | Arduino D9 (PWM 핀)  |
| 3 (G)        | 초록색 제어 | Arduino D10 (PWM 핀) |
| 4 (B)        | 파란색 제어 | Arduino D11 (PWM 핀) |

> 일반적으로 **R, G, B 핀은 모두 저항을 통해 연결**하는 것이 안전합니다. (220Ω\~330Ω)

---

## ✅ 샘플 코드: RGB LED로 다양한 색 만들기

```cpp
// 핀 정의
const int redPin = 9;
const int greenPin = 10;
const int bluePin = 11;

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

void loop() {
  // 빨간색
  setColor(255, 0, 0);
  delay(1000);

  // 초록색
  setColor(0, 255, 0);
  delay(1000);

  // 파란색
  setColor(0, 0, 255);
  delay(1000);

  // 노란색
  setColor(255, 255, 0);
  delay(1000);

  // 하늘색
  setColor(0, 255, 255);
  delay(1000);

  // 보라색
  setColor(255, 0, 255);
  delay(1000);

  // 흰색 (모든 색 최대)
  setColor(255, 255, 255);
  delay(1000);

  // LED 끄기
  setColor(0, 0, 0);
  delay(1000);
}

// 색상 설정 함수 (0~255)
void setColor(int red, int green, int blue) {
  analogWrite(redPin, 255 - red);
  analogWrite(greenPin, 255 - green);
  analogWrite(bluePin, 255 - blue);
}
```

> CNT1은 공통 GND 방식이므로 **PWM 값은 반대로 적용**됩니다.
> 즉, `analogWrite(pin, 0)` = 최대 밝기, `analogWrite(pin, 255)` = 꺼짐
> (`255 - 값` 으로 변환해서 직관적으로 다룰 수 있습니다.)

---

## 📝 요약

| 색상 조합 | R   | G   | B   |
| --------- | --- | --- | --- |
| 빨강      | 255 | 0   | 0   |
| 초록      | 0   | 255 | 0   |
| 파랑      | 0   | 0   | 255 |
| 노랑      | 255 | 255 | 0   |
| 하늘      | 0   | 255 | 255 |
| 보라      | 255 | 0   | 255 |
| 흰색      | 255 | 255 | 255 |
