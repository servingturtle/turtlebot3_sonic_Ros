#include "../../include/LED.h"


void setupRgbPins() {
  pinMode(RGB_R_PIN, OUTPUT);
  pinMode(RGB_G_PIN, OUTPUT);
  pinMode(RGB_B_PIN, OUTPUT);
  analogWrite(RGB_R_PIN, 0);
  analogWrite(RGB_G_PIN, 0);
  analogWrite(RGB_B_PIN, 0);
}

// m(미터) -> PWM(0~255): 가까울수록 밝게 (LED용 50cm 최대)
uint8_t distToPwm(float m) {
  if (isnan(m)) return 0;  // 미검출이면 꺼두기
  
  // LED용 거리 범위: 2cm ~ 50cm
  const float LED_MIN_M = 0.02f;
  const float LED_MAX_M = 0.50f;
  
  // 범위 클램프
  if (m < LED_MIN_M) m = LED_MIN_M;
  if (m > LED_MAX_M) m = LED_MAX_M;

  // 가까울수록 1.0, 멀수록 0.0 (50cm까지 세밀하게)
  float gain = (LED_MAX_M - m) / (LED_MAX_M - LED_MIN_M);
  int pwm = (int)(gain * 255.0f + 0.5f);
  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;
  return (uint8_t)pwm;
}

// RGB 출력 (센서: 0->R, 1->G, 2->B)
void updateRgbFromUltrasound() {
  uint8_t r = distToPwm(us[0].last_m);
  uint8_t g = distToPwm(us[1].last_m);
  uint8_t b = distToPwm(us[2].last_m);

  // 공통 애노드면 PWM 반전
  if (RGB_COMMON_ANODE) {
    r = 255 - r;
    g = 255 - g;
    b = 255 - b;
  }

  analogWrite(RGB_R_PIN, r);
  analogWrite(RGB_G_PIN, g);
  analogWrite(RGB_B_PIN, b);
}