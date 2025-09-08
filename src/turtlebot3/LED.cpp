#include "../../include/LED.h"

// LED 반짝임 방지를 위한 변수들
static uint8_t current_r = 0, current_g = 0, current_b = 0;
static uint8_t target_r = 0, target_g = 0, target_b = 0;
static unsigned long last_update_time = 0;
static const unsigned long UPDATE_INTERVAL = 100; // 100ms마다 업데이트
static const uint8_t FADE_STEP = 5; // 한 번에 변경할 PWM 값
static bool led_initialized = false;

void setupRgbPins() {
  pinMode(RGB_R_PIN, OUTPUT);
  pinMode(RGB_G_PIN, OUTPUT);
  pinMode(RGB_B_PIN, OUTPUT);
  analogWrite(RGB_R_PIN, 0);
  analogWrite(RGB_G_PIN, 0);
  analogWrite(RGB_B_PIN, 0);
  
  // 초기값 설정
  current_r = current_g = current_b = 0;
  target_r = target_g = target_b = 0;
  led_initialized = true;
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

// 부드러운 LED 전환 함수
void fadeToTarget() {
  if (!led_initialized) return;
  
  // Red 채널 부드러운 전환
  if (current_r < target_r) {
    current_r = min(current_r + FADE_STEP, target_r);
  } else if (current_r > target_r) {
    current_r = max(current_r - FADE_STEP, target_r);
  }
  
  // Green 채널 부드러운 전환
  if (current_g < target_g) {
    current_g = min(current_g + FADE_STEP, target_g);
  } else if (current_g > target_g) {
    current_g = max(current_g - FADE_STEP, target_g);
  }
  
  // Blue 채널 부드러운 전환
  if (current_b < target_b) {
    current_b = min(current_b + FADE_STEP, target_b);
  } else if (current_b > target_b) {
    current_b = max(current_b - FADE_STEP, target_b);
  }
  
  // 공통 애노드면 PWM 반전
  uint8_t r_out = current_r, g_out = current_g, b_out = current_b;
  if (RGB_COMMON_ANODE) {
    r_out = 255 - r_out;
    g_out = 255 - g_out;
    b_out = 255 - b_out;
  }
  
  analogWrite(RGB_R_PIN, r_out);
  analogWrite(RGB_G_PIN, g_out);
  analogWrite(RGB_B_PIN, b_out);
}

// RGB 출력 (센서: 0->R, 1->G, 2->B)
void updateRgbFromUltrasound() {
  if (!led_initialized) return;
  
  unsigned long current_time = millis();
  
  // 업데이트 주기 제한
  if (current_time - last_update_time < UPDATE_INTERVAL) {
    // 주기 제한 중에는 부드러운 전환만 수행
    fadeToTarget();
    return;
  }
  
  last_update_time = current_time;
  
  // 센서 값이 유효한지 확인하고 안전한 기본값 사용
  float left_dist = (us[0].last_m > 0 && !isnan(us[0].last_m)) ? us[0].last_m : 1.0f;
  float front_dist = (us[1].last_m > 0 && !isnan(us[1].last_m)) ? us[1].last_m : 1.0f;
  float right_dist = (us[2].last_m > 0 && !isnan(us[2].last_m)) ? us[2].last_m : 1.0f;
  
  // 새로운 목표값 계산
  target_r = distToPwm(left_dist);
  target_g = distToPwm(front_dist);
  target_b = distToPwm(right_dist);
  
  // 부드러운 전환 수행
  fadeToTarget();
}