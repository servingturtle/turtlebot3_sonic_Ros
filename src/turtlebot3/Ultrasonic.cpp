#include <Arduino.h>
#include <math.h>
#include "../../include/Ultrasonic.h"

// ===== 핀/측정 설정 "정의"(헤더 extern과 일치, static 금지) =====
const uint8_t  US_TRIG[3]  = {5, 6, 7};
const uint8_t  US_ECHO[3]  = {2, 3, 4};  // EXTI_0, EXTI_1, EXTI_2 사용

const float    US_DMAX_M   = 1.5f;
const float    US_MIN_M    = 0.02f;
const float    US_FOV_RAD  = 0.52f;
const uint32_t US_ECHO_US  = (uint32_t)(2.0f * US_DMAX_M / 343.0f * 1e6);  // 약 8.7ms
const uint32_t US_GAP_US   = US_ECHO_US + 5000;   // 간격 줄임

// ===== 상태 "정의" =====
USensor us[3] = {0};

static volatile uint8_t isr_idx_0 = 0, isr_idx_1 = 1, isr_idx_2 = 2;
static unsigned long last_trig = 0;
static int trig_idx = 0;
static bool sensors_initialized = false;

static inline void fireTrig(uint8_t pin){
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin, LOW);
}

static inline void handle_echo_edge(uint8_t i) {
  // 센서가 초기화되지 않았으면 인터럽트 무시
  if (!sensors_initialized) return;
  
  bool level = digitalRead(US_ECHO[i]);
  unsigned long now = micros();
  if (level) {
    us[i].t_rise = now;
    us[i].have_rise = true;
  } else if (us[i].have_rise) {
    us[i].t_fall = now;
    us[i].dur_us = us[i].t_fall - us[i].t_rise;
    us[i].updated = true;
    us[i].have_rise = false;
  }
}

void ISR_ECHO0(){ handle_echo_edge(isr_idx_0); }
void ISR_ECHO1(){ handle_echo_edge(isr_idx_1); }
void ISR_ECHO2(){ handle_echo_edge(isr_idx_2); }

void UltrasonicBegin() {
  sensors_initialized = false;
  
  for (int i=0;i<3;i++){
    pinMode(US_TRIG[i], OUTPUT);
    digitalWrite(US_TRIG[i], LOW);
    pinMode(US_ECHO[i], INPUT);
    us[i].have_rise  = false;
    us[i].updated    = false;
    us[i].timeout_at = 0;
    us[i].last_m     = NAN;
  }
  
  // 인터럽트 설정 시도 (센서가 없어도 오류 방지)
  bool interrupt_attached = true;
  
  // 인터럽트 핀이 유효한지 확인 후 설정
  if (digitalPinToInterrupt(US_ECHO[0]) != NOT_AN_INTERRUPT) {
    attachInterrupt(digitalPinToInterrupt(US_ECHO[0]), ISR_ECHO0, CHANGE);
  } else {
    interrupt_attached = false;
  }
  
  if (digitalPinToInterrupt(US_ECHO[1]) != NOT_AN_INTERRUPT) {
    attachInterrupt(digitalPinToInterrupt(US_ECHO[1]), ISR_ECHO1, CHANGE);
  } else {
    interrupt_attached = false;
  }
  
  if (digitalPinToInterrupt(US_ECHO[2]) != NOT_AN_INTERRUPT) {
    attachInterrupt(digitalPinToInterrupt(US_ECHO[2]), ISR_ECHO2, CHANGE);
  } else {
    interrupt_attached = false;
  }
  
  // 인터럽트가 성공적으로 설정되었을 때만 센서 초기화 완료
  if (interrupt_attached) {
    sensors_initialized = true;
    
    // 첫 번째 센서부터 시작
    last_trig = micros();
    
    // 첫 번째 센서 즉시 측정 시작
    us[0].updated = false;
    us[0].have_rise = false;
    us[0].timeout_at = micros() + (US_ECHO_US + 5000);
    fireTrig(US_TRIG[0]);
  } else {
    // 센서가 연결되지 않은 경우 기본값 설정
    for (int i=0;i<3;i++){
      us[i].last_m = 1.0f; // 안전한 거리로 설정
    }
  }
}

void UltrasonicSpinOnce() {
  // 센서가 초기화되지 않았으면 기본값 유지
  if (!sensors_initialized) {
    return;
  }
  
  unsigned long now = micros();

  // 현재 활성 센서가 측정 완료되었는지 확인
  if (us[trig_idx].updated || (us[trig_idx].timeout_at && (long)(now - us[trig_idx].timeout_at) >= 0)) {
    // 다음 센서로 이동
    trig_idx = (trig_idx + 1) % 3;
    
    // 새 센서 측정 시작
    us[trig_idx].updated    = false;
    us[trig_idx].have_rise  = false;
    us[trig_idx].timeout_at = now + (US_ECHO_US + 5000);  // 8.7ms + 5ms = 약 14ms
    fireTrig(US_TRIG[trig_idx]);
    
    last_trig = now;
  }

  for (int i=0;i<3;i++){
    noInterrupts();
    bool upd = us[i].updated;
    unsigned long dur = us[i].dur_us;
    interrupts();

    if (upd) {
      us[i].updated = false;
      float dist_m = (dur / 1e6f) * 343.0f * 0.5f;
      if (dist_m < US_MIN_M || dist_m > US_DMAX_M) dist_m = NAN;
      us[i].last_m = dist_m;
    } else if (us[i].timeout_at && (long)(now - us[i].timeout_at) >= 0) {
      us[i].timeout_at = 0;
      us[i].last_m = NAN;
    }
  }
}
