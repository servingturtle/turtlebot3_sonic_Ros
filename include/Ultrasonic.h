/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*     http://www.apache.org/licenses/LICENSE-2.0
*******************************************************************************/

#pragma once
#include <Arduino.h>   // uint8_t, uint32_t 등

// ===== 측정 설정 (정의는 Ultrasonic.cpp) =====
extern const float    US_DMAX_M;
extern const float    US_MIN_M;
extern const float    US_FOV_RAD;
extern const uint32_t US_ECHO_US;
extern const uint32_t US_GAP_US;

// ===== 핀맵 (정의는 Ultrasonic.cpp) =====
extern const uint8_t US_TRIG[3];
extern const uint8_t US_ECHO[3];

// ===== 센서 상태 구조체 =====
struct USensor {
  volatile bool     have_rise;      // 상승에지 감지 플래그
  volatile bool     updated;        // 측정 완료 플래그
  volatile unsigned long t_rise;    // 상승에지 시각
  volatile unsigned long t_fall;    // 하강에지 시각
  volatile unsigned long dur_us;    // 에코 펄스폭(μs)

  unsigned long timeout_at;         // 타임아웃 시각 (micros 기준)
  float         last_m;             // 마지막 유효 거리(m) — 초기화는 .cpp에서!
};

// 전역 상태 배열 (정의는 Ultrasonic.cpp)
extern USensor us[3];

// ===== 드라이버 API =====
void UltrasonicBegin();     // setup()에서 1회 호출
void UltrasonicSpinOnce();  // loop()에서 주기적으로 호출 (논블로킹)
