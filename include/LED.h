#include <Arduino.h>
#include <math.h>
#include "Ultrasonic.h"
// === RGB LED 설정 ===
// 공통 애노드(+) LED면 1, 공통 캐소드(-)면 0
#define RGB_COMMON_ANODE 0

// PWM 가능한 핀으로 선택! (초음파 TRIG/ECHO와 절대 겹치지 않게)
#define RGB_R_PIN 9
#define RGB_G_PIN 10
#define RGB_B_PIN 11
/*
// PWM 가능한 핀으로 선택! (초음파 TRIG/ECHO와 절대 겹치지 않게)
const int RGB_R_PIN = 9;   // 예: PWM
const int RGB_G_PIN = 10;   // 예: PWM
const int RGB_B_PIN = 11;   // 예: PWM
*/

uint8_t distToPwm(float m);
void updateRgbFromUltrasound();
void setupRgbPins();   // 핀모드 설정용 (setup()에서 한 번 호출)