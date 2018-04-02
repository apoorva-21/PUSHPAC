#include "esp32-hal-ledc.h"

#define ESC_CONTROL_FRONT D2
#define ESC_CONTROL_BACK D13
#define ESC_CONTROL_RIGHT D10
#define ESC_CONTROL_LEFT D8

#define TIMER_WIDTH 16

double MAX_PWM = 2000;
double MIN_PWM = 1000;

//Servo escFront, escBack, escRight, escLeft;
void setup() {
  ledcSetup(1, 50, TIMER_WIDTH);
  ledcAttachPin(1, 2);
//  ledcWrite(1, MAX_PWM);
  //  escLeft.writeMicroseconds(MAX_PWM);
//  delay(2000);
  //  escLeft.writeMicroseconds(MIN_PWM);
//  ledcWrite(1, MIN_PWM);
//  delay(2000);
}
void loop() {
  int i = 0;
  for (int i = 0; i < 8888; i += 100)
  {
    ledcWrite(1, i);
    delay(100);
  }
  
  //  escLeft.writeMicroseconds(1100);
}
