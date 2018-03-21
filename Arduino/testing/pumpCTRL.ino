#include <Servo.h>
//d2 10
//d6 14 d4 12 d7 15

#define LEFT_PUMP_ESC 14
#define RIGHT_PUMP_ESC 10
#define TP1_ESC 15
#define TP2_ESC 12
 
Servo leftPump, rightPump, tp1Pump, tp2Pump;

void setup() {
  // put your setup code here, to run once:
leftPump.attach(LEFT_PUMP_ESC);
rightPump.attach(RIGHT_PUMP_ESC);
tp1Pump.attach(TP1_ESC);
tp2Pump.attach(TP2_ESC);

leftPump.writeMicroseconds(2000);
rightPump.writeMicroseconds(2000);
tp1Pump.writeMicroseconds(2000);
tp2Pump.writeMicroseconds(2000);

delay(2000);

leftPump.writeMicroseconds(1000);
rightPump.writeMicroseconds(1000);
tp1Pump.writeMicroseconds(1000);
tp2Pump.writeMicroseconds(1000);

delay(2000);

}

void loop() {
  // put your main code here, to run repeatedly:
leftPump.writeMicroseconds(1300);
rightPump.writeMicroseconds(1300);
tp1Pump.writeMicroseconds(1300);
tp2Pump.writeMicroseconds(1300);
}
