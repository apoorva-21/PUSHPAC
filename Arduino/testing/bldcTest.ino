#include <Servo.h>
//d2
//backd8
//fwdd10d13

Servo pinToAttach1,pinToAttach2,pinToAttach3,pinToAttach4;
void setup() {
pinToAttach1.attach(14 );
delay(2000);

pinToAttach1.writeMicroseconds(2000);

delay(2000);

pinToAttach1.writeMicroseconds(1000);


//digitalWrite(D13, LOW);
delay(2000);
//pinMode(D13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
pinToAttach1.writeMicroseconds(1200);

//digitalWrite(D13, HIGH);

//delay(2000);
//pinToAttach1.writeMicroseconds(2000);
//pinToAttach2.writeMicroseconds(2000);
//pinToAttach3.writeMicroseconds(2000);
//pinToAttach4.writeMicroseconds(2000);
//digitalWrite(D13, LOW);
//delay(2000);


}
