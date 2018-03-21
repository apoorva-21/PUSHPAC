#include <MPU6050GyroKalman.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  setupMPU();

}
//+ve roll to the right, +ve to the front
void loop() {
  // put your main code here, to run repeatedly:
updateGyro();
Serial.print("Gyro X Rate : ");
Serial.print(gyroXrate);
Serial.print("\t");
Serial.print("Gyro Y Rate : ");
Serial.print(gyroYrate);
Serial.println();
}
