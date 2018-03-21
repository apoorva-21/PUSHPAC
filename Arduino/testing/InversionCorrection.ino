#include <MPU6050GyroKalmanINV.h>

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
setupMPU();
enableInvertedConfig();
}

void loop() {
  // put your main code here, to run repeatedly:
updateAngles();
updateGyro();

/*float correctedPitch = kalAngleY + 180;
if(correctedPitch > 180)
  correctedPitch -= 360;*/
Serial.print("Pitch from Kalman = ");
Serial.print(kalAngleY);
//kalAngleY = correctedPitch;
Serial.print(" CorrectedPitch = ");
Serial.print(correctedAngleY);
Serial.print("Roll from Kalman = ");
Serial.print(kalAngleX);
Serial.print(" Angular Velocity in X = ");
Serial.print(gyroXrate);
Serial.print(" Angular Velocity in Y = ");
Serial.print(gyroYrate);
Serial.println();
}

