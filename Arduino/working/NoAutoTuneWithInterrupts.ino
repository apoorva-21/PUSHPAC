#include <PID_AutoTune_v0.h>
#include <PID_v1.h>



/*
    //-1.2p -1.4roll
  TODOododo : write the root of the correction
    lolwa
*/

#include <ESP8266WiFi.h>
#include <Servo.h>
//#include <WDTUtils.h>
#include <MPU6050GyroKalmanINV_LPF_Double.h>

extern "C" {
#include "user_interface.h"
}
////CONTROLLER DEFINITIONS::
//#define CH_1 D2
//#define CH_2 D6
//#define CH_3 D13//D7
//#define CH_4 D8
//#define CH_5 D10
//#define CH_6 D11

#define CH_1 D1
#define CH_2 D0
#define CH_3 D6//D7
#define CH_4 D7
//#define CH_5 D10
//#define CH_6 D11

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

//#define ESC_CONTROL_FRONT D13
//#define ESC_CONTROL_BACK D2
//#define ESC_CONTROL_RIGHT D8
//#define ESC_CONTROL_LEFT D10

#define ESC_CONTROL_FRONT D2
#define ESC_CONTROL_BACK D13
#define ESC_CONTROL_RIGHT D10
#define ESC_CONTROL_LEFT D8

bool BOT_ENABLED = true;

long loopTimer = 0;
long stableReadingTimer = 0;
bool calibrated = false;

double ROLL_SET_POINT = 0;//true value: -1.4;
double PITCH_SET_POINT = 0;//true value = -1.2;

double LEFT_MOTOR_CORRECTION = 1;//1.03;//1.038;
//double resolutionLEFT_MOTOR_CORRECTION = 0.01;

double resolutionROLL_SET_POINT = 0.1;
double resolutionPITCH_SET_POINT = 0.1;
double resolutionOPT = 10;
double resolutionINTEGRAL_LIMIT = 10;

double ROLL_SET_POINT_BUFFER = 0;
double PITCH_SET_POINT_BUFFER = 0;

double ROLL_SET_POINT_BUFFER_INTEGRAL_TO_ZERO = 1;
double PITCH_SET_POINT_BUFFER_INTEGRAL_TO_ZERO = 1;


double OPT = 1320; //the optimal speed was 1400


double XkP = 7; //7;//12
double XkD = 0.1; //1; //20
double XkI = 2;//0.008;//3 //0.05

double YkP = 7; //7;//12
double YkD = 0.1; //1; //20
double YkI = 2;//0.008;//3 //0.05

double INTEGRAL_LIMIT = 30;
double MAX_PWM = 2000;
double PWM_CAP = 1800;
double MIN_PWM = 1000;


/*
  double ROLL_SET_POINT = -1.5;
  double PITCH_SET_POINT = -1.5;
  double OPT = 1400; //the optimal speed was 1400
  double INTEGRAL_LIMIT = 30;
*/

#define WDT_INTERVAL WDTO_1S
Servo escFront, escBack, escRight, escLeft;

double resolutionKP = 1;
double resolutionKI = 0.1;
double resolutionKD = 0.1;


const char* ssid = "apg";
const char* password = "sramlr11";
int count = 0;

WiFiServer server(80);

double pitchError = 0;
double pitchErrorPrev = 0;
double pitchErrorDerivative = 0;
double pitchErrorIntegral = 0;
double pitchCorrection = 0;

double rollError = 0;
double rollErrorPrev = 0;
double rollErrorDerivative = 0;
double rollErrorIntegral = 0;
double rollCorrection = 0;

int leftPWM = 0;
int rightPWM = 0;
int frontPWM = 0;
int backPWM = 0;


// TODO: Make calibration routine


//FAST LOOP PARAMETERS ::
double gyroCorrectionFactor = 0;

double gyroErrorRoll = 0;
double gyroErrorPitch = 0;

double gyroCorrectionRoll = 0;
double gyroCorrectionPitch = 0;

double GYRO_X_INITIAL_RATE = 0;
double GYRO_Y_INITIAL_RATE = 0;

double resolutionGyroFactor = 1;

int numIters = 1;


bool USING_KALMAN = true;
int KALMAN_LOOP_FREQ = 1;


//CONTROLLER BUFFERS::

int rightStickHorizontal, rightStickVertical, leftStickVertical, leftStickHorizontal, ch5, ch6;
int leftStickHorizontalControl, leftStickVerticalControl, rightStickHorizontalControl, rightStickVerticalControl;

int LEFT_STICK_HORIZONTAL_SET_POINT = 1513;
int LEFT_STICK_HORIZONTAL_RIGHT = 1912;
int LEFT_STICK_HORIZONTAL_LEFT = 1135;

int LEFT_STICK_VERTICAL_SET_POINT = 1545;
int LEFT_STICK_VERTICAL_UP = 1950;
int LEFT_STICK_VERTICAL_DOWN = 1210;

int RIGHT_STICK_HORIZONTAL_SET_POINT = 1521;
int RIGHT_STICK_HORIZONTAL_RIGHT = 1904;
int RIGHT_STICK_HORIZONTAL_LEFT = 1130;

int RIGHT_STICK_VERTICAL_SET_POINT = 1514;
int RIGHT_STICK_VERTICAL_UP = 1179;
int RIGHT_STICK_VERTICAL_DOWN = 1835;

double STICK_CORRECTION_FACTOR = 0.3;
double OPT_STICK_CORRECTION_FACTOR = 0.8;

double ALPHA = 0.1;

double stickCorrectionRoll = 0;
double stickCorrectionPitch = 0;
double stickCorrectionYaw = 0;
double stickCorrectionOPT = 0;

double prevStickCorrectionRoll = 0;
double prevStickCorrectionPitch = 0;
double prevStickCorrectionYaw = 0;
double prevStickCorrectionOPT = 0;

int channelOutputs[4];
int channelStartTime[4];
int channelBuffers[4];

int ch1 = 0;
int ch2 = 1;
int ch3 = 2;
int ch4 = 3;

/*
   CH_4::D2::LeftStickHorizontal(L1135-1513-R1912)
   CH_3::D13::LeftStickVertical(U1950-1545-D1210)
   CH_1::D2::RightStickHorizontal(L1130-1521-R1904)
   CH_2::D6::RightStickVertical(U1179-1514-D1835)

   rightStick : L-R :roll control
                U-D :pitch control

   leftStick : L-R :yaw control
             : U-D :throttle(OPT)
*/


//AUTO TUNING ::

PID pidX = PID(&kalAngleX, &rollCorrection, &ROLL_SET_POINT, XkP, XkI, XkD, P_ON_E, REVERSE);
PID pidY = PID(&correctedAngleY, &pitchCorrection, &PITCH_SET_POINT, YkP, YkI, YkD, P_ON_E, REVERSE);

double aTuneStep = -200;//the output corrections to make when the bot is autotuning
double aTuneNoise = 0.5;//1;//the magnitude of noise due to vibrations on the MPU
double aTuneStartValueX = 0;//initial value of the correction along  X direction
double aTuneStartValueY = 0; //intitial value of correction along Y-direcitom
unsigned int aTuneLookBack = 10; //no of timesteps to look back when searching for error extremas (oscillations)

byte ATuneModeRememberX = 2, ATuneModeRememberY = 2;
boolean tuningX = true, tuningY = true;

int PID_SAMPLE_TIME = 1;

void readChannel(int channelNo, int channelPin)
{
  if(digitalRead(channelPin) == HIGH)
    channelStartTime[channelNo] = micros();

  else channelBuffers[channelNo] = micros() - channelStartTime[channelNo];
  
}

void readChannel1()
{
  readChannel(ch1, CH_1);
}

void readChannel2()
{
  readChannel(ch2, CH_2);
}


void readChannel3()
{
  readChannel(ch3, CH_3);
}

void readChannel4()
{
  readChannel(ch4, CH_4);
}


void readRC()
{
  memcpy(channelOutputs, (const void *)channelBuffers, sizeof(channelBuffers));
}


void setupPID()
{
  //AutoPID definition ::
  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);

  pidX.SetOutputLimits(-1000, 1000);
  pidY.SetOutputLimits(-1000, 1000);

  pidX.SetSampleTime(PID_SAMPLE_TIME);
  pidY.SetSampleTime(PID_SAMPLE_TIME);

  pidX.SetTunings(XkP, XkI, XkD);
  pidY.SetTunings(YkP, YkI, YkD);

}


void calibrateMPU()
{
  updateAngles();
  updateGyro();
  ROLL_SET_POINT = kalAngleX;
  rollError = 0;
  rollCorrection = 0;
  PITCH_SET_POINT = correctedAngleY;
  pitchError = 0;
  pitchCorrection = 0;
  GYRO_X_INITIAL_RATE = gyroXrate;
  gyroErrorRoll = 0;
  GYRO_Y_INITIAL_RATE = gyroYrate;
  gyroErrorPitch = 0;
  //recalibrate the PID controllers::
  pidX = PID(&kalAngleX, &rollCorrection, &ROLL_SET_POINT, XkP, XkI, XkD, P_ON_E, REVERSE);
  pidY = PID(&correctedAngleY, &pitchCorrection, &PITCH_SET_POINT, YkP, YkI, YkD, P_ON_E, REVERSE);
  setupPID();
  Serial.println("Calibrated the MPU!");
}

void getCorrectionRoll()
{
  rollErrorDerivative = rollError - rollErrorPrev;
  rollErrorIntegral += rollError;
  if ((XkI * rollErrorIntegral) > INTEGRAL_LIMIT)
    rollErrorIntegral = INTEGRAL_LIMIT / XkI;
  if ((XkI * rollErrorIntegral) < (-1 * INTEGRAL_LIMIT))
    rollErrorIntegral = -1 * INTEGRAL_LIMIT / XkI;
  rollCorrection = XkP * rollError + XkD * rollErrorDerivative + XkI * rollErrorIntegral;///////////////////////////////////ADDED DERIVATIVE TERM//////////////

  rollErrorPrev = rollError;
}

void getCorrectionPitch()
{
  pitchErrorDerivative = pitchError - pitchErrorPrev;
  pitchErrorIntegral += pitchError;
  if ((YkI * pitchErrorIntegral) > INTEGRAL_LIMIT)
    pitchErrorIntegral = INTEGRAL_LIMIT / YkI;
  if ((YkI * pitchErrorIntegral) < (-1 * INTEGRAL_LIMIT))
    pitchErrorIntegral = -1 * INTEGRAL_LIMIT / YkI;
  pitchCorrection = YkP * pitchError + YkD * pitchErrorDerivative + YkI * pitchErrorIntegral;///////////////////////////////////ADDED DERIVATIVE TERM//////////////

  pitchErrorPrev = pitchError;
}

void getError()
{
  pitchError = correctedAngleY - PITCH_SET_POINT;
  rollError = kalAngleX - ROLL_SET_POINT;

  pitchError = pitchError > PITCH_SET_POINT_BUFFER || pitchError < (-PITCH_SET_POINT_BUFFER) ? pitchError : 0;
  rollError = rollError > ROLL_SET_POINT_BUFFER || rollError < (-ROLL_SET_POINT_BUFFER) ? rollError : 0;

  pitchErrorIntegral = pitchError < PITCH_SET_POINT_BUFFER_INTEGRAL_TO_ZERO || pitchError > (-PITCH_SET_POINT_BUFFER_INTEGRAL_TO_ZERO) ? pitchErrorIntegral : 0;
  rollErrorIntegral = rollError < ROLL_SET_POINT_BUFFER_INTEGRAL_TO_ZERO || rollError > (-ROLL_SET_POINT_BUFFER_INTEGRAL_TO_ZERO) ? rollErrorIntegral : 0;

}


double constrainValue(double upLimit, double downLimit, double value)
{
  if (value > upLimit)
    value = upLimit;
  else if (value < downLimit)
    value = downLimit;
  return value;
}

//**********GYRO************
void getGyroError()
{
  gyroErrorRoll = gyroXrate - GYRO_X_INITIAL_RATE;
  gyroErrorPitch = gyroYrate - GYRO_Y_INITIAL_RATE;

}
void correctGyroError()
{
  gyroCorrectionRoll = gyroCorrectionFactor * gyroErrorRoll;
  gyroCorrectionPitch = gyroCorrectionFactor * gyroErrorPitch;

  if (!USING_KALMAN)
  {

    leftPWM = constrainValue(PWM_CAP, MIN_PWM, LEFT_MOTOR_CORRECTION * (OPT  + gyroCorrectionRoll + stickCorrectionRoll + stickCorrectionYaw));
    rightPWM = constrainValue(PWM_CAP, MIN_PWM, (OPT - gyroCorrectionRoll - stickCorrectionRoll + stickCorrectionYaw) / LEFT_MOTOR_CORRECTION);;
    frontPWM = constrainValue(PWM_CAP, MIN_PWM, OPT - gyroCorrectionPitch - stickCorrectionPitch - stickCorrectionYaw);
    backPWM = constrainValue(PWM_CAP, MIN_PWM, OPT + gyroCorrectionPitch + stickCorrectionPitch - stickCorrectionYaw);//perfect for 17-03-2018

    int frontCorrection = 0.3125 * (backPWM - 1581) + 29;
    if (0) { //(backPWM > 1488){
      escFront.writeMicroseconds(frontPWM + frontCorrection);
      escBack.writeMicroseconds(backPWM);
      escLeft.writeMicroseconds(leftPWM);
      escRight.writeMicroseconds(rightPWM);//******
    }
    else {
      escFront.writeMicroseconds(frontPWM);
      escBack.writeMicroseconds(backPWM);
      escLeft.writeMicroseconds(leftPWM);
      escRight.writeMicroseconds(rightPWM);//******
    }
  }
}
//right angvel -ve roll +ve
//front both angvel and pitch positive
/*
   void getControllerCorrection() {

  readController();
  stickCorrectionOPT = STICK_CORRECTION_FACTOR * leftStickVertical;
  stickCorrectionPitch = STICK_CORRECTION_FACTOR * rightStickVertical;
  stickCorrectionRoll = STICK_CORRECTION_FACTOR * rightStickHorizontal;
  stickCorrectionYaw = STICK_CORRECTION_FACTOR * leftStickVertical;

  }

*/
void writePWMRoll()
{
  leftPWM = constrainValue(PWM_CAP, MIN_PWM, LEFT_MOTOR_CORRECTION * (OPT + rollCorrection + gyroCorrectionRoll + stickCorrectionRoll + stickCorrectionYaw));
  rightPWM = constrainValue(PWM_CAP, MIN_PWM, (OPT - rollCorrection - gyroCorrectionRoll - stickCorrectionRoll + stickCorrectionYaw) / LEFT_MOTOR_CORRECTION);
  escLeft.writeMicroseconds(leftPWM);
  escRight.writeMicroseconds(rightPWM);//******
}

void writePWMPitch()
{
  frontPWM = constrainValue(PWM_CAP, MIN_PWM, OPT - pitchCorrection - gyroCorrectionPitch - stickCorrectionPitch - stickCorrectionYaw);
  backPWM = constrainValue(PWM_CAP, MIN_PWM, OPT + pitchCorrection + gyroCorrectionPitch + stickCorrectionPitch - stickCorrectionYaw);
  int frontCorrection = 0.3125 * (backPWM - 1581) + 29;
  if (0) //(frontCorrection > 0)
    escFront.writeMicroseconds(frontPWM + frontCorrection);
  else
    escFront.writeMicroseconds(frontPWM);
  escBack.writeMicroseconds(backPWM);
}


void setupESC() {
  escFront.attach(ESC_CONTROL_FRONT);
  escBack.attach(ESC_CONTROL_BACK);
  escRight.attach(ESC_CONTROL_RIGHT);
  escLeft.attach(ESC_CONTROL_LEFT);

  escFront.writeMicroseconds(MAX_PWM);
  escBack.writeMicroseconds(MAX_PWM);
  escRight.writeMicroseconds(MAX_PWM);
  escLeft.writeMicroseconds(MAX_PWM);

  delay(2000);

  escFront.writeMicroseconds(MIN_PWM);
  escBack.writeMicroseconds(MIN_PWM);
  escRight.writeMicroseconds(MIN_PWM);
  escLeft.writeMicroseconds(MIN_PWM);

  delay(2000);
}
void printPWM(bool printingMore) {
  Serial.print("Left: ");
  Serial.print (leftPWM);
  Serial.print("\t");

  Serial.print("Right: ");
  Serial.print (rightPWM);
  Serial.print("\t");


  Serial.print("Front: ");
  Serial.print (frontPWM);
  Serial.print("\t");


  Serial.print("Back: ");
  Serial.print (backPWM);
  Serial.print("\t");

  if (!printingMore)
    Serial.println();
}
void printPWMWithKalman(bool printingMore)
{
  Serial.print("Left: ");
  Serial.print (leftPWM);
  Serial.print("\t");

  Serial.print("Right: ");
  Serial.print (rightPWM);
  Serial.print("\t");


  Serial.print("Front: ");
  Serial.print (frontPWM);
  Serial.print("\t");


  Serial.print("Back: ");
  Serial.print (backPWM);
  Serial.print("\t");

  Serial.print("Pitch: ");
  Serial.print (correctedAngleY);
  Serial.print("\t");

  Serial.print("Roll: ");
  Serial.print (kalAngleX);
  Serial.print("\t");


  Serial.print("Pitch Error: ");
  Serial.print (pitchError);
  Serial.print("\t");

  Serial.print("Roll Error: ");
  Serial.print (rollError);
  Serial.print("\t");


  //  Serial.print("Pitch Rate: ");
  //  Serial.print (gyroYrate);
  //  Serial.print("\t");
  //
  //  Serial.print("Roll Rate: ");
  //  Serial.print (gyroXrate);
  //  Serial.print("\t");
  //
  //  Serial.print("INITIAL_GYRO_Y_RATE: ");
  //  Serial.print (GYRO_Y_INITIAL_RATE);
  //  Serial.print("\t");
  //
  //  Serial.print("INITIAL_GYRO_X_RATE: ");
  //  Serial.print (GYRO_X_INITIAL_RATE);
  //  Serial.print("\t");

  if (!printingMore)
    Serial.println();

}

void printMPU(bool printingMore)
{
  Serial.print("Pitch: ");
  Serial.print (correctedAngleY);
  Serial.print("\t");

  Serial.print("Roll: ");
  Serial.print (kalAngleX);
  Serial.print("\t");
  if (!printingMore)
    Serial.println();
}

void printGyro(bool printingMore)
{
  Serial.print("Pitch Rate: ");
  Serial.print (gyroYrate);
  Serial.print("\t");

  Serial.print("Roll Rate: ");
  Serial.print (gyroXrate);
  Serial.print("\t");

  Serial.print("Pitch Rate Error : ");
  Serial.print (gyroCorrectionPitch);
  Serial.print("\t");

  Serial.print("Roll Rate Error : ");
  Serial.print (gyroCorrectionRoll);
  Serial.print("\t");

  if (!printingMore)
    Serial.println();
}
//Webpage Functions::
void setupCommunications()
{
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // Start the server
  server.begin();
  Serial.println("Server started");

  // Print the IP address
  Serial.print("Use this URL : ");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
}


void updateParameters()
{
  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

  // Wait until the client sends some data
  Serial.println("new client");
  if (client.available()) {
    delay(1);


    // Read the first line of the request
    String request = client.readStringUntil('\r');
    Serial.println(request);
    client.flush();
    Serial.println("CLIENT FLUSHED!");

    // Match the request
    //GYRO Tunables ::

    if (request.indexOf("/gyroFactorUp") != -1) {
      gyroCorrectionFactor += resolutionGyroFactor;
    }
    if (request.indexOf("/gyroFactorDown") != -1) {
      gyroCorrectionFactor -= resolutionGyroFactor;
    }

    //                                                                   Gyro tunables end ::
    if (request.indexOf("/kPUp") != -1) {
      XkP += resolutionKP;
    }
    if (request.indexOf("/kPDown") != -1) {
      XkP -= resolutionKP;
    }

    if (request.indexOf("/kPUp") != -1) {
      YkP += resolutionKP;
    }
    if (request.indexOf("/kPDown") != -1) {
      YkP -= resolutionKP;
    }



    if (request.indexOf("/kIUp") != -1) {
      XkI += resolutionKI;
    }
    if (request.indexOf("/kIDown") != -1) {
      XkI -= resolutionKI;
    }

    if (request.indexOf("/kIUp") != -1) {
      YkI += resolutionKI;
    }
    if (request.indexOf("/kIDown") != -1) {
      YkI -= resolutionKI;
    }

    if (request.indexOf("/kIResUp") != -1) {
      resolutionKI *= 10;
    }
    if (request.indexOf("/kIResDown") != -1) {
      resolutionKI /= 10;
    }



    if (request.indexOf("/kDUp") != -1) {
      XkD += resolutionKD;
    }
    if (request.indexOf("/kDDown") != -1) {
      XkD -= resolutionKD;
    }

    if (request.indexOf("/kDUp") != -1) {
      YkD += resolutionKD;
    }
    if (request.indexOf("/kDDown") != -1) {
      YkD -= resolutionKD;
    }

    if (request.indexOf("/kDResUp") != -1) {
      resolutionKD *= 10;
    }
    if (request.indexOf("/kDResDown") != -1) {
      resolutionKD /= 10;
    }

    /*
      double ROLL_SET_POINT = -1.5;
      double PITCH_SET_POINT = -1.5;
      double OPT = 1400; //the optimal speed was 1400
      double INTEGRAL_LIMIT = 30;
    */

    if (request.indexOf("/OPTUp") != -1) {
      OPT += resolutionOPT;
    }
    if (request.indexOf("/OPTDown") != -1) {
      OPT -= resolutionOPT;
    }

    if (request.indexOf("/OPTResUp") != -1) {
      resolutionOPT *= 10;
    }
    if (request.indexOf("/OPTResDown") != -1) {
      resolutionOPT /= 10;
    }

    if (request.indexOf("/INTEGRAL_LIMITUp") != -1) {
      INTEGRAL_LIMIT += resolutionINTEGRAL_LIMIT;
    }
    if (request.indexOf("/INTEGRAL_LIMITDown") != -1) {
      INTEGRAL_LIMIT -= resolutionINTEGRAL_LIMIT;
    }


    if (request.indexOf("/kalmanSwitch") != -1) {
      USING_KALMAN = !USING_KALMAN;
    }

    if (request.indexOf("/resetSetPoints") != -1) {
      calibrateMPU();
    }

    // Return the response
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println(""); //  do not forget this one
    client.println("<!DOCTYPE HTML>");
    Serial.println("Stage 1 Clear!");
    client.flush();
    client.print("<html><body><form action = \"/botSwitch\"><input type = \"submit\" value = \"BOT SWITCH\"> <p> BOT STATUS = ");
    client.print(BOT_ENABLED);
    client.print("<p></form><table width = \"50%\" height = \"25%\"><tr><td colspan = \"5\"> Motor Values </td></tr><tr><td>Left</td><td>Right</td><td>Front</td><td>Back</td></tr><tr><td>");
    client.print(leftPWM);
    client.print("</td><td>");
    client.print(rightPWM);
    client.print("</td><td>");
    client.print(frontPWM);
    client.print("</td><td>");
    client.print(backPWM);
    client.print("</td>");
    client.print("<tr><td colspan = \"5\"> MPU Values </td></tr><tr><td>Direction</td><td>Kalman Angle</td><td>Angle Error</td><td>Gyro Rate</td><td>Rate Error</td></tr><tr><td>X Direction</td><td>");
    client.print(kalAngleX);
    client.print("</td><td>");
    client.print(rollError);
    client.print("</td><td>");
    client.print(gyroXrate);
    client.print("</td><td>");
    client.print(gyroErrorRoll);
    client.print("</td></tr>");
    client.print("<tr><td>Y Direction</td><td>");
    client.print(correctedAngleY);
    client.print("</td><td>");
    client.print(pitchError);
    client.print("</td><td>");
    client.print(gyroYrate);
    client.print("</td><td>");
    client.print(gyroErrorPitch);
    client.print("</td></tr></table>");
    Serial.println("Stage 2 Clear!");
    client.flush();

    client.print("GYRO FACTOR = ");
    client.print(gyroCorrectionFactor);
    client.println("<br><form action = \"/gyroFactorUp\"><input type = \"submit\" value = \"Increase GYRO FACTOR\"></form><form action = \"/gyroFactorDown\"><input type = \"submit\" value = \"Decrease GYRO FACTOR\"></form><br><br>");
    client.print("OPT = ");
    client.print(OPT);
    client.println("<form action = \"/OPTUp\"><input type = \"submit\" value = \"Increase OPT\"></form><form action = \"/OPTDown\"><input type = \"submit\" value = \"Decrease OPT\"></form><br><br>");
    client.print("Resolution for OPT = ");
    client.print(resolutionOPT);
    client.println("<form action = \"/OPTResUp\"><input type = \"submit\" value = \"RESOLUTION for OPT x 10\"></form><form action = \"/OPTResDown\"><input type = \"submit\" value = \"RESOLUTION  for OPT / 10\"></form><br><br><br>");
    Serial.println("Stage 3 Clear!");
    client.flush();

    client.print("kP = ");
    client.print(XkP);
    client.println("<br><form action = \"/kPUp\"><input type = \"submit\" value = \"Increase kP\"></form><form action = \"/kPDown\"><input type = \"submit\" value = \"Decrease kP\"></form><br><br>");
    client.print("kI = ");
    client.print(XkI);
    client.println("<form action = \"/kIUp\"><input type = \"submit\" value = \"Increase kI\"></form><form action = \"/kIDown\"><input type = \"submit\" value = \"Decrease kI\"></form><br><br>");
    client.print("Resolution for kI = ");
    client.print(resolutionKI);
    client.println("<form action = \"/kIResUp\"><input type = \"submit\" value = \"RESOLUTION for kI x 10\"></form><form action = \"/kIResDown\"><input type = \"submit\" value = \"RESOLUTION  for kI / 10\"></form><br><br><br>");
    client.print("kD = ");
    client.print(XkD);
    client.println("<form action = \"/kDUp\"><input type = \"submit\" value = \"Increase kD\"></form><form action = \"/kDDown\"><input type = \"submit\" value = \"Decrease kD\"></form><br><br>");
    client.print("Resolution for kD = ");
    client.print(resolutionKD);
    client.println("<form action = \"/kDResUp\"><input type = \"submit\" value = \"RESOLUTION for kD x 10\"></form><form action = \"/kDResDown\"><input type = \"submit\" value = \"RESOLUTION  for kD / 10\"></form><br><br><br>");
    Serial.println("Stage 4 Clear!");
    client.flush();

    client.print("kP = ");
    client.print(YkP);
    client.println("<br><form action = \"/kPUp\"><input type = \"submit\" value = \"Increase kP\"></form><form action = \"/kPDown\"><input type = \"submit\" value = \"Decrease kP\"></form><br><br>");
    client.print("kI = ");
    client.print(YkI);
    client.println("<form action = \"/kIUp\"><input type = \"submit\" value = \"Increase kI\"></form><form action = \"/kIDown\"><input type = \"submit\" value = \"Decrease kI\"></form><br><br>");
    client.print("Resolution for kI = ");
    client.print(resolutionKI);
    client.println("<form action = \"/kIResUp\"><input type = \"submit\" value = \"RESOLUTION for kI x 10\"></form><form action = \"/kIResDown\"><input type = \"submit\" value = \"RESOLUTION  for kI / 10\"></form><br><br><br>");
    client.print("kD = ");
    client.print(YkD);
    client.println("<form action = \"/kDUp\"><input type = \"submit\" value = \"Increase kD\"></form><form action = \"/kDDown\"><input type = \"submit\" value = \"Decrease kD\"></form><br><br>");
    client.print("Resolution for kD = ");
    client.print(resolutionKD);
    client.println("<form action = \"/kDResUp\"><input type = \"submit\" value = \"RESOLUTION for kD x 10\"></form><form action = \"/kDResDown\"><input type = \"submit\" value = \"RESOLUTION  for kD / 10\"></form><br><br><br>");
    Serial.println("Stage 4 Clear!");
    client.flush();

    client.print("INTEGRAL_LIMIT = ");
    client.print(INTEGRAL_LIMIT);
    client.println("<form action = \"/INTEGRAL_LIMITUp\"><input type = \"submit\" value = \"Increase INTEGRAL_LIMIT\"></form><form action = \"/INTEGRAL_LIMITDown\"><input type = \"submit\" value = \"Decrease INTEGRAL_LIMIT\"></form><br><br>");
    client.print("<form action = \"/resetSetPoints\"><input type = \"submit\" value = \"CALIBRATE SET POINTS\"></form><br><br><br><br>");
    client.println("<form action = \"/\"><input type = \"submit\" value = \"HOMEPAGE\"></form><br><form action = \"/kalmanSwitch\"><input type = \"submit\" value = \"KALMAN FILTER\">");
    client.print(USING_KALMAN);
    client.print("</form><br><br><br><br>");
    client.print("<p>leftStickHorizontal");
    client.print(leftStickHorizontal);
    client.print("</p><br><br>");
    client.print("<p>leftStickVertical");
    client.print(leftStickVertical);
    client.print("</p><br><br>");
    client.print("<p>rightStickHorizontal");
    client.print(rightStickHorizontal);
    client.print("</p><br><br>");
    client.print("<p>rightStickVertical");
    client.print(rightStickVertical);
    client.print("</p><br><br>");
    client.println("</body></html>");
    Serial.println("All Clear!");
    client.flush();
  }
  delay(1);
  Serial.println("Client disconnected");
  Serial.println("");

  pidX.SetTunings(XkP, XkI, XkD);
  pidY.SetTunings(YkP, YkI, YkD);
}


void printParameters()
{
  Serial.print("Roll Set Point: ");
  Serial.print (ROLL_SET_POINT);
  Serial.print("\t");


  Serial.print("Pitch Set Point: ");
  Serial.print (PITCH_SET_POINT);
  Serial.print("\t");


  Serial.print("OPT: ");
  Serial.print (OPT);
  Serial.print("\t");


  Serial.print("INTEGRAL LIMIT: ");
  Serial.print (INTEGRAL_LIMIT);
  Serial.print("\t");

  Serial.print("USING_KALMAN: ");
  Serial.print (USING_KALMAN);
  Serial.print("\t");
  //
  //  Serial.print("kP: ");
  //  Serial.print (kP);
  //  Serial.print("\t");
  //
  //  Serial.print("resolutionKP: ");
  //  Serial.print (resolutionKP);
  //  Serial.print("\t");
  //
  //  Serial.print("kI: ");
  //  Serial.print (kI);
  //  Serial.print("\t");
  //
  //  Serial.print("resolutionKI: ");
  //  Serial.print (resolutionKI);
  //  Serial.print("\t");
  //
  //
  //  Serial.print("kD: ");
  //  Serial.print (kD);
  //  Serial.print("\t");
  //
  //  Serial.print("resolutionKD: ");
  //  Serial.print (resolutionKD);
  //  Serial.print("\t");
  //
  Serial.println();
}

void printKalman() {

  Serial.print("PITCH : ");
  Serial.print(correctedAngleY);
  Serial.print("ROLL : ");
  Serial.print(kalAngleX);
  Serial.println();
}

void printConstants(bool printingMore) {
  Serial.print(" XkP = ");
  Serial.print(XkP);
  Serial.print(" XkI = ");
  Serial.print(XkI);
  Serial.print(" XkD = ");
  Serial.print(XkD);

  Serial.print(" YkP = ");
  Serial.print(YkP);
  Serial.print(" YkI = ");
  Serial.print(YkI);
  Serial.print(" YkD = ");
  Serial.print(YkD);

  if (!printingMore)
    Serial.println();
  
}

/*void setupController() {

  pinMode(CH_1, INPUT);
  pinMode(CH_2, INPUT);
  pinMode(CH_3, INPUT);
  pinMode(CH_4, INPUT);
  //  pinMode(CH_5, INPUT);
  //  pinMode(CH_6, INPUT);
  Serial.println("CONTROLLER SETUP COMPLETE!");

}*/

/*
   #define CH_1 D0
  #define CH_2 D1
  #define CH_3 D6//D7
  #define CH_4 D7
  //#define CH_5 D10
  //#define CH_6 D11
   CH_4::D7::LeftStickHorizontal(L1145-1513-R1912)
   CH_3::D6::LeftStickVertical(U1950-1545-D1200)
   CH_2::D0::RightStickVertical(U1179-1514-D1835)
   CH_1::D1::RightStickHorizontal(L1140-1521-R1904)

   rightStick : L-R :roll control
                U-D :pitch control

   leftStick : L-R :yaw control
             : U-D :throttle(OPT)
*/
int mapValue(int sourceValue, int sourceDownlimit, int sourceUplimit, int targetDownlimit, int targetUplimit)
{
  int targetValue = targetDownlimit + sourceValue * (targetUplimit - targetDownlimit) / (sourceUplimit - sourceDownlimit);
  return targetValue;
}

void readController()
{
  readRC();
  rightStickHorizontal = constrainValue(1000, -1000, mapValue(channelOutputs[ch1] - RIGHT_STICK_HORIZONTAL_LEFT, RIGHT_STICK_HORIZONTAL_LEFT, RIGHT_STICK_HORIZONTAL_RIGHT, -1000, 1000));
  rightStickVertical = constrainValue(1000, -1000, mapValue((channelOutputs[ch2] - RIGHT_STICK_VERTICAL_DOWN - 15), RIGHT_STICK_VERTICAL_DOWN, RIGHT_STICK_VERTICAL_UP, -1000, 1000));
  leftStickVertical = constrainValue(2500, 1000, mapValue((channelOutputs[ch3] - LEFT_STICK_VERTICAL_DOWN - 15) , LEFT_STICK_VERTICAL_DOWN, LEFT_STICK_VERTICAL_UP, 1000, 2500));
  leftStickHorizontal = constrainValue(1000, -1000, mapValue(channelOutputs[ch4] - LEFT_STICK_HORIZONTAL_LEFT, LEFT_STICK_HORIZONTAL_LEFT, LEFT_STICK_HORIZONTAL_RIGHT, -1000, 1000));

  //  leftStickHorizontalControl = leftStickHorizontal - LEFT_STICK_HORIZONTAL_SET_POINT
}

void getControllerCorrection() {

  readController();
  stickCorrectionOPT = (1 - ALPHA) * prevStickCorrectionOPT + ALPHA * OPT_STICK_CORRECTION_FACTOR * leftStickVertical;
  OPT = stickCorrectionOPT;
  stickCorrectionPitch = (1 - ALPHA) * prevStickCorrectionPitch + ALPHA * STICK_CORRECTION_FACTOR * rightStickVertical;
  stickCorrectionRoll = (1 - ALPHA) * prevStickCorrectionRoll + ALPHA * STICK_CORRECTION_FACTOR * rightStickHorizontal;
//  stickCorrectionYaw = (1 - ALPHA) * prevStickCorrectionYaw + ALPHA * STICK_CORRECTION_FACTOR * leftStickHorizontal;
  prevStickCorrectionOPT = stickCorrectionOPT;
  prevStickCorrectionRoll = stickCorrectionRoll;
  prevStickCorrectionPitch = stickCorrectionPitch;
  prevStickCorrectionYaw = stickCorrectionYaw;

}

void printController(bool printMore)
{
  Serial.print("  rightStickHorizontal : ");
  Serial.print(rightStickHorizontal);

  Serial.print("  rightStickVertical : ");
  Serial.print(rightStickVertical);

  Serial.print("  leftStickVertical : ");
  Serial.print(leftStickVertical);

  Serial.print("  leftStickHorizontal : ");
  Serial.print(leftStickHorizontal);

  if (!printMore)
    Serial.println();
}

void printControllerCorrection(bool printMore)
{
  Serial.print("  stickCorrectionRoll : ");
  Serial.print(stickCorrectionRoll);

  Serial.print("  stickCorrectionPitch : ");
  Serial.print(stickCorrectionPitch);

  Serial.print("  stickCorrectionYaw : ");
  Serial.print(stickCorrectionYaw);

  Serial.print("  stickCorrectionOPT : ");
  Serial.print(stickCorrectionOPT);

  if (!printMore)
    Serial.println();
}

void setupController()
{
  pinMode(CH_1, INPUT);
  pinMode(CH_2, INPUT);
  pinMode(CH_3, INPUT);
  pinMode(CH_4, INPUT);

  attachInterrupt(digitalPinToInterrupt(CH_1), readChannel1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_2), readChannel2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_3), readChannel3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_4), readChannel4, CHANGE);
}

void setup() {
  disableInvertedConfig();
  
  setupPID();

  //  Serial.begin(115200);
  //  wdt_enable(WDT_INTERVAL);
  setupMPU();
  //  wdt_disable();
  //  Serial.println("MPU Initialized!");
  setupESC();
  //  Serial.println("ESCs Initialized!");

  setupController();

  setupCommunications();
  //  printPWMWithKalman();
  calibrateMPU();
  stableReadingTimer = millis();
  calibrated = false;
}


//-15 to 12 pitch and roll
void loop() {

  loopTimer = millis();
  updateParameters();//ENABLE KALMAN TO SEE THE ERROR
  if (BOT_ENABLED) {
    getControllerCorrection();
    //  printControllerCorrection(false);
    //  wdt_enable(WDT_INTERVAL);
    updateGyro();
    //  wdt_disable();
    getGyroError();
    correctGyroError();
    if ((USING_KALMAN) && (numIters % KALMAN_LOOP_FREQ == 0))
    {

      numIters = 0;
      updateAngles();
      getError();
      pidX.Compute();
      writePWMRoll();
      pidY.Compute();
      writePWMPitch();

    }

    if (USING_KALMAN) {
      numIters ++;
    }


    //CALIBRATION ROUTINE::
    //  if((millis() - stableReadingTimer > 10000) && (!calibrated)){
    //     //set the initial rate to what it is during setup time::
    //     calibrated = true;
    //    calibrateMPU();
    //  }

    //Write out the updated PID Params:

    

    /* Print Data */

    //Serial.print("  ");
    //    printPWM(true);
    //    printController(false);

    //Serial.print("Loop Time = ");
    //Serial.println(millis() - loopTimer);

    //  printMPU(true);
    //  printConstants(false);

    //printKalman();
    //  printGyro();
    //delay(5000);
    //printParameters();
  }
}


/*
   CH_4::D2::LeftStickHorizontal(L1145-1513-R1912)
   CH_3::D13::LeftStickVertical(U1950-1545-D1200)
   CH_2::D6::RightStickVertical(U1179-1514-D1835)
   CH_1::D2::RightStickHorizontal(L1140-1521-R1904)

   rightStick : L-R :roll control
                U-D :pitch control

   leftStick : L-R :yaw control
             : U-D :throttle(OPT)
*/

