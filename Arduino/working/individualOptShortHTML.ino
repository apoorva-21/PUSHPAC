/*
    //-1.2p -1.4roll
  TODOododo : write the root of the correction
    lolwa
*/

#include <ESP8266WiFi.h>
#include <Servo.h>
//#include <WDTUtils.h>
#include <MPU6050GyroKalmanINV_LPF.h>

extern "C" {
#include "user_interface.h"
}

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

#define ESC_CONTROL_FRONT D13
#define ESC_CONTROL_BACK D2
#define ESC_CONTROL_RIGHT D8
#define ESC_CONTROL_LEFT D10

long loopTimer = 0;
long stableReadingTimer = 0;
bool calibrated = false;

float ROLL_SET_POINT = 0;//true value: -1.4;
float PITCH_SET_POINT = 0;//true value = -1.2;

float LEFT_MOTOR_CORRECTION = 1;//1.03;//1.038;
//float resolutionLEFT_MOTOR_CORRECTION = 0.01;

float resolutionROLL_SET_POINT = 0.1;
float resolutionPITCH_SET_POINT = 0.1;
float resolutionOPT = 10;
float resolutionINTEGRAL_LIMIT = 10;

float ROLL_SET_POINT_BUFFER = 0;
float PITCH_SET_POINT_BUFFER = 0;

float ROLL_SET_POINT_BUFFER_INTEGRAL_TO_ZERO = 1;
float PITCH_SET_POINT_BUFFER_INTEGRAL_TO_ZERO = 1;


float OPTR = 1376; //the optimal speed was 1400
float OPTL = 1440;//1398  
float OPTF = 1416;
float OPTB = 1410;

float kP = 0; //7;//12
float kD = 0; //1; //20
float kI = 0;//0.008;//3 //0.05
float INTEGRAL_LIMIT = 30;
float MAX_PWM = 2000;
float MIN_PWM = 1000;


/*
  float ROLL_SET_POINT = -1.5;
  float PITCH_SET_POINT = -1.5;
  float OPT = 1400; //the optimal speed was 1400
  float INTEGRAL_LIMIT = 30;
*/

#define WDT_INTERVAL WDTO_1S
Servo escFront, escBack, escRight, escLeft;

float resolutionKP = 1;
float resolutionKI = 0.1;
float resolutionKD = 0.1;


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
float gyroCorrectionFactor = 0;

float gyroErrorRoll = 0;
float gyroErrorPitch = 0;

float gyroCorrectionRoll = 0;
float gyroCorrectionPitch = 0;

float GYRO_X_INITIAL_RATE = 0;
float GYRO_Y_INITIAL_RATE = 0;

float resolutionGyroFactor = 1;

int numIters = 1;


bool USING_KALMAN = true;
int KALMAN_LOOP_FREQ = 1;

void calibrateMPU()
{
  updateAngles();
  updateGyro();
  ROLL_SET_POINT = kalAngleX;
  rollError = 0;
  PITCH_SET_POINT = correctedAngleY;
  pitchError = 0;
  GYRO_X_INITIAL_RATE = gyroXrate;
  gyroErrorRoll = 0;
  GYRO_Y_INITIAL_RATE = gyroYrate;
  gyroErrorPitch = 0;
  Serial.println("Calibrated the MPU!");
}
void getCorrectionRoll()
{
  rollErrorDerivative = rollError - rollErrorPrev;
  rollErrorIntegral += rollError;
  if ((kI * rollErrorIntegral) > INTEGRAL_LIMIT)
    rollErrorIntegral = INTEGRAL_LIMIT / kI;
  if ((kI * rollErrorIntegral) < (-1 * INTEGRAL_LIMIT))
    rollErrorIntegral = -1 * INTEGRAL_LIMIT / kI;
  rollCorrection = kP * rollError + kD * rollErrorDerivative + kI * rollErrorIntegral;///////////////////////////////////ADDED DERIVATIVE TERM//////////////

  rollErrorPrev = rollError;
}

void getCorrectionPitch()
{
  pitchErrorDerivative = pitchError - pitchErrorPrev;
  pitchErrorIntegral += pitchError;
  if ((kI * pitchErrorIntegral) > INTEGRAL_LIMIT)
    pitchErrorIntegral = INTEGRAL_LIMIT / kI;
  if ((kI * pitchErrorIntegral) < (-1 * INTEGRAL_LIMIT))
    pitchErrorIntegral = -1 * INTEGRAL_LIMIT / kI;
  pitchCorrection = kP * pitchError + kD * pitchErrorDerivative + kI * pitchErrorIntegral;///////////////////////////////////ADDED DERIVATIVE TERM//////////////

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

}//aage +ve right +ve


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
    leftPWM = constrainValue(MAX_PWM, MIN_PWM, LEFT_MOTOR_CORRECTION * (OPTL - gyroCorrectionRoll));
    rightPWM = constrainValue(MAX_PWM, MIN_PWM, (OPTR  + gyroCorrectionRoll) / LEFT_MOTOR_CORRECTION);
    frontPWM = constrainValue(MAX_PWM, MIN_PWM, OPTF  + gyroCorrectionPitch);
    backPWM = constrainValue(MAX_PWM, MIN_PWM, OPTB  - gyroCorrectionPitch);//perfect for 17-03-2018

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
void writePWMRoll()
{
  leftPWM = constrainValue(MAX_PWM, MIN_PWM, LEFT_MOTOR_CORRECTION * (OPTL - rollCorrection - gyroCorrectionRoll));
  rightPWM = constrainValue(MAX_PWM, MIN_PWM, (OPTR + rollCorrection + gyroCorrectionRoll) / LEFT_MOTOR_CORRECTION);
  escLeft.writeMicroseconds(leftPWM);
  escRight.writeMicroseconds(rightPWM);//******
}

void writePWMPitch()
{
  frontPWM = constrainValue(MAX_PWM, MIN_PWM, OPTF + pitchCorrection + gyroCorrectionPitch);
  backPWM = constrainValue(MAX_PWM, MIN_PWM, OPTB - pitchCorrection - gyroCorrectionPitch);
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
void printPWM() {
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
}
void printPWMWithKalman()
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

  Serial.println();

}

void printMPU()
{
  Serial.print("Pitch: ");
  Serial.print (correctedAngleY);
  Serial.print("\t");

  Serial.print("Roll: ");
  Serial.print (kalAngleX);
  Serial.print("\t");

  Serial.println();
}

void printGyro()
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
      kP += resolutionKP;
    }
    if (request.indexOf("/kPDown") != -1) {
      kP -= resolutionKP;
    }



    if (request.indexOf("/kIUp") != -1) {
      kI += resolutionKI;
    }
    if (request.indexOf("/kIDown") != -1) {
      kI -= resolutionKI;
    }

    if (request.indexOf("/kIResUp") != -1) {
      resolutionKI *= 10;
    }
    if (request.indexOf("/kIResDown") != -1) {
      resolutionKI /= 10;
    }



    if (request.indexOf("/kDUp") != -1) {
      kD += resolutionKD;
    }
    if (request.indexOf("/kDDown") != -1) {
      kD -= resolutionKD;
    }

    if (request.indexOf("/kDResUp") != -1) {
      resolutionKD *= 10;
    }
    if (request.indexOf("/kDResDown") != -1) {
      resolutionKD /= 10;
    }

    /*
      float ROLL_SET_POINT = -1.5;
      float PITCH_SET_POINT = -1.5;
      float OPT = 1400; //the optimal speed was 1400
      float INTEGRAL_LIMIT = 30;
    */

    if (request.indexOf("/OPTUp") != -1) {
      OPTL += resolutionOPT;
      OPTR += resolutionOPT;
      OPTB += resolutionOPT;
      OPTF += resolutionOPT;
      
    }
    if (request.indexOf("/OPTDown") != -1) {
      OPTL -= resolutionOPT;
      OPTR -= resolutionOPT;
      OPTB -= resolutionOPT;
      OPTF -= resolutionOPT;
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
    //      Serial.println("Stage 1 Clear!");
    client.flush();
    client.print("<html><body><table width = \"50%\" height = \"25%\"><tr><td colspan = \"5\"> Motor Values </td></tr><tr><td>Left</td><td>Right</td><td>Front</td><td>Back</td></tr><tr><td>");
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
    //      Serial.println("Stage 2 Clear!");
    client.flush();

    client.print("GYRO FACTOR = ");
    client.print(gyroCorrectionFactor);
    client.println("<br><form action = \"/gyroFactorUp\"><input type = \"submit\" value = \"Increase GYRO FACTOR\"></form><form action = \"/gyroFactorDown\"><input type = \"submit\" value = \"Decrease GYRO FACTOR\"></form><br><br>");
    client.print("OPT = ");
    client.print(OPTL);
    client.println("<form action = \"/OPTUp\"><input type = \"submit\" value = \"Increase OPT\"></form><form action = \"/OPTDown\"><input type = \"submit\" value = \"Decrease OPT\"></form><br><br>");
    client.print("Resolution for OPT = ");
    client.print(resolutionOPT);
    client.println("<form action = \"/OPTResUp\"><input type = \"submit\" value = \"RESOLUTION for OPT x 10\"></form><form action = \"/OPTResDown\"><input type = \"submit\" value = \"RESOLUTION  for OPT / 10\"></form><br><br><br>");
    //      Serial.println("Stage 3 Clear!");
    client.flush();

    client.print("kP = ");
    client.print(kP);
    client.println("<br><form action = \"/kPUp\"><input type = \"submit\" value = \"Increase kP\"></form><form action = \"/kPDown\"><input type = \"submit\" value = \"Decrease kP\"></form><br><br>");
    client.print("kI = ");
    client.print(kI);
    client.println("<form action = \"/kIUp\"><input type = \"submit\" value = \"Increase kI\"></form><form action = \"/kIDown\"><input type = \"submit\" value = \"Decrease kI\"></form><br><br>");
    client.print("Resolution for kI = ");
    client.print(resolutionKI);
    client.println("<form action = \"/kIResUp\"><input type = \"submit\" value = \"RESOLUTION for kI x 10\"></form><form action = \"/kIResDown\"><input type = \"submit\" value = \"RESOLUTION  for kI / 10\"></form><br><br><br>");
    client.print("kD = ");
    client.print(kD);
    client.println("<form action = \"/kDUp\"><input type = \"submit\" value = \"Increase kD\"></form><form action = \"/kDDown\"><input type = \"submit\" value = \"Decrease kD\"></form><br><br>");
    client.print("Resolution for kD = ");
    client.print(resolutionKD);
    client.println("<form action = \"/kDResUp\"><input type = \"submit\" value = \"RESOLUTION for kD x 10\"></form><form action = \"/kDResDown\"><input type = \"submit\" value = \"RESOLUTION  for kD / 10\"></form><br><br><br>");
    //      Serial.println("Stage 4 Clear!");
    client.flush();

    client.print("INTEGRAL_LIMIT = ");
    client.print(INTEGRAL_LIMIT);
    client.println("<form action = \"/INTEGRAL_LIMITUp\"><input type = \"submit\" value = \"Increase INTEGRAL_LIMIT\"></form><form action = \"/INTEGRAL_LIMITDown\"><input type = \"submit\" value = \"Decrease INTEGRAL_LIMIT\"></form><br><br>");
    client.print("<form action = \"/resetSetPoints\"><input type = \"submit\" value = \"CALIBRATE SET POINTS\"></form><br><br><br><br>");
    client.println("<form action = \"/\"><input type = \"submit\" value = \"HOMEPAGE\"></form><br><form action = \"/kalmanSwitch\"><input type = \"submit\" value = \"KALMAN FILTER\">");
    client.print(USING_KALMAN);
    client.println("</form><br><br><br><br></body></html>");
    //      Serial.println("All Clear!");
    client.flush();
  }
  delay(1);
  Serial.println("Client disconnected");
  Serial.println("");
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
  Serial.print (OPTL);
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


void setup() {
  //  system_update_cpu_freq(160);DON'T.

  disableInvertedConfig();


  Serial.begin(115200);
  //  wdt_reset();
  //  wdt_enable(WDT_INTERVAL);
  setupMPU();
  //  wdt_disable();
  Serial.println("MPU Initialized!");
  setupESC();
  Serial.println("ESCs Initialized!");
  setupCommunications();
  //  printPWMWithKalman();
  calibrateMPU();
  stableReadingTimer = millis();
  calibrated = false;
}


//-15 to 12 pitch and roll
void loop() {

  loopTimer = millis();
  //  wdt_reset();
  //  wdt_enable(WDT_INTERVAL);
  updateGyro();
  //  wdt_disable();

  if ((USING_KALMAN) && (numIters % KALMAN_LOOP_FREQ == 0))
  {
    numIters = 0;
    updateAngles();
    getError();

    getCorrectionRoll();
    writePWMRoll();

    getCorrectionPitch();
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
  updateParameters();


  getGyroError();
  correctGyroError();

  /* Print Data */

  //Serial.print("  ");
  //printPWMWithKalman();//ENABLE KALMAN TO SEE THE ERROR
  //Serial.print("Loop Time = ");
  //Serial.println(millis() - loopTimer);
  printMPU();
  //printKalman();
  //  printGyro();
  //Serial.print("Gyro X Rate = ");
  //Serial.print(gyroXrate);
  //Serial.print("Gyro Error Roll = ");
  //Serial.print(gyroErrorRoll);
  //Serial.print("  gyroRateX - INITIAL GYRO RATE = ");
  //Serial.println((gyroXrate - GYRO_X_INITIAL_RATE));

  //delay(5000);
  //printParameters();

}

