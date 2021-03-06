
/* 
 *  //-1.2p -1.4roll
 TODOododo : write the root of the correction
    
 */
#include <ESP8266WiFi.h>
#include <Servo.h>
//#include <WDTUtils.h>
#include <MPU6050Kalman.h>
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

#define ESC_CONTROL_FRONT D10
#define ESC_CONTROL_BACK D8
#define ESC_CONTROL_RIGHT D13//************
#define ESC_CONTROL_LEFT D2


float ROLL_SET_POINT = -2.1;//true value: -1.4;
float PITCH_SET_POINT = -2.1;//true value = -1.2;


float resolutionROLL_SET_POINT = 0.1;
float resolutionPITCH_SET_POINT = 0.1;
float resolutionOPT = 10;
float resolutionINTEGRAL_LIMIT = 0.1;

float ROLL_SET_POINT_BUFFER = 0;
float PITCH_SET_POINT_BUFFER = 0;

float ROLL_SET_POINT_BUFFER_INTEGRAL_TO_ZERO = 1;
float PITCH_SET_POINT_BUFFER_INTEGRAL_TO_ZERO = 1;
float OPT = 1200; //the optimal speed was 1400
float kP = 7;//12
float kD =  1; //20
float kI = 0.008;//3 //0.05
float INTEGRAL_LIMIT = 30;
float MAX_PWM = 2000;
float MIN_PWM = 1000;


   /*
   * float ROLL_SET_POINT = -1.5;
float PITCH_SET_POINT = -1.5;
float OPT = 1400; //the optimal speed was 1400
float INTEGRAL_LIMIT = 30;
   */

#define WDT_INTERVAL WDTO_1S
Servo escFront, escBack, escRight, escLeft;

float resolutionKP = 0.1;
float resolutionKI = 0.1;
float resolutionKD = 0.1;


const char* ssid = "OnePlus X";
const char* password = "abcd1234";
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




 
void getCorrectionRoll()
{
  rollErrorDerivative = rollError - rollErrorPrev;
  rollErrorIntegral += rollError;
  if((kI * rollErrorIntegral) > INTEGRAL_LIMIT)
    rollErrorIntegral = INTEGRAL_LIMIT / kI;
     if((kI * rollErrorIntegral) < (-1 * INTEGRAL_LIMIT))
    rollErrorIntegral = -1 * INTEGRAL_LIMIT / kI;
  rollCorrection = kP * rollError - kD * rollErrorDerivative + kI * rollErrorIntegral;

  rollErrorPrev = rollError;
}

void getCorrectionPitch()
{
  pitchErrorDerivative = pitchError - pitchErrorPrev;
  pitchErrorIntegral += pitchError;
  if((kI*pitchErrorIntegral) > INTEGRAL_LIMIT)
    pitchErrorIntegral = INTEGRAL_LIMIT / kI;
    if((kI * pitchErrorIntegral) < (-1 * INTEGRAL_LIMIT))
    pitchErrorIntegral = -1 * INTEGRAL_LIMIT / kI;
  pitchCorrection = kP * pitchError - kD * pitchErrorDerivative + kI * pitchErrorIntegral;

  pitchErrorPrev = pitchError;
}


void getError()
{
  pitchError = kalAngleY - PITCH_SET_POINT;
  rollError = kalAngleX - ROLL_SET_POINT;

  pitchError = pitchError > PITCH_SET_POINT_BUFFER || pitchError < (-PITCH_SET_POINT_BUFFER) ? pitchError : 0;
  rollError = rollError > ROLL_SET_POINT_BUFFER || rollError < (-ROLL_SET_POINT_BUFFER) ? rollError : 0;

  pitchErrorIntegral = pitchError < PITCH_SET_POINT_BUFFER_INTEGRAL_TO_ZERO || pitchError > (-PITCH_SET_POINT_BUFFER_INTEGRAL_TO_ZERO) ? pitchErrorIntegral : 0;
  rollErrorIntegral = rollError < ROLL_SET_POINT_BUFFER_INTEGRAL_TO_ZERO || rollError > (-ROLL_SET_POINT_BUFFER_INTEGRAL_TO_ZERO) ? rollErrorIntegral : 0;
  
}//aage +ve right +ve

double constrainValue(double upLimit,double downLimit, double value)
{
     if(value > upLimit)
      value = upLimit;
     else if(value < downLimit)
      value = downLimit;
     return value;
}
void writePWMRoll()
{
  leftPWM = constrainValue(MAX_PWM, MIN_PWM, OPT - rollCorrection);
  rightPWM = constrainValue(MAX_PWM, MIN_PWM, OPT + rollCorrection);
  escLeft.writeMicroseconds(leftPWM);
  escRight.writeMicroseconds(rightPWM - 50);//******
}

void writePWMPitch()
{
  frontPWM = constrainValue(MAX_PWM, MIN_PWM, OPT + pitchCorrection);
  backPWM = constrainValue(MAX_PWM, MIN_PWM, OPT - pitchCorrection);
  escFront.writeMicroseconds(frontPWM);
  escBack.writeMicroseconds(backPWM + 30);
}



void setupESC(){
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

void printPWM()
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
  Serial.print (kalAngleY);
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
  Serial.println();
}

void printMPU()
{
  Serial.print("Pitch: ");
  Serial.print (kalAngleY);
  Serial.print("\t");
  
  Serial.print("Roll: ");
  Serial.print (kalAngleX);
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
  while(!client.available()){
    delay(1);
  }
 
  // Read the first line of the request
  String request = client.readStringUntil('\r');
  Serial.println(request);
  client.flush();
 
  // Match the request
 
  if (request.indexOf("/kPUp") != -1) {
  kP+= resolutionKP;
  } 
  if (request.indexOf("/kPDown") != -1){
  kP-= resolutionKP;
  }

  if (request.indexOf("/kPResUp") != -1) {
  resolutionKP *= 10;
  } 
  if (request.indexOf("/kPResDown") != -1){
  resolutionKP /= 10;
  }



  if (request.indexOf("/kIUp") != -1) {
  kI+= resolutionKI;
  } 
  if (request.indexOf("/kIDown") != -1){
  kI-= resolutionKI;
  }

  if (request.indexOf("/kIResUp") != -1) {
  resolutionKI *= 10;
  } 
  if (request.indexOf("/kIResDown") != -1){
  resolutionKI /= 10;
  }  
 


  if (request.indexOf("/kDUp") != -1) {
  kD += resolutionKD;
  } 
  if (request.indexOf("/kDDown") != -1){  kD -= resolutionKD;
  }

  if (request.indexOf("/kDResUp") != -1) {
  resolutionKD *= 10;
  } 
  if (request.indexOf("/kDResDown") != -1){
  resolutionKD /= 10;
  }  
 
   /*
   * float ROLL_SET_POINT = -1.5;
float PITCH_SET_POINT = -1.5;
float OPT = 1400; //the optimal speed was 1400
float INTEGRAL_LIMIT = 30;
   */

    if (request.indexOf("/ROLL_SET_POINTUp") != -1) {
  ROLL_SET_POINT += resolutionROLL_SET_POINT;
  } 
  if (request.indexOf("/ROLL_SET_POINTDown") != -1){  ROLL_SET_POINT -= resolutionROLL_SET_POINT;
  }

  if (request.indexOf("/ROLL_SET_POINTResUp") != -1) {
  resolutionROLL_SET_POINT *= 10;
  } 
  if (request.indexOf("/ROLL_SET_POINTResDown") != -1){
  resolutionROLL_SET_POINT /= 10;
  }  

     if (request.indexOf("/PITCH_SET_POINTUp") != -1) {
  PITCH_SET_POINT += resolutionPITCH_SET_POINT;
  } 
  if (request.indexOf("/PITCH_SET_POINTDown") != -1){  PITCH_SET_POINT -= resolutionPITCH_SET_POINT;
  }

  if (request.indexOf("/PITCH_SET_POINTResUp") != -1) {
  resolutionPITCH_SET_POINT *= 10;
  } 
  if (request.indexOf("/PITCH_SET_POINTResDown") != -1){
  resolutionPITCH_SET_POINT /= 10;
  }  

      if (request.indexOf("/OPTUp") != -1) {
  OPT += resolutionOPT;
  } 
  if (request.indexOf("/OPTDown") != -1){  OPT -= resolutionOPT;
  }

  if (request.indexOf("/OPTResUp") != -1) {
  resolutionOPT *= 10;
  } 
  if (request.indexOf("/OPTResDown") != -1){
  resolutionOPT /= 10;
  }  

   if (request.indexOf("/INTEGRAL_LIMITUp") != -1) {
  INTEGRAL_LIMIT += resolutionINTEGRAL_LIMIT;
  } 
  if (request.indexOf("/INTEGRAL_LIMITDown") != -1){  INTEGRAL_LIMIT -= resolutionINTEGRAL_LIMIT;
  }

  if (request.indexOf("/INTEGRAL_LIMITResUp") != -1) {
  resolutionINTEGRAL_LIMIT *= 10;
  } 
  if (request.indexOf("/INTEGRAL_LIMITResDown") != -1){
  resolutionINTEGRAL_LIMIT /= 10;
  }  

    if (request.indexOf("/setupMPU") != -1){
  setupMPU();
  }  

    if (request.indexOf("/setupESC") != -1){
  setupESC();
  }  
     if (request.indexOf("/resetSetPoints") != -1){
  ROLL_SET_POINT = kalAngleX;
  PITCH_SET_POINT = kalAngleY;
  }  
  
  // Return the response
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println(""); //  do not forget this one
  client.println("<!DOCTYPE HTML>");
  client.println("<html>");

  client.print("OPT = ");
  client.print(OPT);
  client.println("<form action = \"/OPTUp\">");
  client.println("<input type = \"submit\" value = \"Increase OPT\">");
  client.println("</form>");
  client.println("<form action = \"/OPTDown\">");
  client.println("<input type = \"submit\" value = \"Decrease OPT\">");
  client.println("</form>");
  client.println("<br><br>");

  client.print("Resolution for OPT = ");
  client.print(resolutionOPT);
  client.println("<form action = \"/OPTResUp\">");
  client.println("<input type = \"submit\" value = \"RESOLUTION for OPT x 10\">");
  client.println("</form>");
  client.println("<form action = \"/OPTResDown\">");
  client.println("<input type = \"submit\" value = \"RESOLUTION  for OPT / 10\">");
  client.println("</form>");
  client.println("<br><br><br>");
  
  client.print("kP = ");
  client.print(kP);
  client.println("<br>");
  client.println("<form action = \"/kPUp\">");
  client.println("<input type = \"submit\" value = \"Increase kP\">");
  client.println("</form>");
  client.println("<form action = \"/kPDown\">");
  client.println("<input type = \"submit\" value = \"Decrease kP\">");
  client.println("</form>");
  client.println("<br><br>");
  
  client.print("Resolution for kP = ");
  client.print(resolutionKP);
  client.println("<form action = \"/kPResUp\">");
  client.println("<input type = \"submit\" value = \"RESOLUTION for kP x 10\">");
  client.println("</form>");
  client.println("<form action = \"/kPResDown\">");
  client.println("<input type = \"submit\" value = \"RESOLUTION  for kP / 10\">");
  client.println("</form>");
  client.println("<br><br><br>");
  client.println("<br><br><br>");


 
  client.print("kI = ");
  client.print(kI);
  client.println("<form action = \"/kIUp\">");
  client.println("<input type = \"submit\" value = \"Increase kI\">");
  client.println("</form>");
  client.println("<form action = \"/kIDown\">");
  client.println("<input type = \"submit\" value = \"Decrease kI\">");
  client.println("</form>");
  client.println("<br><br>");
  
  client.print("Resolution for kI = ");
  client.print(resolutionKI);
  client.println("<form action = \"/kIResUp\">");
  client.println("<input type = \"submit\" value = \"RESOLUTION for kI x 10\">");
  client.println("</form>");
  client.println("<form action = \"/kIResDown\">");
  client.println("<input type = \"submit\" value = \"RESOLUTION  for kI / 10\">");
  client.println("</form>");
  client.println("<br><br><br>");
  client.println("<br><br><br>");



  client.print("kD = ");
  client.print(kD);
  client.println("<form action = \"/kDUp\">");
  client.println("<input type = \"submit\" value = \"Increase kD\">");
  client.println("</form>");
  client.println("<form action = \"/kDDown\">");
  client.println("<input type = \"submit\" value = \"Decrease kD\">");
  client.println("</form>");
  client.println("<br><br>");

  client.print("Resolution for kD = ");
  client.print(resolutionKD);
  client.println("<form action = \"/kDResUp\">");
  client.println("<input type = \"submit\" value = \"RESOLUTION for kD x 10\">");
  client.println("</form>");
  client.println("<form action = \"/kDResDown\">");
  client.println("<input type = \"submit\" value = \"RESOLUTION  for kD / 10\">");
  client.println("</form>");
  client.println("<br><br><br>");
  /*
   * float ROLL_SET_POINT = -1.5;
float PITCH_SET_POINT = -1.5;
float OPT = 1400; //the optimal speed was 1400
float INTEGRAL_LIMIT = 30;
   */

  client.print("ROLL_SET_POINT = ");
  client.print(ROLL_SET_POINT);
  client.println("<form action = \"/ROLL_SET_POINTUp\">");
  client.println("<input type = \"submit\" value = \"Increase ROLL_SET_POINT\">");
  client.println("</form>");
  client.println("<form action = \"/ROLL_SET_POINTDown\">");
  client.println("<input type = \"submit\" value = \"Decrease ROLL_SET_POINT\">");
  client.println("</form>");
  client.println("<br><br>");

  client.print("Resolution for ROLL_SET_POINT = ");
  client.print(resolutionROLL_SET_POINT);
  client.println("<form action = \"/ROLL_SET_POINTResUp\">");
  client.println("<input type = \"submit\" value = \"RESOLUTION for ROLL_SET_POINT x 10\">");
  client.println("</form>");
  client.println("<form action = \"/ROLL_SET_POINTResDown\">");
  client.println("<input type = \"submit\" value = \"RESOLUTION  for ROLL_SET_POINT / 10\">");
  client.println("</form>");
  client.println("<br><br><br>");

    client.print("PITCH_SET_POINT = ");
  client.print(PITCH_SET_POINT);
  client.println("<form action = \"/PITCH_SET_POINTUp\">");
  client.println("<input type = \"submit\" value = \"Increase PITCH_SET_POINT\">");
  client.println("</form>");
  client.println("<form action = \"/PITCH_SET_POINTDown\">");
  client.println("<input type = \"submit\" value = \"Decrease PITCH_SET_POINT\">");
  client.println("</form>");
  client.println("<br><br>");

  client.print("Resolution for PITCH_SET_POINT = ");
  client.print(resolutionPITCH_SET_POINT);
  client.println("<form action = \"/PITCH_SET_POINTResUp\">");
  client.println("<input type = \"submit\" value = \"RESOLUTION for PITCH_SET_POINT x 10\">");
  client.println("</form>");
  client.println("<form action = \"/PITCH_SET_POINTResDown\">");
  client.println("<input type = \"submit\" value = \"RESOLUTION  for PITCH_SET_POINT / 10\">");
  client.println("</form>");
  client.println("<br><br><br>");


    client.print("INTEGRAL_LIMIT = ");
  client.print(INTEGRAL_LIMIT);
  client.println("<form action = \"/INTEGRAL_LIMITUp\">");
  client.println("<input type = \"submit\" value = \"Increase INTEGRAL_LIMIT\">");
  client.println("</form>");
  client.println("<form action = \"/INTEGRAL_LIMITDown\">");
  client.println("<input type = \"submit\" value = \"Decrease INTEGRAL_LIMIT\">");
  client.println("</form>");
  client.println("<br><br>");

  client.print("Resolution for INTEGRAL_LIMIT = ");
  client.print(resolutionINTEGRAL_LIMIT);
  client.println("<form action = \"/INTEGRAL_LIMITResUp\">");
  client.println("<input type = \"submit\" value = \"RESOLUTION for INTEGRAL_LIMIT x 10\">");
  client.println("</form>");
  client.println("<form action = \"/INTEGRAL_LIMITResDown\">");
  client.println("<input type = \"submit\" value = \"RESOLUTION  for INTEGRAL_LIMIT / 10\">");
  client.println("</form>");
  client.println("<br><br><br>");

//MISCELLANEOUS OPERATIONS :
  client.println("<form action = \"/setupMPU\">");
  client.println("<input type = \"submit\" value = \"SETUP MPU\">");
  client.println("</form>");
  client.println("<br>");
  client.println("<form action = \"/setupESC\">");
  client.println("<input type = \"submit\" value = \"SETUP ESC of BLDC Motors\">");
  client.println("</form>");
  client.println("<form action = \"/resetSetPoints\">");
  client.println("<input type = \"submit\" value = \"CALIBRATE SET POINTS\">");
  client.println("</form>");
  client.println("<br>");
  client.println("<br><br><br>");
   
  client.println("</html>");
  
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
  Serial.print (OPT);
  Serial.print("\t");

  
  Serial.print("INTEGRAL LIMIT: ");
  Serial.print (INTEGRAL_LIMIT);
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


void setup() {
  Serial.begin(9600);
  wdt_enable(WDT_INTERVAL);
  setupMPU();
  wdt_disable();
//  setupCommunications();
//  setupESC();
  Serial.println("ESCs Initialized!");
}


//-15 to 12 pitch and roll
void loop() {
   wdt_enable(WDT_INTERVAL);
  updateAngles();
  wdt_disable();

  getError();
  getCorrectionRoll();
  writePWMRoll();
  
  getCorrectionPitch();
  writePWMPitch();

//updateParameters();

  /* Print Data */
//  printPWM();
printMPU();

//delay(5000);
//printParameters();
//  printData();

}

