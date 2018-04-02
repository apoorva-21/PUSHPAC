//
////#define CH_1 D2
////#define CH_2 D6
////#define CH_3 D13//D7
////#define CH_4 D8
////#define CH_5 D10
////#define CH_6 D11

//#define CH_1 D1
//#define CH_2 D0
#define CH_3 D6//D7
#define CH_4 D7

int ch1, ch2;
int ch3 = 2;
int ch4 = 3;
//ch5, ch6;
int channelOutputs[4];
int channelStartTime[4];
int channelBuffers[4];

void readChannel(int channelNo, int channelPin)
{
  if(digitalRead(channelPin) == HIGH)
    channelStartTime[channelNo] = micros();

  else channelBuffers[channelNo] = micros() - channelStartTime[channelNo];
  
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
void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(CH_3, INPUT);
  pinMode(CH_4, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(CH_3), readChannel3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_4), readChannel4, CHANGE);
  
  Serial.println("SETUP");

}

void loop() {
  // put your main code here, to run repeatedly:
  int loopTimer = millis();
  readRC();
//  ch1 = pulseIn(CH_1, HIGH);
//  Serial.print("  ch1 : ");
//  Serial.print(ch1);
//
//  ch2 = pulseIn(CH_2, HIGH);
//  Serial.print("  ch2 : ");
//  Serial.print(ch2);

//  ch3 = pulseIn(CH_3, HIGH);
  Serial.print("  ch3 : ");
  Serial.print(channelOutputs[ch3]);

//  ch4 = pulseIn(CH_4, HIGH);
  Serial.print("  ch4 : ");
  Serial.print(channelOutputs[ch4]);

  Serial.print("Loop Time : ");
  Serial.println(millis() - loopTimer);
  
}
//
///*
// * CH_4::D2::LeftStickHorizontal(1145-1513-1912)
// * CH_3::D13::LeftStickVertical(U1946-1545-D1195)
// * CH_2::D6::RightStickVertical(U1179-1514-D1835)
// * CH_1::D2::RightStickHorizontal(L1140-1521-R1904)
// */

//#include <attachInterrupt.h>

//#define SERIAL_PORT_SPEED 115200
//#define RC_NUM_CHANNELS  4
//
//#define RC_CH1  0
//#define RC_CH2  1
//#define RC_CH3  2
//#define RC_CH4  3
//
//#define RC_CH1_INPUT  D1
//#define RC_CH2_INPUT  D0
//#define RC_CH3_INPUT  D6
//#define RC_CH4_INPUT  D7
//
//uint16_t rc_values[RC_NUM_CHANNELS];
//uint32_t rc_start[RC_NUM_CHANNELS];
//volatile uint16_t rc_shared[RC_NUM_CHANNELS];
//
//void rc_read_values() {
//  noInterrupts();
//  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
//  interrupts();
//}
//
//void calc_input(uint8_t channel, uint8_t input_pin) {
//  if (digitalRead(input_pin) == HIGH) {
//    rc_start[channel] = micros();
//  } else {
//    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
//    rc_shared[channel] = rc_compare;
//  }
//}
//
//void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
//void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
//void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
//void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }
//
//void setup() {
//  Serial.begin(SERIAL_PORT_SPEED);
//
//  pinMode(RC_CH1_INPUT, INPUT);
//  pinMode(RC_CH2_INPUT, INPUT);
//  pinMode(RC_CH3_INPUT, INPUT);
//  pinMode(RC_CH4_INPUT, INPUT);
//
//  attachInterrupt(digitalPinToInterrupt(RC_CH1_INPUT), calc_ch1, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(RC_CH2_INPUT), calc_ch2, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(RC_CH3_INPUT), calc_ch3, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(RC_CH4_INPUT), calc_ch4, CHANGE);
//  Serial.print("LOLLLOLOLOLOLOL");
//}
//
//void loop() {
//  rc_read_values();
//
//  Serial.print("CH1:"); Serial.print(rc_values[RC_CH1]); Serial.print("\t");
//  Serial.print("CH2:"); Serial.print(rc_values[RC_CH2]); Serial.print("\t");
//  Serial.print("CH3:"); Serial.print(rc_values[RC_CH3]); Serial.print("\t");
//  Serial.print("CH4:"); Serial.println(rc_values[RC_CH4]);
//
//  delay(200);
//}
