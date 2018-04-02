
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

int ch1, ch2, ch3, ch4, ch5, ch6;
void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200);
//  pinMode(CH_1, INPUT);
//  pinMode(CH_2, INPUT);
  pinMode(CH_3, INPUT);
  pinMode(CH_4, INPUT);
//  pinMode(CH_5, INPUT);
//  pinMode(CH_6, INPUT);
  Serial.println("SETUP");
  

}

void loop() {
  // put your main code here, to run repeatedly:
  int loopTimer = millis();
  
//  ch1 = pulseIn(CH_1, HIGH);
//  Serial.print("  ch1 : ");
//  Serial.print(ch1);
//
//  ch2 = pulseIn(CH_2, HIGH);
//  Serial.print("  ch2 : ");
//  Serial.print(ch2);

  ch3 = pulseIn(CH_3, HIGH);
  Serial.print("  ch3 : ");
  Serial.print(ch3);

  ch4 = pulseIn(CH_4, HIGH);
  Serial.print("  ch4 : ");
  Serial.print(ch4);

  Serial.print("Loop Time : ");
  Serial.println(millis() - loopTimer);
  
}

/*
 * CH_4::D2::LeftStickHorizontal(1145-1513-1912)
 * CH_3::D13::LeftStickVertical(U1946-1545-D1195)
 * CH_2::D6::RightStickVertical(U1179-1514-D1835)
 * CH_1::D2::RightStickHorizontal(L1140-1521-R1904)
 */
