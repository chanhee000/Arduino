
const byte outPin = 13; // Output pin: digital pin 4(D4)
//const byte interruptPin1 = 2; // Interrupt pin: D2
//const byte interruptPin2 = 3; // Interrupt pin: D2
const byte encoder1_A = 2;
const byte encoder1_B = 3;
const byte encoder2_A = 18;
const byte encoder2_B = 19;


const byte resetPinm = 5;
volatile byte state = 0;
//unsigned long cnt1 = 0; //추가
//unsigned long cnt2 = 0; //추가
long cnt1 = 0; //추가
long cnt2 = 0; //추가

void setup()
{
 pinMode(outPin, OUTPUT); // Output mode
 //pinMode(interruptPin1, INPUT_PULLUP); // Input mode, pull-up
 //pinMode(interruptPin2, INPUT_PULLUP); // Input mode, pull-up
 pinMode(encoder1_A,INPUT_PULLUP);
 pinMode(encoder1_B,INPUT_PULLUP);
 pinMode(encoder2_A,INPUT_PULLUP);
 pinMode(encoder2_B,INPUT_PULLUP);
 pinMode(resetPinm, INPUT);
 //attachInterrupt(digitalPinToInterrupt(interruptPin1), intfunc1, RISING); // Enable interrupt
 //attachInterrupt(digitalPinToInterrupt(interruptPin2), intfunc2, RISING); // Enable interrupt
 attachInterrupt(digitalPinToInterrupt(encoder1_A), intfunc1, RISING); // Enable interrupt
 attachInterrupt(digitalPinToInterrupt(encoder2_A), intfunc2, RISING); // Enable interrupt
 Serial.begin(115200);
}

void intfunc1()
{
  if (digitalRead(encoder1_B) == HIGH)
  {
    cnt1++;
  }
  else
  {
    cnt1--;
  }
}

void intfunc2()
{
  if (digitalRead(encoder2_B) == LOW)
  {
    cnt2++;
  }
  else
  {
    cnt2--;
  }
}


/*void intfunc1()
{
  if(encoder1_A == HIGH)
  {
    cnt1++;
  }
  else
  {
    cnt1++;
  }
}
void intfunc2()
{
  if(encoder1_B == HIGH)
  {
    cnt2++;
  }
  else
  {
    cnt2++;
  }
}*/
/*void intfunc1() // Interrupt function
{
 cnt1++;
 if (state == 0) // If D4 output is low
 {
  digitalWrite(outPin, HIGH);
  state = 1;
 }
 else
 {
  digitalWrite(outPin, LOW);
  state = 0;
 }
}

void intfunc2() // Interrupt function
{
 cnt2++;
 if (state == 0) // If D4 output is low
 {
  digitalWrite(outPin, HIGH);
  state = 1;
 }
 else
 {
  digitalWrite(outPin, LOW);
  state = 0;
 }
}*/

void loop()
{
 Serial.print(cnt1);
 Serial.print("     ");
 Serial.println(cnt2);
 /*if(digitalRead(resetPinm) == HIGH)
 {
  cnt1 = 0;
  cnt2 = 0;
 }*/

}
