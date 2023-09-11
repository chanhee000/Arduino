#include <MsTimer2.h>

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

long cnt1_old = 0; //추가
long cnt2_old = 0; //추가

double pulse_to_distance_left  = 0.2/939.8;
double pulse_to_distance_right = 0.2/503.1;

const double odom_left  = 0;
const double odom_right = 0;

double heading(double x,double y)
{
  double head = atan2(y,x); //Slope Y,Slope X
  return head;
}

void msTimer2_ISR()
{
  double odom_left_delta  = 0.0;
  double odom_right_delta = 0.0;
  double theta_delta  = 0.0;
  double wheel_track = 0.15;
  
  odom_left_delta   = (cnt1 - cnt1_old) * pulse_to_distance_left;
  odom_right_delta  = (cnt2 - cnt2_old) * pulse_to_distance_right;

  theta_delta = heading(wheel_track,(odom_left_delta - odom_right_delta));
  
  Serial.print(odom_left_delta);
  Serial.print("    ");
  Serial.println(odom_right_delta);
  Serial.print("    ");
  theta_delta = theta_delta * 180.8/3.14150;  
  Serial.println(theta_delta);
  
  cnt1_old = cnt1;
  cnt2_old = cnt2;
}

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
 MsTimer2::set(100,msTimer2_ISR);
 MsTimer2::start();
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
 delay(1000);
 /*if(digitalRead(resetPinm) == HIGH)
 {
  cnt1 = 0;
  cnt2 = 0;
 }*/

}
