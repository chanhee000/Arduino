////////////////sonar///////////////////////////


#define TRIG1 3  // 초음파 센서 1번 Trig 핀 번호
#define ECHO1 4  // 초음파 센서 1번 Echo 핀 번호

#define TRIG2 5  // 초음파 센서 2번 Trig 핀 번호
#define ECHO2 6  // 초음파 센서 2번 Echo 핀 번호
#define TRIG3 7  // 초음파 센서 3번 Trig 핀 번호
#define ECHO3 8  // 초음파 센서 3번 Echo 핀 번호
//////////////////L298N//////////////////////////////
#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13

void setup() {
  // put your setup code here, to run once:
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);

  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);

  Serial.begin(115200); // 통신 속도를 115200으로 정의함
}
long sonar1(void)
{
  long duration, distance;
  digitalWrite(TRIG1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG1,LOW);
  duration = pulseIn(ECHO1, HIGH);
  distance = ( (float) (340*duration)/1000)/2;
  return distance;
}
long sonar2(void)
{
  long duration, distance;
  digitalWrite(TRIG2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG2,LOW);
  duration = pulseIn(ECHO1, HIGH);
  distance = ( (float) (340*duration)/1000)/2;
  return distance;
}
long sonar3(void)
{
  long duration, distance;
  digitalWrite(TRIG3, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG3,LOW);
  duration = pulseIn(ECHO1, HIGH);
  distance = ( (float) (340*duration)/1000)/2;
  return distance;
}
void motor_A_control(int direction_a, int motor_speed_a) //모터 A의 방향(direction)과 속도(speed)제어
{
   if(direction_a == HIGH)
   {
     digitalWrite(IN1,HIGH);       //모터의 방향제어
     digitalWrite(IN2,LOW);
     analogWrite(ENA,motor_speed_a);  //모터의 속도 제어
     }
   else
   {
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(ENA,motor_speed_a);
   }
}
void motor_B_control(int direction_b, int motor_speed_b) //모터 B의 방향(direction)과 속도(speed)제어
{
   if(direction_b == HIGH)
   {
     digitalWrite(IN3,LOW);           //모터의 방향제어
     digitalWrite(IN4,HIGH);
     analogWrite(ENB,motor_speed_b); //모터의 속도 제어
     }
   else
   {
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
    analogWrite(ENB,motor_speed_b);
   }
}

void loop() {
  Serial.print("Distance1; ");
  Serial.println(sonar1()  ); 
  Serial.print("Distance2;");
  Serial.println( sonar2() );
  Serial.print("Distance3;");
  Serial.println( sonar3() );

  motor_A_control(HIGH,100);
  motor_B_control(HIGH,100);
 
  delay(500);
}
