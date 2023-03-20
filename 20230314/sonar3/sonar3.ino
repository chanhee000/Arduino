#define TRIG1 3  // 초음파 센서 1번 Trig 핀 번호
#define ECHO1 4  // 초음파 센서 1번 Echo 핀 번호

#define TRIG2 5  // 초음파 센서 2번 Trig 핀 번호
#define ECHO2 6  // 초음파 센서 2번 Echo 핀 번호
#define TRIG3 7  // 초음파 센서 3번 Trig 핀 번호
#define ECHO3 8  // 초음파 센서 3번 Echo 핀 번호
void setup() {
  // put your setup code here, to run once:
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);

  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);

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
void loop() {
  // put your main code here, to run repeatedly:
  //Serial.print("Duration: ");
  //Serial.println(duration);
  Serial.print("Distance1; ");
  Serial.println(sonar1()  ); 
  Serial.print("Distance2;");
  Serial.println( sonar2() );
  Serial.print("Distance3;");
  Serial.println( sonar3() );
 
  delay(500);
}
