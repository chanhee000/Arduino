#include <MPU6050_tockn.h>
#include <MsTimer2.h>

#define RAD2DEG(x)    (x*180.0/3.14159)
#define DEG2RAD(x)    (x*3.14159/180.0)
#define wheel_track  0.144     //m단위로 구하기    0.1인경우 10cm


#define A0pin A0
#define SIpin  22
#define CLKpin  23
#define NPIXELS 128

#define BTSerial Serial1
#define BT_BAUDRATE 9600
#define RxD 17
#define TxD 16

#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13
#define NPIXELS 128

double kp_vision = 0.1;
double kd_vision = 0.3;
double ki_vision = 0.0;
double error = 0.0;
double error_old = 0.0;
double Target = NPIXELS/2;
/////////////imu////////////////

#include <Wire.h>
MPU6050 mpu6050(Wire);


byte Pixel[NPIXELS];
byte LineSensor_threshold_Data[NPIXELS];
int LineSensor_Data[NPIXELS];
int LineSensor_Data_Adaption[NPIXELS];
int MAX_LineSensor_Data[NPIXELS];
int MIN_LineSensor_Data[NPIXELS];
int flag_line_adapation;

#define FASTADC 1

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void setup()
{
  // put your setup code here, to run once:
  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data[i] = 0;
    LineSensor_Data_Adaption[i] = 0;
    MAX_LineSensor_Data[i] = 1023;  // 0
    MIN_LineSensor_Data[i] = 0;     // 1023
  }
  pinMode(SIpin, OUTPUT);
  pinMode(CLKpin, OUTPUT);
  pinMode(A0pin, INPUT);

  digitalWrite(SIpin, LOW);   //IDLE state
  digitalWrite(CLKpin, LOW);  //IDLE state

 BTSerial.begin(BT_BAUDRATE);
#if FASTADC
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif

  flag_line_adapation = 0;
  Serial.begin(115200);
  Serial.println("TSL1401");
}
void threshold_line_image(int threshold_value)
{
 for (int i = 0; i < NPIXELS; i++)
 {
  if(Pixel[i] >= threshold_value)
  {
    LineSensor_threshold_Data[i] = 255;
  }
  else
  {
   LineSensor_threshold_Data[i] = 0; 

  }
 }
}
void read_line_camera(void)
{
  delay(1);
  digitalWrite (CLKpin, LOW);
  digitalWrite (SIpin, HIGH);
  digitalWrite (CLKpin, HIGH);
  digitalWrite (SIpin, LOW);
  delayMicroseconds(1);

  for (int i = 0; i < NPIXELS; i++)
  {
    Pixel[i] = analogRead (A0pin)/4; // 8-bit is enough
    digitalWrite (CLKpin, LOW);
    delayMicroseconds(1);
    digitalWrite (CLKpin, HIGH);
  }
  digitalWrite (CLKpin, LOW);
}

double line_centroid(void)
{
  double centroid = 0.0;
  double mass_sum = 0;

  for(int i = 0; i < NPIXELS; i++)
  {
    mass_sum += LineSensor_threshold_Data[i];
    centroid += LineSensor_threshold_Data[i] * i;
  }
  centroid = centroid / mass_sum;

  return centroid;
}
void motor_R_control(int motor_speed_R) // 모터 A의 속도(speed)제어
{
  if (motor_speed_R >= 0)
  {
    digitalWrite(IN1, LOW);         //모터의 방향 제어
    digitalWrite(IN2, HIGH);
    if (motor_speed_R >= 255) motor_speed_R = 255;
    analogWrite(ENA, motor_speed_R); //모터의 속도 제어
  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    if (motor_speed_R <= -255) motor_speed_R = -255;
    analogWrite(ENA, -motor_speed_R);
  }
}

void motor_L_control(int motor_speed_L) // 모터 A의 속도(speed)제어
{
  if (motor_speed_L >= 0)
  {
    digitalWrite(IN3, HIGH);         //모터의 방향 제어
    digitalWrite(IN4, LOW);
    if (motor_speed_L >= 255) motor_speed_L = 255;
    analogWrite(ENB, motor_speed_L); //모터의 속도 제어
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    if (motor_speed_L <= -255) motor_speed_L = -255;
    analogWrite(ENB, -motor_speed_L);
  }
}


double PID_control(double line_center)
{
  /*
  double kp_vision = 0.1;
  double kd_vision = 0.3;
  double ki_vision = 0.0;
  double error = 0.0;
  double error_old = 0.0;
  double Target = NPIXELS/2;
  */
  int pwm_value = 0;
  double error_d = 0;
  error = Target - line_center;
  error_d = error - error_old;
  pwm_value = int(kp_vision*error + kd_vision * error_old);
  if(pwm_value >= 200)  pwm_value = 200;
  if(pwm_value <= -200) pwm_value = -200;
  error_old = error;
  return pwm_value;
}

void vision_line_control(int base_speed, double l_c)
{
  int pwm_control_value = PID_control(l_c);
  motor_L_control(base_speed + pwm_control_value);     //상황에 따라 + - 변경이된다. 변경될 경우 둘 다 반대로 변경
  motor_R_control(base_speed - pwm_control_value);
  //motor_control_left(base_speed + pwm_control_value);     //상황에 따라 + - 변경이된다. 변경될 경우 둘 다 반대로 변경
 //motor_control_right(base_speed - pwm_control_value);
}

void loop()
{
  // put your main code here, to run repeatedly:
  double cx = 0;
  int cx_int;
  read_line_camera();
  threshold_line_image(50);
  cx = line_centroid();
  vision_line_control(50,cx);
   for (int i = 0; i < NPIXELS; i++)
   {
    Serial.print(LineSensor_threshold_Data[i] + 1);
    Serial.print(" ");
    //Serial.println((byte)Pixel[i] + 1);
   }
   cx_int = (int)cx;
   Serial.print(cx);
   Serial.print(" ");
   Serial.println(" ");
   //Serial1.println(cx);
   if (BTSerial.available())
  {
    char command = BTSerial.read();
    if (command == '1')
    {
      // Move forward
      motor_L_control(200);
      motor_R_control(200);
    }
    else if (command == '2')
    {
      // Move backward
      motor_L_control(-200);
      motor_R_control(-200);
    }
    else if (command == '3')
    {
      // Turn right
      motor_L_control(200);
      motor_R_control(-200);
    }
    else if (command == '4')
    {
      // Turn left
      motor_L_control(-200);
      motor_R_control(200);
    }
    else
    {
      // Stop
      motor_L_control(0);
      motor_R_control(0);
    }
  }
   
}
