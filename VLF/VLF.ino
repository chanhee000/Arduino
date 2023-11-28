#define LINE_PRINT   1
/////////////////////////// Motor 관련 ///////////////////////
#define encodPinA1   2
#define encodPinB1   3
#define MOTOR_DIR    4
#define MOTOR_PWM    5

/////////////////////// Line Camera 관련 ///////////////////////
#define A0pin        A0
#define SIpin        13
#define CLKpin       12

#define threshold_valus 65
#define NPIXELS         128
#define OFFSET          4


byte Pixel[NPIXELS];

int LineSensor_Data[NPIXELS];
int LineSensor_Data_Adaption[NPIXELS];   // 최대최소 구분
int MAX_LineSensor_Data[NPIXELS];       // Max value of sensor
int MIN_LineSensor_Data[NPIXELS];       // Min value of sensor

/////////////////////// Steering 관련 ///////////////////////
#include <Servo.h>
#define RC_SERVO_PIN 8
#define LEFT_STEER_ANGLE    -45
#define RIGHT_STEER_ANGLE    45
#define NEURAL_ANGLE         81.5   //81.5
#define NEURAL_ANGLE_offset  0

///////////////////////// Sonar 관련 /////////////////////
#include <NewPing.h>

#define MAX_DISTANCE  2000
float UltrasonicSensorData[3];

////////////////////////// PID 제어 관련 //////////////////////
float Kp = 0.9;     //130&&150 1.2 1.5
float Ki = 0;
float Kd = 0.9;

float Kp_maze  = 0.1;  //0.1
float Ki_maze  = 0;
float Kd_maze  = 0.1;
float Pi = 0;

int mode = 0;
int mission_flag = 0;

int odom = 0;

int Steering_Angle = 0;
int steer_data = 0;

int Line_Center       = NPIXELS / 2;
int Line_L_Center     = 10;
int Line_R_Center     = NPIXELS - 10;

int Line_L_Center_old = 10;
int Line_R_Center_old = NPIXELS - 10;


float error    = 0;
float error_s  = 0;
float error_d  = 0;

float error_old = 0;
float error_maze = 0;
float error_maze_old = 0;
float error_maze_d = 0;

float sonar_data = 0;

NewPing sonar[3] =
{
  NewPing(53, 52, MAX_DISTANCE),
  NewPing(51, 50, MAX_DISTANCE),
  NewPing(49, 48, MAX_DISTANCE)
};

void read_front_ultrasonic_sensor(void)
{
  UltrasonicSensorData[0] = sonar[0].ping_cm() * 10.0; // 전방
}

void read_ultrasonic_sensor(void)
{
  UltrasonicSensorData[0] = sonar[0].ping_cm() * 10.0; // 전방
  UltrasonicSensorData[1] = sonar[1].ping_cm() * 10.0; // 왼쪽
  UltrasonicSensorData[2] = sonar[2].ping_cm() * 10.0; // 오른쪽

  if (UltrasonicSensorData[0] == 0)    UltrasonicSensorData[0] = MAX_DISTANCE;
  if (UltrasonicSensorData[1] == 0)    UltrasonicSensorData[1] = MAX_DISTANCE;
  if (UltrasonicSensorData[2] == 0)    UltrasonicSensorData[2] = MAX_DISTANCE;


}



void Robot_Mode_Define(void)
{
  read_ultrasonic_sensor();

  //양쪽에 벽이 있는 경우
  if ((UltrasonicSensorData[1] <= 600) && (UltrasonicSensorData[2] <= 600)) // 600 600
  {
    mode = 1;
  }

  else if ((UltrasonicSensorData[1] <= 600) && (UltrasonicSensorData[2] >= 1000))
  {
    mode = 2;
  }
  if (UltrasonicSensorData[0] >= 1200)
  {
    mode = 3;
  }

  /*
    Serial.print("mode: ");
    Serial.print(mode);
    Serial.println(" ");*/

}

void ultrasonic_sensor_serial_print(void)
{
  read_ultrasonic_sensor();

  Serial.print("F_sonar: ");
  Serial.print(UltrasonicSensorData[0]);
  Serial.print(" ");

  Serial.print("L_sonar: ");
  Serial.print(UltrasonicSensorData[1]);
  Serial.print(" ");

  Serial.print("R_sonar: ");
  Serial.print(UltrasonicSensorData[2]);
  Serial.println(" ");

  //Serial.print("sonar_error: ");
  //Serial.print(UltrasonicSensorData[1] - UltrasonicSensorData[2]);
  //Serial.print(" ");
}



#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

Servo Steeringservo;

int steering_control(int Steering_Angle)
{
  // 서보 angle값 제한
  if (Steering_Angle <= LEFT_STEER_ANGLE)  Steering_Angle = LEFT_STEER_ANGLE;
  if (Steering_Angle >= RIGHT_STEER_ANGLE) Steering_Angle = RIGHT_STEER_ANGLE;

  Steeringservo.write(Steering_Angle + NEURAL_ANGLE + NEURAL_ANGLE_offset);
}

void steering_control_maze(int Steering_Angle)
{
  Steeringservo.write(Steering_Angle + NEURAL_ANGLE + NEURAL_ANGLE_offset);    // 25;   32
}

void steering_control_maze_1(int Steering_Angle)
{
  Steeringservo.write(Steering_Angle  + NEURAL_ANGLE + NEURAL_ANGLE_offset);    // 25;   /1226
}

void steering_control_0(void)
{
  Steeringservo.write(NEURAL_ANGLE + NEURAL_ANGLE_offset);    // 25;   /1226
}

volatile long encoderPos = 0;
void reset_encoder(void)
{
  encoderPos = 0;
}

void encoderB()
{
  delayMicroseconds(2);
  if (digitalRead(encodPinB1) == LOW) encoderPos++;
  else                                encoderPos--;
}

void encoder_pulse(void)
{

}

void encoderPos_serial_print(void)
{
  Serial.print("encoderPos: ");
  Serial.print(encoderPos);
  Serial.println("  ");
}
void read_lane_sensor(void)
{
  int i;
  delayMicroseconds (1);
  delay(10);

  digitalWrite (CLKpin, LOW);
  digitalWrite (SIpin, HIGH);
  digitalWrite (CLKpin, HIGH);
  digitalWrite (SIpin, LOW);

  delayMicroseconds (1);

  for (i = 0; i < NPIXELS; i++)
  {
    Pixel[i] = analogRead (A0pin);
    digitalWrite (CLKpin, LOW);
    delayMicroseconds (1);
    digitalWrite (CLKpin, HIGH);
  }
}

void lane_data_serial_print(void)
{
  if (LINE_PRINT != 1)  return;

  for (int i = 0; i < NPIXELS; i++)
  {
    //Serial.println(LineSensor_Data_Adaption[i]);
    Serial.println((byte)Pixel[i]);
    Serial.print(" ");
  }
  //Serial.print(Line_L_Center); //노랑
  //Serial.print(" ");
  //Serial.print(Line_R_Center); //파랑
  //Serial.print(" ");
  //Serial.print(Line_Center);   //빨강
  //Serial.print(" ");
}

void threshold(void)
{
  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    if (Pixel[i] >= threshold_valus)
    {
      LineSensor_Data_Adaption[i] = 255;
    }
    else
    {
      LineSensor_Data_Adaption[i] = 0;
    }
  }
}

void find_lane_center(void)
{
  int i;
  long sum = 0;
  long x_sum = 0;
  int distance_L = 0; int distance_R = 0;

  for (i = 0; i < NPIXELS; i++)
  {
    sum += LineSensor_Data_Adaption[i];
    x_sum += (LineSensor_Data_Adaption[i]) * i;
  }
  Line_Center = (x_sum / sum);

  if (LineSensor_Data_Adaption[Line_Center] != 255) // Line이 2개 검출 될때
  {
    for (i = 0, sum = 0, x_sum = 0; i < Line_Center; i++)
    {
      sum  += LineSensor_Data_Adaption[i];
      x_sum += LineSensor_Data_Adaption[i] * i;
    }

    if (sum !=  0) Line_L_Center = (x_sum / sum);

    for (i = Line_Center, sum = 0, x_sum = 0; i < NPIXELS; i++)
    {
      sum += LineSensor_Data_Adaption[i];
      x_sum += LineSensor_Data_Adaption[i] * i;
    }

    if (sum !=  0) Line_R_Center = (x_sum / sum);

    Line_Center = (Line_R_Center + Line_L_Center) / 2;
  }
  else
  {
    distance_L = abs(Line_Center - Line_L_Center_old);
    distance_R = abs(Line_Center - Line_R_Center_old);
    if (distance_L < distance_R)
    {
      Line_L_Center = Line_Center;
      Line_R_Center = Line_L_Center + 22;
    }

    if (distance_L > distance_R)
    {
      Line_R_Center = Line_Center;
      Line_L_Center = Line_R_Center - 22;
    }

    Line_Center = (Line_R_Center + Line_L_Center) / 2;
  }

  Line_L_Center_old = Line_L_Center;
  Line_R_Center_old = Line_R_Center;

}

void PID_lane_control()
{

  error   = Line_Center - NPIXELS / 2 + OFFSET ; // 카메라 보정 방향 반대시 전체에 -1 곱하기
  error_d = error - error_old;
  error_s = (error_s >= 5) ?   5 : error_s;
  error_s = (error_s <= -5) ? -5 : error_s;

  Steering_Angle = Kp * error + Kd * error_d + Ki * error_s;

  if (fabs(error) <= 1)
  {
    error_s = 0;
  }
  steering_control(Steering_Angle);
  /*
    Serial.print(Line_Center);
    Serial.print("   ");
    Serial.print("error: ");
    Serial.print(error);
    Serial.print("   ");
    Serial.print("error_old: ");
    Serial.print(error_old);
    Serial.print("   ");
    Serial.print("error_d: ");
    Serial.print(error_d);
    Serial.print("   ");
    Serial.print("Kp * error: ");
    Serial.print(Kp * error);
    Serial.print("   ");
    Serial.print("Kd * error_d: ");
    Serial.print(Kd * error_d);
    Serial.print("   ");
    Serial.print(Steering_Angle);
    Serial.println("   ");
  */
  error_old = error;
}

void lane_control(void)
{
  read_lane_sensor();
  threshold();
  lane_data_serial_print();
  find_lane_center();
  PID_lane_control();
}

void motor_control(int speed)
{
  if (speed >= 0) {
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM, speed);
  }
  else {
    digitalWrite(MOTOR_DIR, LOW);
    analogWrite(MOTOR_PWM, -speed);
  }
}

void interrupt_setup(void)
{
  pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);                  // quadrature encoder input B
  attachInterrupt(0, encoderB, FALLING);               // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // To prevent Motor Noise
}

float PID_Control_wall(float error)
{
  float PIDvalue = 0;
  float error_maze_d = 0;
  error_maze = error;
  Pi = Pi + error_maze;
  error_maze_d = error_maze - error_maze_old;

  PIDvalue = (Kp_maze * error_maze) + (Kd_maze * error_maze_d);

  serial_print_control_wall();

  error_maze_old = error_maze;

  return PIDvalue;
}


void serial_print_control_wall(void)
{
  /*
    Serial.print("error: ");
    Serial.print(error_maze);
    Serial.print("   ");
    Serial.print("error_old: ");
    Serial.print(error_maze_old);
    Serial.print("   ");
    Serial.print("error_d: ");
    Serial.print(error_maze_d);
    Serial.print("   ");
    Serial.print("Kp * error: ");
    Serial.print(Kp_maze * error_maze);
    Serial.print("   ");
    Serial.print("Kd * error_d: ");
    Serial.print(Kd_maze * error_maze_d);
    Serial.print("   ");
  */
}

void wall_following(void)
{
  sonar_data = UltrasonicSensorData[2] - UltrasonicSensorData[1] ; // 오른쪽 소나 -  왼쪽소나
  Steering_Angle = PID_Control_wall(sonar_data);
  steering_control(Steering_Angle);
}


void mission_flag_serial_print(void)
{
  Serial.print("mission_flag: ");
  Serial.print(mission_flag);
  Serial.println("  ");
}

void setup()
{
  // put your setup code here, to run once:
  for (int a = 0; a < NPIXELS; a++)
  {
    LineSensor_Data[a] = 0;
    LineSensor_Data_Adaption[a] = 0;
    MAX_LineSensor_Data[a] = 1023;
    MIN_LineSensor_Data[a] = 0;
  }

  pinMode(SIpin, OUTPUT);
  pinMode(CLKpin, OUTPUT);
  // pinMode(A0pin, INPUT);
  digitalWrite(CLKpin, LOW);
  digitalWrite(SIpin, LOW);

#if FASTADC
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif

  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  Steeringservo.attach(RC_SERVO_PIN);
  Steeringservo.write(NEURAL_ANGLE);

  interrupt_setup();

  //MsTimer2::set(50, CallBack);
  //MsTimer2::start();

  Serial.begin(115200);
}

int timer = 0;
void Obstacle(void) {
  if (millis() - timer >= 500)
  {
    volatile long encoder_save = encoderPos;
    while (encoderPos - encoder_save <= 600)    //600
    {
      Steering_Angle = NEURAL_ANGLE - 21;
      steering_control(-21);
      motor_control(150);   //150   145
    }
    while (encoderPos - encoder_save <= 1380)
    {
      Steering_Angle = NEURAL_ANGLE + 21;
      steering_control(21);
      motor_control(150);   //150 145
    } 

    timer = millis();
  }
}
void loop()
{
  //mission_flag_serial_print();
  //encoderPos_serial_print();
  //ultrasonic_sensor_serial_print();   //1320  1320
  //lane_data_serial_print();
  // lane_control();

  switch (mission_flag)
  {
    //stop
    case 0:
      read_ultrasonic_sensor();
      if (UltrasonicSensorData[0] <= 450)
      {
        motor_control(0);
      }
      else
      {
        mission_flag = 1;
      }
      break;
    //lane 주행
    case 1:
      lane_control();
      if ((encoderPos) < 4200)
      {
        motor_control(210);
      }
      if ( ((encoderPos) > 4200) && ((encoderPos) < 7400))
      {
        motor_control(150);
      }
      if ( ((encoderPos) > 7400) && ((encoderPos) < 11500))
      {
        motor_control(190);
      }
      if ( ((encoderPos) > 11500) && ((encoderPos) < 11900))  // 11870
      {
        motor_control(150);
      }
      if (encoderPos >= 11900)    //11870
      {

        motor_control(0);
        reset_encoder();
        delay(10);
        steering_control_0();
        mission_flag = 2;
      }
      break;
    //wallfollowing
    case 2:
      read_ultrasonic_sensor();
      wall_following();
      motor_control(150);
      if (UltrasonicSensorData[0] <= 1310)    //1290    //1100
      {
        motor_control(0);
        steering_control_0();
        delay(10);
        reset_encoder();
        mission_flag = 3;
      }
      break;
    //90도 회전
    case 3:
      steering_control_maze(25);
      motor_control(165);
      if (encoderPos >= 1610)
      {
        motor_control(0);
        steering_control_0();
        delay(10);
        reset_encoder();
        mission_flag = 4;
      }
      break;
    //wallfollowing
    case 4:
      read_ultrasonic_sensor();
      wall_following();
      motor_control(165);
      if (encoderPos >= 2500)
      {
        if (UltrasonicSensorData[0] <= 1455)
        {
          motor_control(0);
          steering_control_0();
          delay(10);
          reset_encoder();
          mission_flag = 5;
        }
      }
      break;
    case 5 :
      steering_control_maze_1(25);
      motor_control(160);
      if (encoderPos >= 1620)
      {
        motor_control(0);
        steering_control_0();
        delay(10);
        reset_encoder();

        mission_flag = 6;
      }
      break;
    case  6:
      lane_control();
      motor_control(155);
      if (encoderPos >= 9140)    //11000
      {
        motor_control(0);
        reset_encoder();
        delay(1000);
        mission_flag = 7;
      }
      break;
    case 7:
      Obstacle();
      delay(1000);
      reset_encoder();
      mission_flag = 8;
      break;
    case 8:
      lane_control();
      motor_control(155);
      if (encoderPos > 9400)    //9300
      {
        motor_control(0);
        delay(10);
        mission_flag = 9;
      }
      break;
    case 9:
      read_ultrasonic_sensor();
      motor_control(130);
      if (UltrasonicSensorData[0] <= 400)
      {
        motor_control(0);
      }
      break;
  }

}
