#include <MsTimer2.h>

#define RAD2DEG(x)   (x*180.0/3.14159)
#define DEG2RAD(x)   (x*3.14159/180.0)
#define wheel_track 0.068 //양쪽 바뀌       // m 단위로 구할 것 0.1 -> 10cm

#define BTSerial Serial2
#define RxD 17
#define TxD 16
#define BT_BAUDRATE 9600

/////////////////// L298 ////////////////////
#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13

struct Pose2D
{
  double x;
  double y;
  double theta; //theta = radian
} my_odom;

const byte outPin = 13; // Output pin: digital pin 13(D13)
//const byte interruptPin1 = 20; // Interrupt pin: D2
//const byte interruptPin2 = 21; // Interrupt pin: D2
const byte encoder1_A = 2; // Interrupt pin: D2
const byte encoder1_B = 3; // Interrupt pin: D2
const byte encoder2_A = 18; // Interrupt pin: D2
const byte encoder2_B = 19; // Interrupt pin: D2
const byte resetPin = 5;
volatile byte state = 0;
//unsigned long cnt1 = 0; // 추가
//unsigned long cnt2 = 0; // 추가
long cnt1 = 0; // 추가
long cnt2 = 0; // 추가

long cnt1_old = 0; // 추가
long cnt2_old = 0; // 추가

double pulst_to_distance_left = 0.2 / 974.6;
double pulst_to_distance_right = 0.2 / 974.5;

const double odom_left = 0;
const double odom_right = 0;

double yaw = 0.0;
double yaw_degree = 0.0;
double yaw1 = 0.0; // 추가

double heading(double x, double y)
{
  double head = atan2(y, x); // Slope Y, Slope X  x = wheel_track
  return head;
}

void MsTimer2_ISR()
{
  double odom_left_delta = 0.0;
  double odom_right_delta = 0.0;
  double odom_center_delta = 0.0;
  long delta_encoder_left = 0;
  long delta_encoder_right = 0;
  double theta_delta = 0.0;
  double theta_delta_degree = 0.0;

  delta_encoder_left = cnt1 - cnt1_old;
  delta_encoder_right = cnt2 - cnt2_old;

  // Serial.print("theta_delta = "); Serial.print(theta_delta); Serial.print("    ");

  theta_delta_degree = DEG2RAD(theta_delta);
  //  Serial.print("theta_delta_degree = "); Serial.print(theta_delta_degree); Serial.print("    ");

  odom_left_delta = delta_encoder_left * pulst_to_distance_left;
  odom_right_delta = delta_encoder_right * pulst_to_distance_right;
  odom_center_delta = (odom_left_delta + odom_right_delta) * 0.5; // 안전성 나누기 지원 안하는 것도 있고 속도도 조금 더 빠르기 때문

  //  Serial.print("delta_encoder_left = ");  Serial.print( delta_encoder_left);  Serial.print("    ");
  // Serial.print("delta_encoder_right = "); Serial.print( delta_encoder_right); Serial.println("    ");

  // Serial.print("odom_left_delta = ");  Serial.print(odom_left_delta);  Serial.print("    ");
  //  Serial.print("odom_right_delta = "); Serial.print(odom_right_delta); Serial.println("    ");

  theta_delta = heading(wheel_track, (odom_right_delta - odom_left_delta));
  // Serial.print("delta theta radian: ");  Serial.print(theta_delta); Serial.println("    ");
  yaw += theta_delta;
  theta_delta_degree = RAD2DEG(theta_delta);
  // Serial.print("yaw radian: ");  Serial.print(yaw); Serial.println("    ");
  // Serial.print("delta theta degree: ");  Serial.print(theta_delta_degree); Serial.println("    ");
  yaw_degree += theta_delta_degree;
  // Serial.print("yaw degree: ");  Serial.print(yaw_degree); Serial.println("    ");

  my_odom.x += odom_center_delta * sin(theta_delta);
  my_odom.y += odom_center_delta * cos(theta_delta);
  my_odom.theta += theta_delta;

  // Serial.print("odom : ");
  // Serial.print(my_odom.x); Serial.print("    ");
  // Serial.print(my_odom.y); Serial.print("    ");
  //Serial.println(my_odom.theta); Serial.println("    ");

  cnt1_old = cnt1;
  cnt2_old = cnt2;
}

void update_odometry()
{
  // 엔코더 값에서 이동 거리를 계산합니다.
  double delta_encoder_left = cnt1 - cnt1_old;
  double delta_encoder_right = cnt2 - cnt2_old;
  double odom_left_delta = delta_encoder_left * pulst_to_distance_left;
  double odom_right_delta = delta_encoder_right * pulst_to_distance_right;
  double odom_center_delta = (odom_left_delta + odom_right_delta) * 0.5;

  // IMU에서 측정한 방향을 라디안으로 변환합니다.
  double theta_delta = DEG2RAD(yaw1);

  // 로봇의 위치와 방향을 업데이트합니다.
  my_odom.x += odom_center_delta * sin(theta_delta);
  my_odom.y += odom_center_delta * cos(theta_delta);
  my_odom.theta += theta_delta;

  // 업데이트된 엔코더 값 저장
  cnt1_old = cnt1;
  cnt2_old = cnt2;
}


////////// IMU //////////
#include <Wire.h>
#include <LSM303.h>

#define THRESHOLD_ANGLE1 15
#define THRESHOLD_ANGLE2 7

LSM303 compass;

float heading_angle      = 0.0;
float init_heading_angle = 17.0;   // 초기 방향
float target_heading_angle = 90;
float heading_angle_error = 0.0;  // error 값

void setup()
{
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(outPin, OUTPUT); // Output mode
  //  pinMode(interruptPin1, INPUT_PULLUP); // Input mode, pull-up
  //  pinMode(interruptPin2, INPUT_PULLUP); // Input mode, pull-up
  pinMode(encoder1_A, INPUT_PULLUP);
  pinMode(encoder1_B, INPUT_PULLUP);
  pinMode(encoder2_A, INPUT_PULLUP);
  pinMode(encoder2_B, INPUT_PULLUP);
  pinMode(resetPin, INPUT);
  //  attachInterrupt(digitalPinToInterrupt(interruptPin1), intfunc1, RISING); // Enable interrupt
  //  attachInterrupt(digitalPinToInterrupt(interruptPin2), intfunc2, RISING); // Enable interrupt
  attachInterrupt(digitalPinToInterrupt(encoder1_A), intfunc1, RISING); // Enable interrupt
  attachInterrupt(digitalPinToInterrupt(encoder2_A), intfunc2, RISING); // Enable interrupt

  my_odom.x = 0;   my_odom.y = 0;   my_odom.theta = 0;

  Serial.begin(115200);
  BTSerial.begin(BT_BAUDRATE);

  MsTimer2::set(100, MsTimer2_ISR); // 0.1초마다 작동
  MsTimer2::start();

  Wire.begin();  // IMU initiallize
  compass.init();
  compass.enableDefault();

  compass.m_min = (LSM303::vector<int16_t>)
  {
    -32767, -32767, -32767
  };
  compass.m_max = (LSM303::vector<int16_t>)
  {
    +32767, +32767, +32767
  };
}

void read_imu_sensor(void)
{
  compass.read();
  float heading1 = compass.heading();
  compass.read();
  float heading2 = compass.heading();
  heading_angle = (heading1 + heading2) / 2;
  heading_angle = 360 - heading_angle; // 회전 좌표계를 반시계 방향으로 할 것
  heading_angle_error = target_heading_angle - heading_angle;

  yaw1 = heading_angle; // IMU 측정값을 yaw1 변수에 저장

  if (heading_angle_error > 180)
  {
    heading_angle_error = heading_angle_error - 360;
  }
  else if (heading_angle_error < -180)
  {
    heading_angle_error = heading_angle_error + 360;
  }
  else
  {
    // 다른 작업 수행
  }
}


void imu_rotation(void)
{
  bool flag = true; // bool 타입은 true(1) 또는 false(0)
  while (flag)
  {
    read_imu_sensor();

    if (heading_angle_error > THRESHOLD_ANGLE1) // 반시계방향으로 회전
    {
      motor_L_control(-200);
      motor_R_control(200);
    }
    else if ((heading_angle_error >= THRESHOLD_ANGLE2) && (heading_angle_error <= THRESHOLD_ANGLE1)) //
    {
      motor_L_control(-200);
      motor_R_control(200);
    }
    else if ((heading_angle_error > -THRESHOLD_ANGLE2) && (heading_angle_error < THRESHOLD_ANGLE2)) // 정지
    {
      motor_L_control(0);
      motor_R_control(0);
      flag = false; // 루프를 빠져나오도록 설정
    }
    else if ((heading_angle_error >= -THRESHOLD_ANGLE1) && (heading_angle_error <= -THRESHOLD_ANGLE2)) //
    {
      motor_L_control(200);
      motor_R_control(-200);
    }
    else // heading_angle_error <-THRESHOLD_ANGLE // 시계방향으로 회전
    {
      motor_L_control(200);
      motor_R_control(-200);
    }
    Serial.print("Heading Angle Error : ");
    Serial.print(heading_angle_error); //heading_angle error 표시
    Serial.print(" = ");
    Serial.print(target_heading_angle);
    Serial.print(" - ");
    Serial.println(heading_angle); //heading_angle 표시
  }
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

/*
  void intfunc1() // Interrupt function
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
  }

*/
void intfunc1()
{
  if (digitalRead(encoder1_B) == HIGH)
  {
    cnt1--;
  }
  else
  {
    cnt1++;
  }
}

void intfunc2()
{
  if (digitalRead(encoder2_B) == LOW)
  {
    cnt2--;
  }
  else
  {
    cnt2++;
  }
}

void loop()
{
  /*
    // 목표 방향 설정 및 회전
    //target_heading_angle = 130;
    //imu_rotation();

    // 오도메트리 정보 업데이트
    update_odometry();

    // 이제 업데이트된 오도메트리 정보를 사용하여 필요한 작업을 수행할 수 있습니다.

    // 업데이트된 로봇의 위치 및 방향 정보를 출력
    Serial.print("Robot Position (x, y, theta): ");
    Serial.print(my_odom.x);
    Serial.print(", ");
    Serial.print(my_odom.y);
    Serial.print(", ");
    Serial.println(my_odom.theta);

    Serial.print("cnt1 ");
    Serial.print(cnt1);
    Serial.print("cnt2 ");
    Serial.println(cnt2);


    delay(500);
  */
  /*
    // BT –> Data –> Serial
    if (BTSerial.available()) {
      Serial.write(BTSerial.read());
      digitalWrite (13, HIGH);    delay(10);
    }
    // Serial –> Data –> BT
    if (Serial.available()) {
      BTSerial.write(Serial.read());
      digitalWrite (13, HIGH);    delay(10);
    }
    digitalWrite (13, LOW);
  */
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
