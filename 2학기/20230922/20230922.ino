#include <MPU6050_tockn.h>

#include <MsTimer2.h>


#define RAD2DEG(x)    (x*180.0/3.14159)
#define DEG2RAD(x)    (x*3.14159/180.0)
#define wheel_track  0.144     //m단위로 구하기    0.1인경우 10cm
#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13
/////////////imu////////////////

#include <Wire.h>
MPU6050 mpu6050(Wire);

///////////////////////////////////////

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

double pulse_to_distance_left  = 0.2 / 939.8;
double pulse_to_distance_right = 0.2 / 503.1;
//double pulse_to_distance_left = 0.2/503.1;



const double odom_left  = 0;
const double odom_right = 0;

double yaw  = 0.0;
double yaw1 = 0.0;
double yaw_degree = 0.0;
double heading_angle = 0.0;
float init_heading_angle = 0.0; // 초기 방향
float target_heading_angle = 0.0;
float heading_angle_error = 0.0;  //  error 값

struct Pose2D
{
  double x;
  double y;
  double theta;
}
my_odom;


double heading(double x, double y)
{
  double head = atan2(y, x); //Slope Y,Slope X
  return head;
}

void msTimer2_ISR()
{
  char msg[100] = {0x00,};
  double odom_left_delta   = 0.0;
  double odom_right_delta  = 0.0;
  double odom_center_delta = 0.0;
  long delta_encoder_left  = 0;
  long delta_encoder_right = 0;
  double theta_delta  = 0.0;
  double theta_delta_degree  = 0.0;

  delta_encoder_left  = cnt1 - cnt1_old;
  delta_encoder_right = cnt2 - cnt2_old;

  odom_left_delta   = delta_encoder_left * pulse_to_distance_left;
  odom_right_delta  = delta_encoder_right * pulse_to_distance_right;
  odom_center_delta = (odom_left_delta + odom_right_delta) * 0.5;     //나누기 대신 * 0.5를 사용하는 이유: 곱하기가 나누기보다 속도가 빠르다

  theta_delta = heading(wheel_track, (odom_left_delta - odom_right_delta));

  yaw += theta_delta;
  theta_delta_degree = RAD2DEG(theta_delta);   //theta_delta * 180.8/3.14150;

  yaw_degree += theta_delta_degree;

  my_odom.x     += odom_center_delta * sin(theta_delta);
  my_odom.y     += odom_center_delta * cos(theta_delta);
  my_odom.theta += theta_delta;

  

  cnt1_old = cnt1;
  cnt2_old = cnt2;


  //double wheel_track = 0.15;
  //const double yaw = 0.0;
  //const double yaw_degree = 0.0;
  
  //delta_encoder_left  = 100;
  //delta_encoder_right = -100;

  //sprintf(msg,"encoder delta : [%3d %3d]",delta_encoder_left,delta_encoder_right);
  //Serial.println(msg);

  //Serial.print("delta_encoder_left: ");
  //Serial.print(delta_encoder_left);
  //Serial.print("    ");
  //Serial.print("delta_encoder_right: ");
  //Serial.println(delta_encoder_right);

  //Serial.print("odom_left_delta: "); Serial.print(odom_left_delta);
  //Serial.print("    ");
  //Serial.print("odom_right_delta: "); Serial.println(odom_right_delta);
  //Serial.print("w.t.:  "); Serial.println(wheel_track);

  //Serial.print("delta_theta_raidan: "); Serial.println(theta_delta);

  //Serial.print("yaw radian: "); Serial.println(yaw);
  //Serial.print("theta_delta_degree: "); Serial.println(theta_delta_degree); Serial.println("");

  //Serial.print("yaw degree : "); Serial.println(yaw_degree); Serial.println("");
}

//////////////////////////////////


void setup()
{
  pinMode(outPin, OUTPUT); // Output mode
  //pinMode(interruptPin1, INPUT_PULLUP); // Input mode, pull-up
  //pinMode(interruptPin2, INPUT_PULLUP); // Input mode, pull-up
  pinMode(encoder1_A, INPUT_PULLUP);
  pinMode(encoder1_B, INPUT_PULLUP);
  pinMode(encoder2_A, INPUT_PULLUP);
  pinMode(encoder2_B, INPUT_PULLUP);
  pinMode(resetPinm, INPUT);
  //attachInterrupt(digitalPinToInterrupt(interruptPin1), intfunc1, RISING); // Enable interrupt
  //attachInterrupt(digitalPinToInterrupt(interruptPin2), intfunc2, RISING); // Enable interrupt
  attachInterrupt(digitalPinToInterrupt(encoder1_A), intfunc1, RISING); // Enable interrupt
  attachInterrupt(digitalPinToInterrupt(encoder2_A), intfunc2, RISING); // Enable interrupt

  my_odom.x     = 0;
  my_odom.y     = 0;
  my_odom.theta = 0;

  Serial.begin(115200);
  Wire.begin(); // IMU initiallize
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  MsTimer2::set(1000, msTimer2_ISR);
  MsTimer2::start();



}

////////////////////////////////////////////////

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

void motor_R_control(int direction_R, int motor_speed_R) //모터 A의 방향(direction)과 속도(speed)제어
{
  if (direction_R == HIGH)
  {
    digitalWrite(IN1, LOW);      //모터의 방향제어
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, motor_speed_R); //모터의 속도 제어
  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, motor_speed_R);
  }
}

void motor_L_control(int direction_L, int motor_speed_L) //모터 B의 방향(direction)과 속도(speed)제어
{
  if (direction_L == HIGH)
  {
    digitalWrite(IN3, HIGH);          //모터의 방향제어
    digitalWrite(IN4, LOW);
    analogWrite(ENB, motor_speed_L); //모터의 속도 제어
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, motor_speed_L);
  }
}

void read_imu_sensor(void)
{
  mpu6050.update();
  float heading1 = mpu6050.getAngleZ();
  float heading2 = mpu6050.getAngleZ();
  float heading3 = mpu6050.getAngleZ();

  heading_angle = (heading1 + heading2 + heading3) / 3;

  heading_angle = 360 - heading_angle;  // 회전 좌표계를 반시계 방향으로 전환

  while (heading_angle >= 360)
  {
    heading_angle -= 360;
  }

  heading_angle_error = target_heading_angle - heading_angle;

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

  }

}
void imu()
{
  mpu6050.update();
  yaw1 = mpu6050.getAngleZ();
  Serial.print("yaw1 : ");
  Serial.println(yaw1);
  delay(100);
}
void loop()
{
  Serial.print("odom :");
  Serial.print(my_odom.x);  Serial.print(" ");
  Serial.print(my_odom.y);  Serial.print(" ");
  Serial.println(my_odom.theta); Serial.print(" ");
  delay(100);
  imu();
  /*
  read_imu_sensor();

  // 전진
  motor_R_control(HIGH, 50);
  motor_L_control(HIGH, 50);
  delay(1000);

  // 정지
  motor_R_control(HIGH, 0);
  motor_L_control(HIGH, 0);
  delay(1000);

  //turn();
  while (yaw1 > -10)
  {
    // 회전
    motor_R_control(HIGH, 70);
    motor_L_control(LOW, 70);
    while (mpu6050.getAngleZ() >= 45 && mpu6050.getAngleZ() <= 47)
    {
      motor_R_control(HIGH, 0); // 회전을 멈춤
      motor_L_control(LOW, 0);
      delay(1000);
      break;
    }
  }

  // 전진
  motor_R_control(HIGH, 50);
  motor_L_control(HIGH, 50);
  delay(1000);

  // 정지
  motor_R_control(HIGH, 0);
  motor_L_control(HIGH, 0);
  delay(1000);
*/

  /*
    motor_R_control(HIGH, 50);//직진
    motor_L_control(HIGH, 50);
    delay(1000);
    motor_R_control(HIGH, 0);//정지
    motor_L_control(HIGH, 0);
    delay(1000);

    if (yaw1 < 0)          // 만약 45도가 되면 정지
    {
      motor_R_control(HIGH, 70);//좌회전
      motor_L_control(LOW, 70);
      delay(2000);
      if (yaw1 == 45)
      {
        motor_R_control(HIGH, 0);
        motor_L_control(LOW, 0);
        delay(1000);
      }
    }
  */
  /*
    motor_R_control(HIGH, 50);//직진
    motor_L_control(HIGH, 50);
    delay(500);
    motor_R_control(HIGH, 0);//정지
    motor_L_control(HIGH, 0);
    delay(500);
    motor_R_control(HIGH, 50);//좌회전
    motor_L_control(LOW, 50);
    if (yaw1 == 135)          // 만약 45도가 되면 정지
    {
    motor_R_control(HIGH, 0);
    motor_L_control(LOW, 0);
    delay(500);
    }
    motor_R_control(HIGH, 50);//직진
    motor_L_control(HIGH, 50);
    delay(500);
    motor_R_control(HIGH, 0);//정지
    motor_L_control(HIGH, 0);
    delay(500);
    motor_R_control(HIGH, 50);//좌회전
    motor_L_control(LOW, 50);
    if (yaw1 == 135)          // 만약 45도가 되면 정지
    {
    motor_R_control(HIGH, 0);
    motor_L_control(LOW, 0);
    delay(500);
    }
    motor_R_control(HIGH, 50);//직진
    motor_L_control(HIGH, 50);
    delay(500);
    motor_R_control(HIGH, 0);//정지
    motor_L_control(HIGH, 0);
    delay(500);
  */


  /*
    //전진
    motor_R_control(HIGH, 50);
    motor_L_control(HIGH, 50);
    delay(500);
    //회전
    yaw1 = mpu6050.getAngleZ();
    if ((yaw1 > -2) && (yaw1 < 0))
    {
      motor_R_control(HIGH, 100);
      motor_L_control(LOW , 100);
    }
    if (yaw1 > 45)
    {
      motor_R_control(HIGH, 0);
      motor_L_control(LOW , 0);
    }
  */
  //전진
  // motor_R_control(HIGH, 50);
  // motor_L_control(HIGH, 50);
  // delay(500);

  /*Serial.print(cnt1);
    Serial.print("     ");
    Serial.println(cnt2);
    delay(1000);
    if(digitalRead(resetPinm) == HIGH)
    {
    cnt1 = 0;
    cnt2 = 0;
    }*/

}
