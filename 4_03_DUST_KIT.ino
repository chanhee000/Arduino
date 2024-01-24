#include <Wire.h>               // i2C 통신을 위한 라이브러리
#include <LiquidCrystal_I2C.h>  // LCD 1602 I2C용 라이브러리
#include "DHT.h"                // 온습도 센서 사용을 위한 라이브러리
#define DHTPIN A1               // 온습도 센서 핀 지정
#define DHTTYPE DHT11           // DHT 타입 지정
DHT dht(DHTPIN, DHTTYPE);       // DHT11의 타입, 핀을 dht로 지정

int dust_sensor = A0;     // 미세먼지 핀 설정

int rgb_red = 5;          // rgb 핀 빨간색 핀
int rgb_green = 6;        // rgb핀 녹색색 핀
int rgb_blue = 7;         // rgb핀 파란색 핀

float dust_value = 0;     // 센서에서 입력받은 미세먼지 값
float dustDensityug = 0;  // ug/m^3 값을 계산

int sensor_led = 12;      // 미세먼지 센서 안에 있는 적외선 led 핀 번호
int sampling = 280;       // 적외선 led를 키고, 센서값을 읽어들여 미세먼지를 측정하는 샘플링 시간
int waiting = 40;
float stop_time = 9680;   // 센서를 구동하지 않는 시간

double five_dust[5] = { 0 };
double recent_dust = 0;
double total_dust = 0;

float dust_init = 0;
float dust_initial = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);  // 접근주소: 0x3F or 0x27 1602 Display

byte humi[8] = {     // 물컵모양 출력
  0b00000,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
};
byte temp[8] = {     // 온도계 모양 출력
  0b00100,
  0b01010,
  0b01010,
  0b01010,
  0b01010,
  0b10001,
  0b11111,
  0b01110,
};
byte char_temp[8] = { // 온도 단위 출력
  0b10000,
  0b00110,
  0b01001,
  0b01000,
  0b01000,
  0b01000,
  0b01001,
  0b00110,
};

void setup() {
  Serial.begin(9600);            // 시리얼 모니터 시작, 속도는 9600
  dht.begin();
  lcd.init();                    // LCD 초기화
  lcd.backlight();               // 백라이트 켜기
  lcd.createChar(1, temp);       // 온도계모양 출력
  lcd.createChar(2, humi);       // 물컵 모양 출력
  lcd.createChar(3, char_temp);  // 온도 단위 출력
  pinMode(sensor_led, OUTPUT);   // 미세먼지 적외선 led를 출력으로 설정

  pinMode(rgb_red, OUTPUT);      // 3색 LED 모듈 출력으로 설정, 붉은색
  pinMode(rgb_green, OUTPUT);    // 녹색
  pinMode(rgb_blue, OUTPUT);     // 파란색

  Serial.begin(9600);            // 시리얼 모니터 시작, 속도는 9600

  for (int i = 0; i < 5; i++) {                       //미세먼지 측정센서 초기 값 구하기
    digitalWrite(sensor_led, LOW);                    //미세먼지 측정 5번하기
    delayMicroseconds(sampling);
    dust_init += analogRead(dust_sensor);
    delayMicroseconds(waiting);
    digitalWrite(sensor_led, HIGH);
    delayMicroseconds(stop_time);
  }
  dust_initial = (((dust_init / 5) * 5.0) / 1024.0);  //측정한 5번 미세먼지 값 평균 구하기
  Serial.print("dust_initial : ");
  Serial.println(dust_initial);
}

void loop() {
  digitalWrite(sensor_led, LOW);         // LED 켜기
  delayMicroseconds(sampling);           // 샘플링해주는 시간.

  int count = 0;
  dust_value = analogRead(dust_sensor);  // 센서 값 읽어오기

  delayMicroseconds(waiting);            // 너무 많은 데이터 입력을 피해주기 위해 잠시 멈춰주는 시간.

  digitalWrite(sensor_led, HIGH);        // LED 끄기
  delayMicroseconds(stop_time);          // LED 끄고 대기

  recent_dust = (((dust_value * (5.0 / 1024)) - dust_initial) / 0.005);  // 미세먼지 값 계산
  five_dust[4] = recent_dust;                                            // 새로운 미세먼지 값 입력
  total_dust = five_dust[4];                                             // 5개의 미세먼지 값을 저장할 변수

  for (int i = 0; i < 4; i++) {
    total_dust += five_dust[i];
    five_dust[i] = five_dust[i + 1];  // 0~4번째까지 미세먼지 값 저장을 위해 4번째 배열 비워주기
  }

  if (five_dust[0] != 0) {
    dustDensityug = total_dust / 5;
  }

  int humi = dht.readHumidity();
  int temp = dht.readTemperature();

  lcd.setCursor(0, 0);   // 1번째, 1라인
  lcd.write(byte(1));    // 온도계 출력
  lcd.setCursor(2, 0);   // 3번째, 1라인
  lcd.print((int)temp);  // 온도 출력
  lcd.setCursor(5, 0);   // 6번째 1라인
  lcd.write(byte(3));    // 온도 단위 출력

  lcd.setCursor(8, 0);   // 9번째, 1라인
  lcd.write(byte(2));    // 물컵 출력
  lcd.setCursor(10, 0);  // 11번째, 1라인
  lcd.print(humi);       // 습도 출력
  lcd.setCursor(13, 0);  // 15번째, 1라인
  lcd.print("%");        // % 출력

  lcd.setCursor(0, 1);       // 1번째, 2라인
  lcd.print("F.Dust");       // fine dust 글자 출력
  lcd.setCursor(7, 1);       // 6번째, 2라인
  lcd.print(dustDensityug);  // 미세먼지 출력
  lcd.setCursor(11, 1);
  lcd.print("ug/m3");

  if (dustDensityug <= 30.0)                                // 대기 중 미세먼지가 좋음 일때 파란색 출력
    light(0, 0, 255);
  else if (30.0 < dustDensityug && dustDensityug <= 80.0)   // 대기 중 미세먼지가 보통 일때 녹색 출력
    light(0, 255, 0);
  else if (80.0 < dustDensityug && dustDensityug <= 150.0)  // 대기 중 미세먼지가 나쁨 일때 노란색 출력
    light(255, 80, 1);
  else                                                      // 대기 중 미세먼지가 매우 나쁨 일때 빨간색 출력
    light(255, 0, 0);

  delay(1000);
}

void light(int a, int b, int c) {
  analogWrite(rgb_red, a);
  analogWrite(rgb_green, b);
  analogWrite(rgb_blue, c);
}

/*
 *  이 소스는 에듀이노(Eduino)에 의해서 번역, 수정, 작성되었고 소유권 또한 에듀이노의 것입니다. 
 *  소유권자의 허락을 받지 않고 무단으로 수정, 삭제하여 배포할 시 법적인 처벌을 받을 수 있습니다. 
 *
 *  미세먼지 측정 센서,온습도센서의 값을 LCD 1602(I2C)와 RGB LED 모듈로 출력하는 예제 코드입니다.
 *    
 */