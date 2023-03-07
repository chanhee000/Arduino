/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/
//주석
/*첫 LED프로그램 과제입니다 
 */

// the setup function runs once when you press reset or power the board
void setup() {
  // put your setup code here, to run once;
  pinMode(3,OUTPUT); //3번 핀을 출력으로 설정
  pinMode(4,INPUT); //4번 핀을 입력으로 설정
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(3, HIGH);   //3번 핀 출력을 HIGH로
  delay(1000);             //1000msec 지연
  digitalWrite(3, LOW);    // 3번 핀 출력을 LOW로
  delay(1000);             //1000msec 지연
}
