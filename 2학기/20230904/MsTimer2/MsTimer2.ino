#include <MsTimer2.h>

unsigned long time1;
unsigned long time2;

void flash()
{
  static boolean output = HIGH;

  digitalWrite(19,output);
  output = !output;
}

void setup()
{
  pinMode(19, OUTPUT);
  Serial.begin(115200);
  
  MsTimer2::set(1, flash); 
  MsTimer2::start();
}

void loop()
{
  
}
