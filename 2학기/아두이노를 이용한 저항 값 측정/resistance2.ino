void setup()
{
  Serial.begin(9600);
}

void loop()
{
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.0);

  float supplyVoltage = 5.0;

  float R1;

  if (voltage > 0)
  {
    R1 = (1.0 * supplyVoltage) / (supplyVoltage / voltage - 1);
    
    float R2 = (R1 * voltage) / (supplyVoltage - voltage);

    Serial.print("Voltage_out: ");
    Serial.println(voltage);
    Serial.print("저항1: ");
    Serial.println(R1);
    Serial.print("저항2: ");
    Serial.println(R2);
    Serial.println(" ");
  }
  else 
  {
    Serial.println("Voltage is 0. Cannot calculate resistances.");
  }

  delay(1000);
}
