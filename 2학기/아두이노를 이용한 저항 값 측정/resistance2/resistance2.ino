void setup()
{
  Serial.begin(9600);
}

void loop()
{
  int sensorValue = analogRead(A0);
  float Vout = sensorValue * (5.0 / 1023.0);

  float R1 = 46.7;

  float R2 = (Vout * R1) / (5.0 - Vout);

  Serial.print("출력 전압 (Vout): ");
  Serial.print(Vout, 2); // 소수점 둘째 자리까지 출력
  Serial.println(" V");

  Serial.print("두 번째 저항 R2 값: ");
  Serial.print(R2, 2); // 소수점 둘째 자리까지 출력
  Serial.println(" 옴");

  delay(1000);
}
