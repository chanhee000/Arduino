#define A0pin A0
#define SIpin  22
#define CLKpin  23
#define NPIXELS 128

byte Pixel[NPIXELS];

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

#if FASTADC
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif

  flag_line_adapation = 0;
  Serial.begin(115200);
  Serial.println("TSL1401");
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

void loop()
{
  // put your main code here, to run repeatedly:
  read_line_camera();

   for (int i = 0; i < NPIXELS; i++)
   {
    Serial.println((byte)Pixel[i] + 1);
   }
}
