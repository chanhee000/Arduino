/*int BUTTON_PIN = 3;

int LED_PIN1 = 13;
int LED_PIN2 = 12;
int LED_PIN3 = 11;

int currentLED = 0;
bool lastButtonState = HIGH;
bool buttonPressed = false; 

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // 풀업 저항 활성화

  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(LED_PIN3, OUTPUT);

  digitalWrite(LED_PIN1, LOW);  // 처음에는 모든 LED를 끔
  digitalWrite(LED_PIN2, LOW);
  digitalWrite(LED_PIN3, LOW);
}

void loop() {
  bool buttonState = digitalRead(BUTTON_PIN);


  if (buttonState == LOW && lastButtonState == HIGH) {
    buttonPressed = true; 
  }

  if (buttonPressed) {
    buttonPressed = false; 

    // 현재 켜져 있는 LED를 끄고
    if (currentLED == 0) {
      digitalWrite(LED_PIN1, LOW);
    } else if (currentLED == 1) {
      digitalWrite(LED_PIN2, LOW);
    } else if (currentLED == 2) {
      digitalWrite(LED_PIN3, LOW);
    }

    // 다음 LED로 이동
    currentLED = (currentLED + 1) % 3;

    // 새 LED를 켬
    if (currentLED == 0) {
      digitalWrite(LED_PIN1, HIGH);
    } else if (currentLED == 1) {
      digitalWrite(LED_PIN2, HIGH);
    } else if (currentLED == 2) {
      digitalWrite(LED_PIN3, HIGH);
    }
  }

  lastButtonState = buttonState; 

  delay(50); 
}*/
/*int LED_PIN1 = 13;
int LED_PIN2 = 12;
int LED_PIN3 = 11;

void setup() {
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(LED_PIN3, OUTPUT);


  digitalWrite(LED_PIN1, LOW);
  digitalWrite(LED_PIN2, LOW);
  digitalWrite(LED_PIN3, LOW);


  Serial.begin(115200);
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();

    if (input == '\n' || input == '\r') {
      return; 
    }
    if (input == '1') {
      digitalWrite(LED_PIN1, HIGH); 
      digitalWrite(LED_PIN2, LOW); 
      digitalWrite(LED_PIN3, LOW);
      Serial.println("Blue_LED");
    } 
    else if (input == '2') {
      digitalWrite(LED_PIN1, LOW);
      digitalWrite(LED_PIN2, HIGH); 
      digitalWrite(LED_PIN3, LOW);
      Serial.println("Yellow_LED");
    } 
    else if (input == '3') {
      digitalWrite(LED_PIN1, LOW);
      digitalWrite(LED_PIN2, LOW);
      digitalWrite(LED_PIN3, HIGH);
      Serial.println("RED_LED");
    } 
    else if (input == '4') {
      digitalWrite(LED_PIN1, HIGH);
      digitalWrite(LED_PIN2, HIGH);
      digitalWrite(LED_PIN3, LOW);
      Serial.println("Blue_LED   AND Yellow_LED");
    } 
    else if (input == '5') {
      digitalWrite(LED_PIN1, LOW);
      digitalWrite(LED_PIN2, HIGH);
      digitalWrite(LED_PIN3, HIGH);
      Serial.println("Yellow_LED AND RED_LED");
    } 
    else if (input == '6') {
      digitalWrite(LED_PIN1, HIGH);
      digitalWrite(LED_PIN2, LOW);
      digitalWrite(LED_PIN3, HIGH);
      Serial.println("Blue_LED   AND RED_LED");
    } 
    else if (input == '7') {
      digitalWrite(LED_PIN1, HIGH);
      digitalWrite(LED_PIN2, HIGH);
      digitalWrite(LED_PIN3, HIGH);
      Serial.println("ALL_LED");
    } 
    else {
      digitalWrite(LED_PIN1, LOW);
      digitalWrite(LED_PIN2, LOW);
      digitalWrite(LED_PIN3, LOW);
      Serial.println("ALL_OFF");
    }
  }
}*/


struct LED {
  int pin;  
  bool state;  
};

//LED가 3개라 배열 3개 생성
LED leds[] = {
  {13, false}, 
  {12, false},
  {11, false} 
};

int NUM_LEDS = 3; 

void setup() {
  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(leds[i].pin, OUTPUT);
    digitalWrite(leds[i].pin, LOW);
    leds[i].state = false; 
  }

  Serial.begin(115200);
}

void loop() {
  if (Serial.available() > 0) {
    // 문자열 전체를 읽기 (줄바꿈 문자('\n')까지)
    String input = Serial.readStringUntil('\n');
    input.trim();  // 입력된 문자열에서 앞뒤 공백 제거

    // 입력된 문자열에 따라 LED 제어
    if (input == "Blue") {
      controlLEDs(0);  // 첫 번째 LED(Blue)를 켜기
      Serial.println("Blue_LED");
    }
    else if (input == "Yellow") {
      controlLEDs(1);  // 두 번째 LED(Yellow)를 켜기
      Serial.println("Yellow_LED");
    }
    else if (input == "Red") {
      controlLEDs(2);  // 세 번째 LED(Red)를 켜기
      Serial.println("RED_LED");
    }
    else if (input == "ALL_LED") {
      turnOnAllLEDs();  // 모든 LED 켜기
      Serial.println("ALL_LED");
    }
    else if (input == "OFF_LED") {
      turnOffAllLEDs();  // 모든 LED 끄기
      Serial.println("ALL_OFF");
    }
    else {
      Serial.println("Invalid Input");  // 잘못된 입력 처리
    }
  }
}

// 특정 LED만 켜고 나머지는 끄는 함수
void controlLEDs(int ledIndex) {
  for (int i = 0; i < NUM_LEDS; i++) {
    if (i == ledIndex) {
      digitalWrite(leds[i].pin, HIGH);
      leds[i].state = true;  
    } else {
      digitalWrite(leds[i].pin, LOW); 
      leds[i].state = false; 
    }12417
  }
}

// 모든 LED를 켜는 함수
void turnOnAllLEDs() {
  for (int i = 0; i < NUM_LEDS; i++) {
    digitalWrite(leds[i].pin, HIGH);
    leds[i].state = true;
  }
}

// 모든 LED를 끄는 함수
void turnOffAllLEDs() {
  for (int i = 0; i < NUM_LEDS; i++) {
    digitalWrite(leds[i].pin, LOW);
    leds[i].state = false;
  }
}
