#include <Arduino.h>


int analogPin = A0;
int val = 0;

void setup() {
  Serial.begin(9600);
  Serial.flush();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    int val = char(Serial.read()) - '0';
    if (val == 1) {
      Serial.write("ON\n");
      digitalWrite(LED_BUILTIN, HIGH);
    }
    if (val == 0) {
      Serial.write("OFF\n");
      digitalWrite(LED_BUILTIN, LOW);
    }
    if (val == 2) {
      val = analogRead(analogPin);
      Serial.println(val);
    }
  }
}
