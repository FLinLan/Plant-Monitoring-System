int resVal = 0;
int resPin = A5;

void setup() {
  Serial.begin(9600); 
}

void loop() {
  resVal = analogRead(resPin);

  if (resVal <= 100) {
    Serial.println("Water Level: Empty");
  } else if (resVal > 100 && resVal <= 250) {
    Serial.println("Water Level: Low");
  } else if (resVal > 250 && resVal <= 500) {
    Serial.println("Water Level: Medium");
  } else if(resVal > 500) {
    Serial.println("Water Level: High");
  }
  delay(1000);
}
