int pirSensorPin = 6;
int ledPin = 7;

void setup() {
  pinMode(pirSensorPin, INPUT);
  pinMode(ledPin, OUTPUT);

  Serial.begin(115200);
}

void loop() {
  int pirValue = digitalRead(pirSensorPin);

  if (pirValue == HIGH) {
    digitalWrite(ledPin, HIGH);
    Serial.println("Motion detected");
  } else {
    digitalWrite(ledPin, LOW);
  }

  delay(1000);
}
