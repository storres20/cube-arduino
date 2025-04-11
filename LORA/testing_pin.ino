void setup() {
  pinMode(D2, OUTPUT); //
  pinMode(A0, OUTPUT);  // A0 = GPIO1 on Nano ESP32
}

void loop() {
  digitalWrite(D2, HIGH);
  digitalWrite(A0, HIGH);
  delay(500);
  digitalWrite(D2, LOW);
  digitalWrite(A0, LOW);
  delay(500);
}