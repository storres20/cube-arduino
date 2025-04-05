#include <SPI.h>
#include <LoRa.h>

#define SS_PIN    10
#define RST_PIN   9
#define DIO0_PIN  2
#define LED_PIN   3

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed. Check wiring.");
    while (true);
  }

  Serial.println("Transmitter ready.");
}

void loop() {
  String msg = "Hello";
  Serial.print("Sending: ");
  Serial.println(msg);

  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();

  delay(3000);

  if (LoRa.parsePacket()) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }

    received.trim(); // <- Important!
    Serial.print("Received echo: ");
    Serial.println(received);

    if (received == msg) {
      digitalWrite(LED_PIN, HIGH);
      delay(300);
      digitalWrite(LED_PIN, LOW);
    }
  }
}

