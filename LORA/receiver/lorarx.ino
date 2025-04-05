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

  Serial.println("Receiver ready.");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }

    Serial.print("Received: ");
    Serial.println(incoming);

    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);

    delay(10); // small delay to give LoRa module time to reset RX state
    // Echo it back
    LoRa.beginPacket();
    LoRa.print(incoming);
    LoRa.endPacket();

    Serial.println("Echoed back.");
  }
}
