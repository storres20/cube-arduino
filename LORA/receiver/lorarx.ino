#include <SPI.h>
#include <LoRa.h>

// === LoRa Pin Definitions ===
#define SS_PIN    D10
#define RST_PIN   D9
#define DIO0_PIN  D8

// Optional LED for message indicator
#define LED_PIN   D2  // You can connect an LED here if desired

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // Initialize LoRa
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  if (!LoRa.begin(433E6)) {
    Serial.println("❌ LoRa init failed. Check wiring.");
    while (true);
  }

  Serial.println("✅ LoRa Receiver Ready");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    String received = "";

    while (LoRa.available()) {
      received += (char)LoRa.read();
    }

    received.trim();

    // === Print to Serial Monitor ===
    Serial.println("=================================");
    Serial.println("📥 Message received via LoRa:");
    Serial.println(received);

    // Blink LED on message receive
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
  }
}
