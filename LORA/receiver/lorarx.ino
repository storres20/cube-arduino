#include <SPI.h>
#include <LoRa.h>

// === LoRa Pin Definitions ===
#define SS_PIN    D10
#define RST_PIN   D9
#define DIO0_PIN  D8

// Optional LED for message indicator
#define LED_PIN   D2

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

    // Blink LED to show reception
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);

    // === Print formatted data ===
    Serial.println("=================================");
    Serial.println("📥 LoRa Message Received:");
    printFormattedData(received);
  }
}

void printFormattedData(const String& data) {
  // Optional: print raw data for debugging
  // Serial.println(data);

  int start = 0;
  int index = 0;

  while ((index = data.indexOf(',', start)) != -1) {
    String part = data.substring(start, index);
    Serial.print("🔹 " + part + " ");
    start = index + 1;
  }

  // Print the last part if it exists
  if (start < data.length()) {
    Serial.print("🔹 " + data.substring(start));
  }

  Serial.println(); // Final line break
}
