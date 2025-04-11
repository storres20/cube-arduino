#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>

// === Pin Definitions ===
#define DHTPIN D2
#define DHTTYPE DHT11

#define VOLTAGE_PIN A0     // GPIO1 (A0 on Nano ESP32)

#define SS_PIN    D10
#define RST_PIN   D9
#define DIO0_PIN  D8

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);

  // Initialize DHT11
  dht.begin();

  // Set ADC resolution (12-bit: 0–4095)
  analogReadResolution(12);

  // Initialize LoRa
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  if (!LoRa.begin(433E6)) {
    Serial.println("❌ LoRa init failed. Check wiring.");
    while (true);
  }

  Serial.println("✅ System ready: DHT11 + Voltage + LoRa");
}

void loop() {
  // === Read DHT11 Sensor ===
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("⚠️ DHT11 read failed.");
    return;
  }

  // === Read Voltage Sensor ===
  int rawADC = analogRead(VOLTAGE_PIN);
  float adcVoltage = (rawADC / 4095.0) * 3.3;  // Convert to ADC voltage
  float measuredVoltage = adcVoltage * 5.0;    // Voltage divider scale

  // === Compose message ===
  String message = "Temp:" + String(temperature, 1) + "C";
  message += ",Hum:" + String(humidity, 1) + "%";
  message += ",Volt:" + String(measuredVoltage, 2) + "V";

  // === Print to Serial Monitor ===
  Serial.println("=================================");
  Serial.println("📡 Sending via LoRa:");
  Serial.println(message);

  // === Send via LoRa ===
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();

  delay(3000); // Wait 3 seconds between messages
}
