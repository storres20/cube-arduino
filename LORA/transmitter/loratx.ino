#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>

// === Pin Definitions ===
#define DHTPIN D2
#define DHTTYPE DHT11
#define VOLTAGE_PIN A0

#define SS_PIN    D10
#define RST_PIN   D9
#define DIO0_PIN  D8

DHT dht(DHTPIN, DHTTYPE);
MPU9250_asukiaaa mySensor;
Adafruit_BMP280 bmp;

void setup() {
  Serial.begin(115200);
  Wire.begin();  // Initialize I2C

  dht.begin();
  analogReadResolution(12);

  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  if (!LoRa.begin(433E6)) {
    Serial.println("❌ LoRa init failed. Check wiring.");
    while (true);
  }

  if (!bmp.begin(0x76)) {
    Serial.println("⚠️ BMP280 not found!");
  }

  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  Serial.println("✅ System ready: DHT11 + GY-91 + LoRa");
}

void loop() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("⚠️ DHT11 read failed.");
    return;
  }

  int rawADC = analogRead(VOLTAGE_PIN);
  float adcVoltage = (rawADC / 4095.0) * 3.3;
  float measuredVoltage = adcVoltage * 5.0;

  mySensor.accelUpdate();
  mySensor.gyroUpdate();
  mySensor.magUpdate();

  float accelX = mySensor.accelX();
  float accelY = mySensor.accelY();
  float accelZ = mySensor.accelZ();

  float gyroX = mySensor.gyroX();
  float gyroY = mySensor.gyroY();
  float gyroZ = mySensor.gyroZ();

  float magX = mySensor.magX();
  float magY = mySensor.magY();
  float magZ = mySensor.magZ();

  float bmpTemp = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0; // hPa

  // === Compose message ===
  String message = "Temp:" + String(temperature, 1) + "C";
  message += ",Hum:" + String(humidity, 1) + "%";
  message += ",Volt:" + String(measuredVoltage, 2) + "V";
  message += ",AccX:" + String(accelX, 2);
  message += ",AccY:" + String(accelY, 2);
  message += ",AccZ:" + String(accelZ, 2);
  message += ",GyX:" + String(gyroX, 2);
  message += ",GyY:" + String(gyroY, 2);
  message += ",GyZ:" + String(gyroZ, 2);
  message += ",MagX:" + String(magX, 2);
  message += ",MagY:" + String(magY, 2);
  message += ",MagZ:" + String(magZ, 2);
  message += ",BMP_T:" + String(bmpTemp, 1) + "C";
  message += ",Pres:" + String(pressure, 1) + "hPa";

  // === Print and Send ===
  Serial.println("=================================");
  Serial.println("📡 Sending via LoRa:");
  Serial.println(message);

  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();

  delay(3000);
}
