#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>

// === Pines ===
#define DHTPIN D2
#define DHTTYPE DHT11
#define VOLTAGE_PIN A0
#define SS_PIN    D10
#define RST_PIN   D9
#define DIO0_PIN  D8

// === Sensores ===
DHT dht(DHTPIN, DHTTYPE);
MPU9250_asukiaaa mySensor;
Adafruit_BMP280 bmp;

// === Tiempo de envío LoRa ===
unsigned long lastSendTime = 0;
const unsigned long interval = 3000; // cada 3 segundos

// === Variables para velocidad de descenso ===
float lastAltitude = 0;
unsigned long lastAltitudeTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();  // SDA/SCL por defecto

  dht.begin();
  analogReadResolution(12);

  // BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("⚠️ BMP280 not found!");
  }

  // MPU9250
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  // LoRa
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  if (!LoRa.begin(433E6)) {
    Serial.println("❌ LoRa init failed. Check wiring.");
    while (true);
  }

  Serial.println("✅ System ready: DHT11 + GY-91 + LoRa");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastSendTime >= interval) {
    lastSendTime = currentMillis;

    // === Leer sensores ===
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("⚠️ DHT11 read failed.");
      return;
    }

    int rawADC = analogRead(VOLTAGE_PIN);
    float adcVoltage = (rawADC / 4095.0) * 3.3;
    float measuredVoltage = adcVoltage * 5.0;  // Ajusta si tu divisor es diferente

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

    // === Orientación (ángulo respecto al norte) ===
    float heading = atan2(magY, magX);
    if (heading < 0) heading += 2 * PI;
    float headingDeg = heading * 180 / PI;

    // === Altitud y velocidad de descenso ===
    float currentAltitude = bmp.readAltitude(1013.25); // Ajusta según tu presión local
    unsigned long now = millis();
    float dt = (now - lastAltitudeTime) / 1000.0; // segundos
    float descentSpeed = 0;

    if (dt > 0) {
      descentSpeed = (lastAltitude - currentAltitude) / dt; // m/s
    }

    lastAltitude = currentAltitude;
    lastAltitudeTime = now;

    float bmpTemp = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100.0; // hPa

    // === Componer mensaje ===
    String message = "Volt:" + String(measuredVoltage, 2) + "V"; // voltaje de la batería,
    message += ",Descent:" + String(descentSpeed, 2) + "m/s"; // velocidad de descenso
    message += ",Temp:" + String(temperature, 1) + "C"; // temperatura desde KY-015
    message += ",BMP_T:" + String(bmpTemp, 1) + "C"; // temperatura desde GY-91 (BMP280)
    message += ",Pres:" + String(pressure, 1) + "hPa"; // presion GY-91 (BMP280)
    message += ",Hum:" + String(humidity, 1) + "%"; // humedad KY-015

    //velocidad angular (3 ejes) GY-91
    message += ",GyX:" + String(gyroX, 2) + "°/s";
    message += ",GyY:" + String(gyroY, 2) + "°/s";
    message += ",GyZ:" + String(gyroZ, 2) + "°/s";

    //aceleración (3 ejes) GY-91
    message += ",AccX:" + String(accelX, 2) + "m/s2";
    message += ",AccY:" + String(accelY, 2) + "m/s2";
    message += ",AccZ:" + String(accelZ, 2) + "m/s2";

    //intensidad del campo magnético (3 ejes) GY-91
    message += ",MagX:" + String(magX, 2) + "uT";
    message += ",MagY:" + String(magY, 2) + "uT";
    message += ",MagZ:" + String(magZ, 2) + "uT";

    message += ",Head:" + String(headingDeg, 1) + "°"; // orientación (ángulo con respecto al Norte)
    message += ",Alt:" + String(currentAltitude, 2) + "m"; // altura GY-91 (BMP280)

    // === Enviar por LoRa ===
    Serial.println("=================================");
    Serial.println("📡 Sending via LoRa:");
    Serial.println(message);

    LoRa.beginPacket();
    LoRa.print(message);
    LoRa.endPacket();
  }
}
