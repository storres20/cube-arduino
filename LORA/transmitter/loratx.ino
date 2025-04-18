#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// === Pines ===
#define DHTPIN D2
#define DHTTYPE DHT11
#define VOLTAGE_PIN A0
#define SS_PIN    D10
#define RST_PIN   D9
#define DIO0_PIN  D8

#define GPS_RX    D4  // TX from GPS
#define GPS_TX    D3  // RX from GPS (opcional)

#define SEA_LEVEL_PRESSURE_HPA 1011.0

// === Sensores ===
DHT dht(DHTPIN, DHTTYPE);
MPU9250_asukiaaa mySensor;
Adafruit_BMP280 bmp;
TinyGPSPlus gps;
HardwareSerial GPSSerial(1); // UART1

// === Tiempo de envío LoRa ===
unsigned long lastSendTime = 0;
const unsigned long interval = 3000; // (Send data every 3 secs)

// === Variables para velocidad de descenso ===
float lastAltitude = 0;
unsigned long lastAltitudeTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();  // SDA/SCL por defecto

  dht.begin();
  analogReadResolution(12);

  if (!bmp.begin(0x76)) {
    Serial.println("⚠️ BMP280 not found!");
  }

  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  if (!LoRa.begin(433E6)) {
    Serial.println("❌ LoRa init failed. Check wiring.");
    while (true);
  }

  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("✅ System ready: DHT11 + GY-91 + GPS + LoRa");
}

void loop() {
  // Leer datos del GPS
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }

  unsigned long currentMillis = millis();
  if (currentMillis - lastSendTime >= interval) {
    lastSendTime = currentMillis;

    // Leer sensores
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

    float heading = atan2(magY, magX);
    if (heading < 0) heading += 2 * PI;
    float headingDeg = heading * 180 / PI;

    float currentAltitude = bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA);
    unsigned long now = millis();
    float dt = (now - lastAltitudeTime) / 1000.0;
    float descentSpeed = 0;
    if (dt > 0) {
      descentSpeed = (lastAltitude - currentAltitude) / dt;
    }

    lastAltitude = currentAltitude;
    lastAltitudeTime = now;

    float bmpTemp = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100.0;

    // === GPS Data ===
    String lat = gps.location.isValid() ? String(gps.location.lat(), 6) : "NaN";
    String lon = gps.location.isValid() ? String(gps.location.lng(), 6) : "NaN";
    String altGPS = gps.altitude.isValid() ? String(gps.altitude.meters(), 1) : "NaN";

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

    // GPS data
    message += ",Lat:" + lat;
    message += ",Lon:" + lon;
    message += ",AltGPS:" + altGPS + "m";

    // === Enviar por LoRa ===
    Serial.println("=================================");
    Serial.println("📡 Sending via LoRa:");
    Serial.println(message);

    LoRa.beginPacket();
    LoRa.print(message);
    LoRa.endPacket();
  }
}
