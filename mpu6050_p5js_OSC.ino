#include <WiFi.h>
#include <ArduinoOSCWiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Wi-Fi configuration
const char* ssid = "HQ4856";
const char* password = "P4n69851";

// OSC configuration
const char* oscAddress = "192.168.137.1"; 
const int oscPort = 1234;                 

// MPU6050 configuration
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected!");

  // Initialize MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("MPU6050 sensor not detected!");
    while (1);
  }

  Serial.println("MPU6050 initialized successfully");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  static unsigned long lastTime = 0;

  // Send OSC message every 100 milliseconds
  if (millis() - lastTime > 100) {
    lastTime = millis();

    // Get MPU6050 data
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Scale accelerometer and gyroscope data
    float accelMultiplier = 50.0;
    float gyroMultiplier = 50.0;

    float accelX = accel.acceleration.x * accelMultiplier;
    float accelY = accel.acceleration.y * accelMultiplier;
    float accelZ = accel.acceleration.z * accelMultiplier;

    float gyroX = gyro.gyro.x * gyroMultiplier;
    float gyroY = gyro.gyro.y * gyroMultiplier;
    float gyroZ = gyro.gyro.z * gyroMultiplier;

    // Send OSC messages
    OscWiFi.send(oscAddress, oscPort, "/mpu6050/accel", accelX, accelY, accelZ);
    OscWiFi.send(oscAddress, oscPort, "/mpu6050/gyro", gyroX, gyroY, gyroZ);

    // Output debug information
    Serial.println("Sending accelerometer data: X=" + String(accelX) + ", Y=" + String(accelY) + ", Z=" + String(accelZ));
    Serial.println("Sending gyroscope data: X=" + String(gyroX) + ", Y=" + String(gyroY) + ", Z=" + String(gyroZ));
  }
}
