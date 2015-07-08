#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <SPI.h>
#include <SD.h>

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

Adafruit_BMP085 bmp;
int temp, pressure, altitude, startPressure;

const int chipSelect = 4;

void setup() {
  Serial.begin(38400);
  while(!Serial) {}  
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize device
  Serial.print("Initializing MPU6050...");
  accelgyro.initialize();
  // verify connection
  Serial.print("Testing MPU6050 connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  
  Serial.print("Initializing SD card...");
  pinMode(10,OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed or not present");
    while (1) {}
  }
  Serial.println("card initialized.");
  
  Serial.print("Calibrating altitude sensor...");
  long pressureSum = 0;
  for (int i = 0; i<100; i++) {
    pressureSum += bmp.readPressure();
  }
  startPressure = pressureSum/100;
  Serial.println("Current altitude is now 0");
}

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  temp = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude = bmp.readAltitude();
  
  String dataString;
  dataString = String(ax) + "\t" + String(gx) + "\t" + String(temp) + "\t" + String(pressure) + "\t" + String(altitude);
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    Serial.println(dataString);
  }
}
