#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <SPI.h>
#include <SD.h>
#include <FastLED.h>

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

Adafruit_BMP085 bmp;
int temp, pressure, altitude, startPressure;

int hue;
double azAvg = 0, decayFactor = 0.99;


const int chipSelect = 4;

#define REDPIN    5
#define GREENPIN  6
#define BLUEPIN   11

void setLED( const CRGB& rgb) {
  analogWrite(REDPIN,   rgb.r);
  analogWrite(GREENPIN, rgb.g);
  analogWrite(BLUEPIN,  rgb.b);
}

void setup() {
  pinMode(REDPIN,   OUTPUT);
  pinMode(GREENPIN, OUTPUT);
  pinMode(BLUEPIN,  OUTPUT);
  
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
}

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  azAvg = azAvg*decayFactor + (1 - decayFactor)*az;
  hue = map(int(azAvg), -16384, 16384, 0, 255);
  setLED(CHSV(hue, 255, 255));

}
