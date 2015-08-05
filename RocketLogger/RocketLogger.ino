#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <SPI.h>
#include <SD.h>
#include <FastLED.h>

const int chipSelect = 4;
const int RBF_PIN = 12;
const int GO_PIN = 13;

//RGB pins
const int RED_PIN = 5;
const int GREEN_PIN = 6;
const int BLUE_PIN = 11;

//Altitude min and max to indicate range of RGB color fade.
const int ALT_MIN = 0;
const int ALT_MAX = 100;

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

Adafruit_BMP085 bmp;
int temp = 0;
int pressure = 0;
int altitude = 0;
int startAlt = 0;
int maxAlt = 0;

int red = 0;
int green = 0;
int blue = 0;

void setup() {
  Serial.begin(9600);
  while(!Serial);
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  pinMode(GO_PIN, OUTPUT);
  pinMode(RBF_PIN, INPUT);
  
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  digitalWrite(GO_PIN, LOW);

  //Initialize 3 axis accelerometer/gyro.
  Serial.print("Initializing MPU6050...");
  accelgyro.initialize();
  
  //Verify connection to MPU6050.
  Serial.print("Testing MPU6050 connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  //Check for altitude/temperature sensor.
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }

  //Initialize SD card.
  Serial.print("Initializing SD card...");
  pinMode(10,OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed or not present");
    while (1) {}
  }
  Serial.println("card initialized.");

  //Calibrate altitude sensor.
  Serial.print("Calibrating altitude sensor...");
  long altitudeSum = 0;
  for (int i = 0; i<100; i++) {
    altitudeSum += bmp.readAltitude();
  }
  startAlt = altitudeSum/100;
  Serial.println("Current altitude is now 0");
  
  //Write csv Column titles
  File dataFile = SD.open("datalog.csv", FILE_WRITE);
  if(dataFile)
  {
    String dataString = "Time, Acc X, Acc Y, Acc Z, Gyro X, Gyro Y, Gyro Z, Temp, Pressure, Altitude, MaxAltitude";
    dataFile.println(dataString);
    dataFile.close();
    Serial.println(dataString);
  }

  //Indicate that all initializations are successful and data will begin recording.
  digitalWrite(GO_PIN, HIGH);
}

void loop() {

  //If the RBF jumper is attached, begin a color cycling routine (while blocking, so no SD writing).
  if(digitalRead(RBF_PIN))
  {
    //If we've just entered this 'if', then the RBF was just newly attached.
    digitalWrite(GO_PIN, LOW);
    
    //Wait for RBF jumper to be pulled before starting data collection.
    int hue = 0;
    while(digitalRead(RBF_PIN))
    {
      if(hue == 0) Serial.println("[[[[[[RBF Jumper is attached!]]]]]]"); //Occasionally notify of RBF
      setLED(CHSV(hue, 255, 255));
      hue = (hue + 1) % 256;
      delay(30);
    }

    //If we've exited the loop, then the RBF jumper has been pulled.
    digitalWrite(GO_PIN, HIGH);
    Serial.println("RBF Jumper is detached!");
  }
  
  //Get sensor values
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  temp = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude = bmp.readAltitude() - startAlt;

  /*
   * Fade from red to green as altitude increases
   */
  //Only use altitude if it's greater than the previous max altitude.  This locks greatest height.
  maxAlt = (altitude > maxAlt) ? altitude : maxAlt;
  
  //Fade red down as altitude increases.
  red = map(int(maxAlt), ALT_MIN, ALT_MAX, 255, 0);
  //Fade green up as altitude increases.
  green = map(int(maxAlt), ALT_MIN, ALT_MAX, 0, 255);

  //Set RGB LED values
  setLED(red, green, blue);
  /*
   * End LED code
   */
  Serial.println("Altitude: " + String(altitude) + ", MaxAlt: " + maxAlt + ", Red: " + String(red) + ", Green: " + String(green) + ", Blue: " + String(blue));
  
  //Write sensor values to "datalog.csv"
  String dataString;
  dataString = String(millis()) + "," + String(ax) + "," + String(ay) + "," + String(az) + "," + String(gx) + "," + String(gy) + "," + String(gz) + "," + String(temp) + "," + String(pressure) + "," + String(altitude) + "," + String(maxAlt);
  File dataFile = SD.open("datalog.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    //Serial.println(dataString);
  }
}

//Sets the RGB LEDs based on the CRGB object.
void setLED(const CRGB& rgb)
{
  setLED(rgb.r, rgb.g, rgb.b);
}

void setLED(int r, int g, int b)
{
  Serial.println("setLED(Red: " + String(r) + ", Green: " + String(g) + ", Blue: " + String(b) + ")");
  analogWrite(RED_PIN, r);
  analogWrite(GREEN_PIN, g);
  analogWrite(BLUE_PIN, b);
}
