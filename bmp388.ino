#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

//define SEALEVELPRESSURE_HPA (1013.25)
#define SEALEVELPRESSURE_HPA (995.01)

Adafruit_BMP3XX bmp;

double baroKalmanP = 2.2f,
       baroKalmanQ = 0.05f,
       baroKalmanR = 20.0f,
       baroKalmanG,
       baroKalmanX = 100.0f,
       // alternalive parameter 
       baroKalman[5] = {1.3f, 0.06f, 10.0f, 0.0f, 150.0f};
       //baroKalman[5] = {2.2f, 0.06f, 10.0f, 0.0f, 100.0f};

void setup() {
  Serial.begin(115200);
  bmp.begin_I2C();
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  bmp.performReading();
}

void loop() {
  bmp.performReading();

  float readAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.println("-----");
  Serial.print(readAlt);

  baroKalman[0] = baroKalman[0] + baroKalman[1];

  baroKalman[3] = baroKalman[0] / (baroKalman[0] + baroKalman[2]);
  baroKalman[4] = baroKalman[4] + baroKalman[3] * (readAlt - baroKalman[4]);
  baroKalman[0] = (1 - baroKalman[3]) * baroKalman[0];
  readAlt = baroKalman[4];

  Serial.println("*****");
  Serial.print(readAlt);
  delay(20);
}
