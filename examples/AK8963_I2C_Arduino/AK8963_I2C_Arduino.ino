/*
 * AK8963 library - AK8963_I2C_Arduino.ino
 *
 * This example uses AK8963 in I2C mode with the Arduino's TwoWire
 * library.
 *
 * Created May 4, 2018
 * by Vasil Kalchev
 *
 * https://github.com/VasilKalchev/AK8963
 *
 */

#include <AK8963.hpp>
#include <Wire.h>

// The I2C address of the device.
const byte magAddress = 0x0C;

AK8963 mag(magAddress);

/* This are the callback functions that have to be implemented and
   attached to the AK8963 object for it to work. The functions have
   to implement burst read/write using I2C or SPI. */
bool readI2C(uint8_t device, uint8_t registerAddress,
             uint8_t *data, uint8_t size);
bool writeI2C(uint8_t device, uint8_t registerAddress,
              uint8_t *data, uint8_t size);


void setup() {
  Serial.begin(250000);
  Wire.begin();
  Wire.setClock(400000);

  // The callback functions have to be attached to the object.
  mag.attachInterface(readI2C, writeI2C);
  // The AK8963 also needs a delay function (in microseconds).
  mag.attachDelay(delayMicroseconds);

  mag.initialize();
  if (!mag.checkConnection()) Serial.println("Error connecting to AK8963!");
  mag.resolution(BITS_16);
}

void loop() {
  /* The magnetometer can be used in single measurement mode,
     continuous measurement mode and externally triggered measurement
     mode. In this can we are gonna use the single measurement mode. */
  mag.startMeasurement();
  while (getDataReady() == false); // Wait until measrement is ready.
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  if (mag.read(x, y, z)) {
    /* The measurement data is 'raw'. AK8963::to_ut(float mag);
       converts raw data to microteslas. */
    Serial.print("X: "); Serial.print(mag.to_ut(x));
    Serial.print(", Y: "); Serial.print(mag.to_ut(y));
    Serial.print(", Z: "); Serial.print(mag.to_ut(z));
    Serial.println();
  } else {
    if (mag.getOverflow()) Serial.println("Magnetic field too big!");
    else Serial.println("Sensor reading error. Check wiring.");
  }
}


// Interface callback functions:
// =============================
// Burst-read I2C function using Arduino's TwoWire library.
bool readI2C(uint8_t device, uint8_t registerAddress,
             uint8_t *data, uint8_t size) {
  const int8_t bufferSize = 32; // TwoWire library buffer size.
  // Read in steps of buffer size.
  for (uint8_t i = 0; i < size; i += min(size, bufferSize)) {
    Wire.beginTransmission(device);
    Wire.write(registerAddress);
    Wire.endTransmission(false);
    uint8_t retBytes = Wire.requestFrom(device, (uint8_t)min(size - i, bufferSize));
    if (retBytes == size) {
      for (uint8_t i = 0; i < size; ++i) {
        data[i] = Wire.read();
      }
    } else {
      return false;
    }
  }
  Wire.endTransmission();
  return true;
}

// Burst-write I2C function using Arduino's TwoWire library.
bool writeI2C(uint8_t device, uint8_t registerAddress,
              uint8_t *data, uint8_t size) {
  Wire.beginTransmission(device);
  Wire.write(registerAddress);
  for (int8_t i = 0; i < size; ++i) {
    Wire.write((uint8_t)data[i]);
  }
  return Wire.endTransmission() == 0;
}
