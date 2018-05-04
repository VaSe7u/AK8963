/**
The MIT License (MIT)

Copyright (c) 2016 Vasil Kalchev

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


@file       AK8963.cpp
@author     Vasil Kalchev
@date       2018
@version    0.1.0
@copyright  The MIT License
@brief      AK8963 3-axis electronic compass I2C and SPI library
*/


AK8963::AK8963(uint8_t address) : _address(address) {}

void AK8963::attachInterface(bool (*read)(uint8_t device, uint8_t registerAddress, uint8_t *data, uint8_t size),
                             bool (*write)(uint8_t device, uint8_t registerAddress, uint8_t *data, uint8_t size)) {
  _read = read;
  _write = write;
}

void AK8963::attachDelay(void (*delay)(uint32_t us)) {
  _delay = delay;
}

void AK8963::initialize() {
  softReset();
  _initializeSensitivityAdjustment();
}

uint8_t AK8963::getDeviceId() const {
  uint8_t deviceId = 0;
  _readBytes(Register::WIA, &deviceId);
  return deviceId;
}

bool AK8963::checkConnection() const {
  return (getDeviceId() == ID);
}

uint8_t AK8963::information() const {
  uint8_t info = 0;
  _readBytes(Register::INFO, &info);
  return info;
}

bool AK8963::getDataReady() const {
  bool dataReady = false;
  _readBit(Register::ST1, *dataReady, Bit::DRDY);
  return dataReady;
}

bool AK8963::getDataOverrun() const {
  bool dataOverrun = false;
  _readBit(Register::ST1, *dataOverrun, Bit::DOR);
  return dataOverrun;
}

bool AK8963::getOverflow() const {
  return _overflow;
}

void AK8963::mode(const Mode mode) {
  powerDown();
  _delay(100);
  _writeBits(Register::CNTL1, (uint8_t)mode, Bit::MODE, Length::MODE);
}

Mode AK8963::getMode() const {
  uint8_t mode = 0;
  _readBits(Register::CNTL1, &mode, Bit::MODE, Length::MODE);
  return (Mode)mode;
}

void AK8963::powerDown() {
  _writeBits(Register::CNTL1, (uint8_t)POWER_DOWN, Bit::MODE, Length::MODE);
}

void AK8963::startMeasurement() {
  _writeBits(Register::CNTL1, (uint8_t)SINGLE_MEASUREMENT, Bit::MODE, Length::MODE);
}

void AK8963::resolution(const Resolution resolution) {
  _writeBit(Register::CNTL1, (bool)resolution, Bit::BIT);
  if (resolution == Resolution::BITS14) {
    _sensitivity_ut = 0.6f;
  } else if (resolution == Resolution::BITS16) {
    _sensitivity_ut = 0.15f;
  }
}

Resolution AK8963::getResolution() const {
  bool resolution = 0;
  _readBit(Register::CNTL1, &resolution, Bit::BIT);
  return (Resolution)resolution;
}

void AK8963::softReset() const {
  _writeBit(Register::CNTL2, true, Bit::SRST);
}

bool AK8963::selfTest() {
  bool pass = true;
  Mode mode = getMode();
  powerDown();
  delay(100);
  _writeBit(Register::ASTC, true, Bit::SELF);
  _writeBits(Register::CNTL1, (uint8_t)SELF_TEST, Bit::MODE, Length::MODE);
  float x = 0;
  float y = 0;
  float z = 0;
  while (read(&x, &y, &z) == false);
  _writeBit(Register::ASTC, false, Bit::SELF);
  powerDown();
  uint16_t range = 0;
  Resolution resolution = getResolution();
  if (resolution == BITS_14) {
    range = 50;
    if (z < -800 || z > -200) pass = false;
  } else if (resolution == BITS_16) {
    range = 200;
    if (z < -3200 || z > -800) pass = false;
  } else {
    pass = false;
  }
  if (x < -range || x > range || y < -range || y > range) pass = false;
  mode(mode);
  return pass;
}

void AK8963::disableI2c() {
  _writeByte(Register::I2CDIS, 0b00011011);
}

bool AK8963::read(float &x, float &y, float &z) {
  uint8_t data[8];
  if (_readBytes(Register::ST1, data, 8)) {
    if ((data[0] & (1 << (uint8_t)Bit::DRDY) == 1)) { // Data is ready
      if ((data[7] & (1 << (uint8_t)Bit::HOFL)) == 0) { // No overflow
        x = ( (float) ( ((int16_t)_buffer[2] << 8) | (_buffer[1]) )) * _xSensAdj;
        y = ( (float) ( ((int16_t)_buffer[4] << 8) | (_buffer[3]) )) * _ySensAdj;
        z = ( (float) ( ((int16_t)_buffer[6] << 8) | (_buffer[5]) )) * _zSensAdj;
        _overflow = false;
        return true;
      } // overflow (Eut > 4912uT)
      _overflow = true;
    } // data not ready
  } // couldn't read registers
  return false;
}

bool AK8963::read(float (&mag)[3]) {
  uint8_t data[8];
  if (_readBytes(Register::ST1, data, 8)) {
    if ((data[0] & (1 << (uint8_t)Bit::DRDY) == 1)) { // Data is ready
      if ((data[7] & (1 << (uint8_t)Bit::HOFL)) == 0) { // No overflow
        mag[0] = ( (float) ( ((int16_t)_buffer[2] << 8) | (_buffer[1]) )) * _xSensAdj;
        mag[1] = ( (float) ( ((int16_t)_buffer[4] << 8) | (_buffer[3]) )) * _ySensAdj;
        mag[2] = ( (float) ( ((int16_t)_buffer[6] << 8) | (_buffer[5]) )) * _zSensAdj;
        _overflow = false;
        return true;
      } // overflow (Eut > 4912uT)
      _overflow = true;
    } // data not ready
  } // couldn't read registers
  return false;
}

float AK8963::to_ut(int16_t mag) const {
  return (float)mag * _sensitivity_ut;
}

void AK8963::_initializeSensitivityAdjustment() {
  powerDown();
  delay(100);
  _writeBits(Register::CNTL1, (uint8_t)FUSE_ROM_ACCESS, Bit::MODE, Length::MODE);
  uint8_t data[3];
  _readBytes(Register::ASAX, data, 3);
  _xSensAdj = ((float)data[0] - 128.0f) / 256.0f + 1.0f;
  _ySensAdj = ((float)data[1] - 128.0f) / 256.0f + 1.0f;
  _zSensAdj = ((float)data[2] - 128.0f) / 256.0f + 1.0f;
  powerDown();
}


bool AK8963::_readBit(const Register registerAddress, bool *const bit,
                       const Bit position) const {
  uint8_t reg = 0;
  if (_readBytes(registerAddress, &reg)) {
    *bit = reg & (1 << (uint8_t)bit);
    return true;
  } else {
    return false;
  }
}
bool AK8963::_writeBit(const Register registerAddress, const bool bit,
                        const Bit position) {
  uint8_t reg = 0;
  if (_readBytes(registerAddress, &reg)) {
    if (bit == true) {
      reg |= (1 << (uint8_t)position);
    } else {
      reg &= (1 << (uint8_t)position);
    }
    return _writeBytes(registerAddress, &reg);
  }
}

bool AK8963::_readBits(const Register registerAddress, uint8_t *const bits,
                        const Bit firstBit, const Length length) const {
  uint8_t reg;
  if (_readBytes(registerAddress, &reg)) {
    uint8_t shift = (bitStart - length) + 1; // shift to correct position
    // bitStart begins from zero, length begins from one
    uint8_t mask = ((1 << length) - 1) << shift; // holds 1s where the bits will be changed
    reg &= mask; // clear the other bits
    reg >>= shift; // shift the target bits to the right
    *bits = reg;
    return true;
  } else {
    return false;
  }
}
bool AK8963::_writeBits(const Register registerAddress, const uint8_t bits,
                         const Bit firstBit, const Length length) {
  uint8_t reg; // register
  if (_readBytes(registerAddress, &reg)) {
    uint8_t shift = (bitStart - length) + 1;
    uint8_t mask = ((1 << length) - 1) << shift;
    bits <<= shift; // shifts bits to the correct position in the register
    bits &= mask; // clamp numbers bigger than 2^length
    reg &= ~(mask); // clear the bits that will be changed
    reg |= bits; // set bits in register according to bits
    return _writeBytes(registerAddress, reg);
  } else {
    return false;
  }
}

bool AK8963::_readBytes(const Register registerAddress, uint8_t *const bytes,
                         const int8_t size) const {
  return _read(_device, (uint8_t)registerAddress, byte, size);
}
bool AK8963::_writeBytes(const Register registerAddress, const uint8_t *const byte,
                          const int8_t size) {
  return _write(_device, (uint8_t)registerAddress, byte, size);
}