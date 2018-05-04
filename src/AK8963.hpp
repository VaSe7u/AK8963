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


@file       AK8963.hpp
@author     Vasil Kalchev
@date       2018
@version    0.1.0
@copyright  The MIT License
@brief      AK8963 3-axis electronic compass I2C and SPI library

@todo:
*/

#pragma once
#include <stdint.h>

/*!
 @brief АК8963 class.

 Initial mode: POWER_DOWN.
 I2C max clock: standard mode 100kHz, fast mode 400kHz.
 When changing modes first set POWER_DOWN then 100us wait then mode.
 Sensor overflow: |X|+|Y|+|Z| >= 4912uT.
 CSB pin selects SPI when low and I2C when high.

 Sensitivity:
  0.6 µT/LSB typ. (14-bit)
  0.15µT/LSB typ. (16-bit)
 */
class AK8963 {
public:
  /*!
   @brief Constructor.

   The I2C address of AK8963 is selected by setting pins CAD0 and
   CAD1 to VCC or GND. CAD1 changes address' bit 2 and CAD0 changes
   address' bit 1. VCC sets the corresponding bit to 0 and GND to 1.


   @note R/W set to 0 is write, 1 is read.

   @param[in] address: the I2C address of the device.
   */
  /*
   0 0 0 1 1 CAD1 CAD0 R/W.
   CAD1 CAD0 Address
   0    0    0x0CH
   0    1    0x0DH
   1    0    0x0EH
   1    1    0x0FH
   */
  AK8963(uint8_t address);

  /// @name Public methods
  ///@{
  /*!
   @brief Attach read and write functions.

   In order for this library to be platform independent and to
   support both I2C and SPI interfaces, it uses callback functions
   to perform read and write operations.
   The functions signature is:
   `bool functionName(uint8_t device, uint8_t registerAddress,
                      uint8_t *data, uint8_t size);` Where 'device'
   is either the I2C address of the device or some ID of the device
   when using SPI, 'registerAddress' is the register that will be
   used for reading/writing, '*data' is an uint8_t pointer holding
   the data that will be read/written, 'size' is the size of the data
   in bytes.
   The functions have to implement burst reads/writes using an SPI or
   I2C interface. In case of SPI the function also has to select the
   chip (CS pin).

   @param[in] *read: Pointer to the read function.
   @param[in] *write: Pointer to the write function.
   */
  void attachInterface(bool (*read)(uint8_t device, uint8_t registerAddress, uint8_t *data, uint8_t size),
                       bool (*write)(uint8_t device, uint8_t registerAddress, uint8_t *data, uint8_t size));

  /*!
   @brief Attach delay microseconds function.

   @param[in] *delay: Pointer to the delay microseconds function.
   */
  void attachDelay(void (*delay)(uint32_t us));

  /*!
   @brief Initialize the device.

   */
  void initialize();

  /*!
   @brief Read the device's ID.

   @details Register: WIA - 0x48.

   @return Device's ID.
   */
  uint8_t getDeviceId() const;

  /*!
   @brief Check if device is connected.

   @return Device connection status.
   @retval True if device is connected.
   */
  bool checkConnection() const;

  /*!
   @brief Device information for AKM.

   @details Register: INFO[7:0]

   @return Information byte.
   */
  uint8_t information() const;

  /*!
   @brief Check if there is new measurement data.

   @note DRDY bit turns to “1” when data is ready in single measurement
   mode, continuous measurement mode 1, 2, external trigger
   measurement mode or self-test mode. It returns to “0” when any one
   of ST2 register or measurement data register (HXL~HZH) is read.

   @details Register: ST1[DRDY]

   @return Data ready status.
   @retval True when new sensor data is available.
   */
  bool getDataReady() const;

  /*!
   @brief Check if sensor data was overrun.

   @note DOR bit turns to “1” when data has been skipped in continuous
   measurement mode or external trigger measurement mode. It returns
   to “0” when any one of ST2 register or measurement data register
   (HXL~HZH) is read.

   @details Register: ST1[DOR]

   @return Data overrun status.
   @retval True if sensor data was overrun.
   */
  bool getDataOverrun() const;

  /*!
   @brief Check if sensor overflow occured on last read.

   In single measurement mode, continuous measurement mode, external
   trigger measurement mode and self-test mode, magnetic sensor may
   overflow even though measurement data regiseter is not saturated.
   In this case, measurement data is not correct and should be
   ignored.

   @details Register: ST2[HOFL]

   @return Sensor overflow status.
   @retval True if sensor overflow occured.
   */
  bool getOverflow() const;


  /*!
   @brief Mode of operation enumerator.
   */
  enum Mode : uint8_t {
    SINGLE_MEASUREMENT = 0b0001,
    CONTINUOUS_MEASUREMENT_8HZ = 0b0010,
    CONTINUOUS_MEASUREMENT_SLOW = 0b0010,
    CONTINUOUS_MEASUREMENT_100HZ = 0b0110,
    CONTINUOUS_MEASUREMENT_FAST = 0b0110,
    EXTERNAL_TRIGGER_MEASUREMENT = 0b0100,
  };

  /*!
   @brief Mode of operation setter.

   There are three types of modes of operation: single measurement,
   continuous measurement and externally triggered measurement:
     SINGLE_MEASUREMENT - Measurements are triggered one by one.
     CONTINUOUS_MEASUREMENT_8HZ - Measurements are triggered
     automatically with frequency of 8Hz.
     CONTINUOUS_MEASUREMENT_SLOW - Same as above.
     CONTINUOUS_MEASUREMENT_100HZ - Measurements are triggered
     automatically with frequency of 100Hz.
     CONTINUOUS_MEASUREMENT_FAST - Same as above.
     EXTERNAL_TRIGGER_MEASUREMENT - Measurements are triggered on the
     rising edge of TRG pin, triggering is ignored while measuring.

   @details Register: CNTL1[3:0]
   @see enum Mode

   @param[in] mode: Mode of operation.
   */
  void mode(const Mode mode);

  /*!
   @brief Mode of operation getter.

   @details Register: CNTL1[3:0]
   @see void mode(const Mode mode)
   @see enum Mode

   @return Mode of operation.
   @retval enum Mode.
   */
  Mode getMode() const;

  /*!
   @brief Power down the device.

   @details Register: CNTL1[3:0]
   */
  void powerDown();

  /*!
   @brief Start a single measurement.

   @details Register: CNTL1[3:0]
   */
  void startMeasurement();

  /*!
   @brief Resolution enumerator.
   */
  enum Resolution : uint8_t {
    BITS_14 = 0,
    BITS_16 = 1,
  };

  /*!
   @brief Measurement resolution setter.

   The size of the measurement can be set to 14 or 16 bits. Using
   more bits for the measurement data increases its resolution.
   Resolution at 14 bits is 0.6uT/LSB and resolution at 16 bits is
   0.15uT/LSB.

   @details Register: CNTL1[BIT]
   @see enum Resolution

   @param[in] resolution: measurement's resolution.
   */
  void resolution(const Resolution resolution);
  /*!
   @brief Resolution getter.

   @details Register: CNTL1[BIT]
   @see void mode(const Resolution resolution)
   @see enum Resolution

   @return Measurement resolution.
   @retval enum Resolution.
   */
  Resolution getResolution() const;

  /*!
   @brief Soft reset.

   All registers are reinitialized.

   @details Register: CNTL2[SRST]
   */
  void softReset() const;

  /*!
   @brief Self test.

   @details Register: ASTC[SELF]

   @return Self test result.
   @retval True if self test was successful.
   */
  bool selfTest();

  /*!
   @brief Disable I2C bus interface.

   Disables I2C bus interface (enabled by default). To reenable I2C
   bus interface, reset AK8963.

   @details Register: I2CDIS[7:0]
   */
  void disableI2c();

  /*!
   @brief Read measurement data.

   Reads the measured magnetic fields along the three axis (x, y and
   z) from the device and writes the values into the supplied by
   reference parameters (int16_t &x, int16_t &y and int16_t &z).

   @details Register: from HXL to HZH

   @param[out] &x: magnetic field along x-axis.
   @param[out] &y: magnetic field along y-axis.
   @param[out] &z: magnetic field along z-axis.

   @return Status of operation.
   @retval True if read was successful.
   */
  bool read(float &x, float &y, float &z);

  /*!
   @brief Read measurement data.

   Reads the measured magnetic fields along the three axis (x, y and
   z) from the device and writes the values into the supplied by
   reference array (float (&mag)[3]).

   @details Register: from HXL to HZH

   @param[out] &mag[]: array holding the three magnetic fields.

   @return Status of operation.
   @retval True if read was successful.
   */
  bool read(float (&mag)[3]);

  /*!
   @brief Convert raw magnetic field data into microteslas.

   @param[in] mag: the raw magnetic field to be converted.
   @return Magnetic field in microteslas.
   */
  float to_ut(float mag) const;
  ///@}

private:
  uint8_t _device = 0;
  float _xSensAdj = 0.0f;
  float _ySensAdj = 0.0f;
  float _zSensAdj = 0.0f;
  float _sensitivity_ut = 0.6f;
  bool _overflow = false;

  bool (*_read)(uint8_t device, uint8_t registerAddress, uint8_t *data, int8_t size) = nullptr;
  bool (*_write)(uint8_t device, uint8_t registerAddress, uint8_t *data, int8_t size) = nullptr;
  void (*_delay)(uint32_t time) = nullptr;

  void _initializeSensitivityAdjustment();

  static bool _readBit(const Register registerAddress, bool *const bit,
                       const Bit position) const;
  static bool _writeBit(const Register registerAddress, const bool bit,
                        const Bit position);
  static bool _readBits(const Register registerAddress, uint8_t *const bits,
                        const Bit firstBit, const Length length) const;
  static bool _writeBits(const Register registerAddress, const uint8_t bits,
                         const Bit firstBit, const Length length);

  bool _readBytes(const Register registerAddress, uint8_t *const bytes,
                  const int8_t size = 1) const;
  bool _writeBytes(const Register registerAddress, const uint8_t *const byte,
                   const int8_t size = 1);

  enum class Register : uint8_t {
    WIA =    0x00,  // Device ID (R)
    INFO =   0x01,  // Information (R)
    ST1 =    0x02,  // Status 1 (R) - Data status
    XOUT_L = 0x03,  // Measurement data (R) - X
    XOUT_H = 0x04,  // Measurement data (R) - X
    YOUT_L = 0x05,  // Measurement data (R) - Y
    YOUT_H = 0x06,  // Measurement data (R) - Y
    ZOUT_L = 0x07,  // Measurement data (R) - Z
    ZOUT_H = 0x08,  // Measurement data (R) - Z
    ST2 =    0x09,  // Status 2 (R) - Data status
    CNTL1 =  0x0A,  // Control 1 (R/W) - Function control
    CNTL2 =  0x0B,  // Control 2 (R/W) - Function control
    ASTC =   0x0C,  // Self-test (R/W)
    I2CDIS = 0x0F,  // I2C disable (R/W)
    ASAX =   0x10,  // X-axis sensitivity adjustment value (R) - Fuse ROM
    ASAY =   0x11,  // Y-axis sensitivity adjustment value (R) - Fuse ROM
    ASAZ =   0x12,  // Z-axis sensitivity adjustment value (R) - Fuse ROM
  };
  enum class Bit : uint8_t {
    // ST1
    DOR =  1, // Data overrun
    DRDY = 0, // Data ready

    // ST2
    BITM = 4, // Output bit setting (mirror) | "0": 14bit, "1": 16bit
    HOFL = 3, // Magnetic sensor overflow (if 1 - wrong data)

    // CNTL1
    BIT =  4, // Output bit setting | "0": 14bit, "1": 16bit
    MODE = 3, // Operation mode setting [3:0]

    // CNTL2
    SRST = 0, // Soft reset | "0": Normal, "1": Reset

    // ASTC
    SELF = 6, // Self test control | "0": Normal, "1": ST
  };
  enum class Length : uint8_t {
    MODE = 4, // Operation mode setting [3:0]
  };
  enum : uint8_t {
    ADDRESS_CAD00 = 0x0C, // I2C address
    ADDRESS_CAD01 = 0x0D,
    ADDRESS_CAD10 = 0x0E,
    ADDRESS_CAD11 = 0x0F,
    ID = 0x48,
    DISABLE_I2C = 0b00011011, // Writing this byte to I2CDIS disables I2C
  };
  enum PrivateMode : uint8_t {
    POWER_DOWN = 0b0000,
    SELF_TEST = 0b1000,
    FUSE_ROM_ACCESS = 0b1111,
    // INVALID_MODE = 0b1010,
  };
};