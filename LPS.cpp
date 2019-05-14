#include "LPS.h"
#include <Wire.h>

/*
 * For a rigorously complete implementation of the LPS25HB, see ST's github
 * version at https://github.com/stm32duino/LPS25HB
 */

// Defines ///////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define SA0_LOW_ADDRESS  0b1011100
#define SA0_HIGH_ADDRESS 0b1011101

#define TEST_REG_NACK -1

#define LPS331AP_WHO_ID 0xBB
#define LPS25H_WHO_ID   0xBD

// Constructors //////////////////////////////////////////////////////

LPS::LPS(void)
{
  _device = device_auto;

  // Pololu board pulls SA0 high, so default assumption is that it is
  // high
  address = SA0_HIGH_ADDRESS;
}

// Public Methods ////////////////////////////////////////////////////

// sets or detects device type and slave address; returns bool indicating success
bool LPS::init(deviceType device, byte sa0)
{
  if (!detectDeviceAndAddress(device, (sa0State)sa0))
    return false;

  switch (_device)
  {
    case device_25H:
      translated_regs[-INTERRUPT_CFG] = LPS25H_INTERRUPT_CFG;
      translated_regs[-INT_SOURCE]    = LPS25H_INT_SOURCE;
      translated_regs[-THS_P_L]       = LPS25H_THS_P_L;
      translated_regs[-THS_P_H]       = LPS25H_THS_P_H;
      return true;
      break;

    case device_331AP:
      translated_regs[-INTERRUPT_CFG] = LPS331AP_INTERRUPT_CFG;
      translated_regs[-INT_SOURCE]    = LPS331AP_INT_SOURCE;
      translated_regs[-THS_P_L]       = LPS331AP_THS_P_L;
      translated_regs[-THS_P_H]       = LPS331AP_THS_P_H;
      return true;
      break;
  }
  return false;
}

// turns on sensor and enables 1Hz continuous output
// WARNING, this is absolute, intended for init section.
void LPS::enable1Hz(void)
{
  if (_device == device_25H || _device == device_331AP)
  {
    // 0x90 = 0b10010000
    // PD = 1 (active mode);  ODR = 001b (1 Hz pressure & temperature output data rate)
    writeReg(CTRL_REG1, 0x90);
  }
}


// turns on sensor and enables continuous output at 12.5 Hz
// WARNING, this is absolute, intended for init section.
void LPS::enableDefault(void)
{
  if (_device == device_25H)
  {
    // 0xB0 = 0b10110000
    // PD = 1 (active mode);  ODR = 011 (12.5 Hz pressure & temperature output data rate)
    writeReg(CTRL_REG1, 0xB0);
  }
  else if (_device == device_331AP)
  {
    // 0xE0 = 0b11100000
    // PD = 1 (active mode);  ODR = 110 (12.5 Hz pressure & temperature output data rate)
    writeReg(CTRL_REG1, 0xE0);
  }
}

// sets the device to a specific FIFO mode
void LPS::enableFifo(fifoMode mode, bool decimate) {
  if (_device == device_25H || _device == device_331AP){

    // Set the correct FIFO_CTRL (2Eh)
    byte fiforeg = readReg(FIFO_CTRL);
    // The FIFO mode range is 3 bits wide at [7:5]
    fiforeg &= ~(0b111 << 5);
    fiforeg |= mode << 5;
    writeReg(FIFO_CTRL, fiforeg);

    // enable the FIFO mode in CTRL_REG2 (21h).
    byte ctrlreg = readReg(CTRL_REG2);
    bitSet(ctrlreg, 6);
    // You can also decimate the FIFO in 1Hz mode here if you raise bit 4.
    // Be sure to set the ODR to 0b001 in CTRL_REG1:[6:4] if you do. See enable1Hz() above.
    if (decimate) bitSet(ctrlreg, 4);
    writeReg(CTRL_REG2, ctrlreg);

  }
}

// writes register
void LPS::writeReg(int reg, byte value)
{
  // if dummy register address, look up actual translated address (based on device type)
  if (reg < 0)
  {
    reg = translated_regs[-reg];
  }

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// reads register
byte LPS::readReg(int reg)
{
  byte value;

  // if dummy register address, look up actual translated address (based on device type)
  if (reg < 0)
  {
    reg = translated_regs[-reg];
  }

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false); // restart
  Wire.requestFrom(address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}

// reads pressure in millibars (mbar)/hectopascals (hPa)
float LPS::readPressureMillibars(void)
{
  return (float)readPressureRaw() / 4096.0;
}

// reads pressure in inches of mercury (inHg)
float LPS::readPressureInchesHg(void)
{
  return (float)readPressureRaw() / 138706.5;
}

// reads pressure and returns raw 24-bit sensor output
int32_t LPS::readPressureRaw(void)
{
  Wire.beginTransmission(address);
  // assert MSB to enable register address auto-increment
  Wire.write(PRESS_OUT_XL | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)3);

  while (Wire.available() < 3);

  uint8_t pxl = Wire.read();
  uint8_t pl = Wire.read();
  uint8_t ph = Wire.read();

  // combine bytes
  return (int32_t)(int8_t)ph << 16 | (uint16_t)pl << 8 | pxl;
}

// reads temperature in degrees C
float LPS::readTemperatureC(void)
{
  return 42.5 + (float)readTemperatureRaw() / 480.0;
}

// reads temperature in degrees F
float LPS::readTemperatureF(void)
{
  return 108.5 + (float)readTemperatureRaw() / 480.0 * 1.8;
}

// reads temperature and returns raw 16-bit sensor output
int16_t LPS::readTemperatureRaw(void)
{
  Wire.beginTransmission(address);
  // assert MSB to enable register address auto-increment
  Wire.write(TEMP_OUT_L | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)2);

  while (Wire.available() < 2);

  uint8_t tl = Wire.read();
  uint8_t th = Wire.read();

  // combine bytes
  return (int16_t)(th << 8 | tl);
}

// converts pressure in mbar to altitude in meters, using 1976 US
// Standard Atmosphere model (note that this formula only applies to a
// height of 11 km, or about 36000 ft)
//  If altimeter setting (QNH, barometric pressure adjusted to standard sea
//  level) is given, this function returns an indicated altitude
//  compensated for actual regional pressure; otherwise, it returns
//  the pressure altitude above the standard pressure level of 1013.25
//  mbar or 29.921 inHg
float LPS::pressureToAltitudeMeters(float pressure_mbar, float altimeter_setting_mbar)
{
  return (1.0 - pow(pressure_mbar / altimeter_setting_mbar, 0.190263)) * 44330.8;
}

// converts pressure in inHg to altitude in feet; see notes above
float LPS::pressureToAltitudeFeet(float pressure_inHg, float altimeter_setting_inHg)
{
  return (1.0 - pow(pressure_inHg / altimeter_setting_inHg, 0.190263)) * 145442.0;
}

// Generic extension of the above two functions.
float LPS::pressureToAltitude(float pressure, float altimeter_setting, bool metric)
{
  const float factor = (metric ? 44330.8 : 145442.0);
  return (1.0 - pow(pressure / altimeter_setting, 0.190263)) * factor;
}

// Pressure (hPa), temp (C) and altitude (m), return sea level barometric pressure
// QFF is returned in millibars unless politely declined, in which case you get inHg
float LPS::altitudeToQFF(float altitude_meters, float tempC, bool please_return_mbars) {
  const float factor = (please_return_mbars ? 1.0 : 33.864);
  float pressure_mbar = LPS::readPressureMillibars();

  float qff = pressure_mbar /
        pow(((-0.0065 * altitude_meters) /
             ( 0.0065 * altitude_meters + 273.15 + tempC)), 5.2559);

  return (qff / factor);
}

/*
 * Use this when you first get your device. Tell it your local info and it will help
 * you see what your RPDS_L (0x39) and/or RPDS_H (0x3A) should be. Then you can use the
 * above to report accurately your local true sea level barometric pressure (QFF).
 */
int32_t LPS::opcHelper(float my_local_qff_mbar, float my_local_altitude_meters) {
  // real programmers would turn on BDU here, first.
  int32_t currentRawPressure = LPS::readPressureRaw();
  byte rpds_low, rpds_high;
  int16_t returnVal;

  rpds_low  = readReg(LPS::RPDS_L);
  rpds_high = readReg(LPS::RPDS_H);

  Serial.print("Your local pressure:\t\t");
  Serial.print(my_local_qff_mbar);
  Serial.print(" mbar, which is ");

  Serial.print((int32_t)(my_local_qff_mbar * 4096.0));
  Serial.println(" int32_t");

  Serial.print("Your local altitude:\t\t");
  Serial.print(my_local_altitude_meters);
  Serial.println(" m");

  Serial.println("According to the current device state, your local QFF should be:");
  Serial.println(LPS::altitudeToQFF(my_local_altitude_meters));

  Serial.print("Sensor raw pressure:\t\t");
  Serial.print(currentRawPressure);
  Serial.println(" int32_t");

  Serial.print("Sensor millibars:\t\t");
  Serial.print((float)currentRawPressure / 4096.0);
  Serial.println(" mbar");

  Serial.print("Current RPDS_L:\t\t");
  Serial.print(rpds_low, BIN);
  Serial.println(" binary");
  Serial.print("Current RPDS_H:\t\t");
  Serial.print(rpds_high, BIN);
  Serial.println(" binary");

  returnVal = (currentRawPressure - (int32_t)(my_local_qff_mbar * 4096.0));

  return returnVal;

}

// Private Methods ///////////////////////////////////////////////////

bool LPS::detectDeviceAndAddress(deviceType device, sa0State sa0)
{
  if (sa0 == sa0_auto || sa0 == sa0_high)
  {
    address = SA0_HIGH_ADDRESS;
    if (detectDevice(device)) return true;
  }
  if (sa0 == sa0_auto || sa0 == sa0_low)
  {
    address = SA0_LOW_ADDRESS;
    if (detectDevice(device)) return true;
  }

  return false;
}

bool LPS::detectDevice(deviceType device)
{
  int id = testWhoAmI(address);

  if ((device == device_auto || device == device_25H) && id == LPS25H_WHO_ID)
  {
    _device = device_25H;
    return true;
  }
  if ((device == device_auto || device == device_331AP) && id == LPS331AP_WHO_ID)
  {
    _device = device_331AP;
    return true;
  }

  return false;
}

int LPS::testWhoAmI(byte address)
{
  Wire.beginTransmission(address);
  Wire.write(WHO_AM_I);
  Wire.endTransmission();

  Wire.requestFrom(address, (byte)1);
  if (Wire.available())
    return Wire.read();
  else
    return TEST_REG_NACK;
}
