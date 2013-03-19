#include <LPS331.h>
#include <Wire.h>

#define LPS331AP_ADDRESS_SA0_LOW  0b1011100
#define LPS331AP_ADDRESS_SA0_HIGH 0b1011101

bool LPS331::init(byte sa0)
{
  switch(sa0)
  {
    case LPS331_SA0_LOW:
      address = LPS331AP_ADDRESS_SA0_LOW;
      return true;
      
    case LPS331_SA0_HIGH:
      address = LPS331AP_ADDRESS_SA0_HIGH;
      return true;

    default:
      return autoDetectAddress();
  }
}

void LPS331::enableDefault(void)
{
  // active mode, 12.5 Hz output data rate
  writeReg(LPS331_CTRL_REG1, 0b11100000);
}
    
void LPS331::writeReg(byte reg, byte value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

byte LPS331::readReg(byte reg)
{
  byte value;

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false); // restart
  Wire.requestFrom(address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();

  return value;
}
    
float LPS331::readPressureMillibars(void)
{
  return (float)readPressureRaw() / 4096;
}

float LPS331::readPressureInchesHg(void)
{
  return (float)readPressureRaw() / 138706.5;
}

long LPS331::readPressureRaw(void)
{
  Wire.beginTransmission(address);
  // assert the MSB of the address to get the pressure sensor
  // to do slave-transmit subaddress updating.
  Wire.write(LPS331_PRESS_OUT_XL | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)3);

  while (Wire.available() < 3);

  uint8_t pxl = Wire.read();
  uint8_t pl = Wire.read();
  uint8_t ph = Wire.read();

  // combine bytes
  return (int32_t)ph << 16 | (uint16_t)pl << 8 | pxl;
}

float LPS331::readTemperatureC(void)
{
  return 42.5 + (float)readTemperatureRaw() / 480;
}

float LPS331::readTemperatureF(void)
{
  return 108.5 + (float)readTemperatureRaw() / 480 * 1.8;
}

int LPS331::readTemperatureRaw(void)
{
  Wire.beginTransmission(address);
  // assert the MSB of the address to get the pressure sensor
  // to do slave-transmit subaddress updating.
  Wire.write(LPS331_TEMP_OUT_L | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)2);

  while (Wire.available() < 2);

  uint8_t tl = Wire.read();
  uint8_t th = Wire.read();

  // combine bytes
  return (int16_t)th << 8 | tl;
}

float LPS331::pressureToAltitudeMeters(float pressure_mbar, float altimeter_setting_mbar)
{
  return (pow(altimeter_setting_mbar / 1013.25, 0.190263) - pow(pressure_mbar / 1013.25, 0.190263)) * 44330.8;
}

float LPS331::pressureToAltitudeFeet(float pressure_inHg, float altimeter_setting_inHg)
{
  return (pow(altimeter_setting_inHg / 29.9213, 0.190263) - pow(pressure_inHg / 29.9213, 0.190263)) * 145442;
}

bool LPS331::autoDetectAddress(void)
{
  // try each possible address and stop if reading WHO_AM_I returns the expected response
  address = LPS331AP_ADDRESS_SA0_LOW;
  if (readReg(LPS331_WHO_AM_I) == 0xBB) return true;
  address = LPS331AP_ADDRESS_SA0_HIGH;
  if (readReg(LPS331_WHO_AM_I) == 0xBB) return true;

  return false;
}