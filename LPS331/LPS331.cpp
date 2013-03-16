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
  // 0b11100000
  // active mode, 12.5 Hz output data rate
  writeReg(LPS331_CTRL_REG1, 0xE0);
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
    
float LPS331::readPressure(void)
{
  return (float)readRawPressure() / 4096;
}

long LPS331::readRawPressure(void)
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
  return (int32_t)((int32_t)ph << 16 | pl << 8 | pxl);
}

float LPS331::readTemperature(void)
{
  return 42.5 + (float)readRawTemperature() / 480;
}

int LPS331::readRawTemperature(void)
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
  return (int16_t)(th << 8 | tl);
}
    
float LPS331::toAltitude(float pressure_mbar)
{
  return (1 - pow(pressure_mbar / 1013.25, 0.190284)) * 44307.694;
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