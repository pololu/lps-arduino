#ifndef LPS331_h
#define LPS331_h

#include <Arduino.h> // for byte data type

class LPS331
{
  public:
    enum sa0State { sa0_low, sa0_high, sa0_auto };

    // register addresses
    // Note: Some of the register names in the datasheet are inconsistent
    // between Table 14 in section 6 and the register descriptions in
    // section 7. Where they differ, the names from section 7 have been
    // used here. 
    enum regAddr
    {
      REF_P_XL      = 0x08,
      REF_P_L       = 0x09,
      REF_P_H       = 0x0A,

      WHO_AM_I      = 0x0F,

      RES_CONF      = 0x10,

      CTRL_REG1     = 0x20,
      CTRL_REG2     = 0x21,
      CTRL_REG3     = 0x22,
      INTERRUPT_CFG = 0x23,
      INT_SOURCE    = 0x24,
      THS_P_L       = 0x25,
      THS_P_H       = 0x26,
      STATUS_REG    = 0x27,

      PRESS_OUT_XL  = 0x28,
      PRESS_OUT_L   = 0x29,
      PRESS_OUT_H   = 0x2A,

      TEMP_OUT_L    = 0x2B,
      TEMP_OUT_H    = 0x2C,

      AMP_CTRL      = 0x30,

      DELTA_PRESS_XL= 0x3C,
      DELTA_PRESS_L = 0x3D,
      DELTA_PRESS_H = 0x3E
    };

    LPS331(void);

    bool init(sa0State sa0 = sa0_auto);

    void enableDefault(void);

    void writeReg(byte reg, byte value);
    byte readReg(byte reg);

    float readPressureMillibars(void);
    float readPressureInchesHg(void);
    int32_t readPressureRaw(void);
    float readTemperatureC(void);
    float readTemperatureF(void);
    int16_t readTemperatureRaw(void);

    static float pressureToAltitudeMeters(float pressure_mbar, float altimeter_setting_mbar = 1013.25);
    static float pressureToAltitudeFeet(float pressure_inHg, float altimeter_setting_inHg = 29.9213);

  private:
    byte address;

    bool autoDetectAddress(void);
    bool testWhoAmI(void);
};

#endif