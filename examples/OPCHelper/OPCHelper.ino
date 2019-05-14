#include <Wire.h>
#include <LPS.h>

LPS ps;

/***
 * So, you have a brand new LPS sensor and you have soldered on the pins.
 * Now you need to see if it reports true pressure readings. I assume you
 * know your true altitude. I assume, also, that you can find an accurate
 * local pressure reading from nearby weather stations or extrapolating
 * from your weather authority's isoline maps. Using those two "true" values
 * and the pressure from the sensor, we should be able to show what you ought
 * to use for setting the RPDS registers.

 * Why do you want to set the RPDS registers? Glad you asked.
 * If your sensor is not going to move in any vertical direction, say, it's
 * stuck on a pole in your backyard, then it is easy to report your local
 * "weather" pressure by adjusting those registers. Set those in your init
 * section and then your pressure values will show your adjusted sea level
 * pressure.

 * If you just want to report your actual air pressure, then just move along
 * because this function may not be helpful. But you still ought to calibrate
 * your unit to make sure that it's reporting accurately.
***/

/* Assign these two values from your current local references */
static const float pressure = 1016.4;
static const float altitude = 419.7;

bool delaying(int msdelay){
  delay(msdelay);
  return true;
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  if (!ps.init())
  {
    Serial.println("Failed to autodetect pressure sensor!");
    while (delaying(60));
  }

  Serial.println("Pololu LPS OPC Helper");

  delay(100);
  ps.enableDefault();

  delay(100);
  
}

void loop()
{
  ps.opcHelper(pressure, altitude);

  delay(90000);
}
