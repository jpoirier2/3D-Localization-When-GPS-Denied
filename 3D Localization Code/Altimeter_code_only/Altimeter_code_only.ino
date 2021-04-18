/**************************************************************************/
/*!
    @file     Adafruit_MPL3115A2.cpp
    @author   K.Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    Example for the MPL3115A2 barometric pressure sensor

    This is a library for the Adafruit MPL3115A2 breakout
    ----> https://www.adafruit.com/products/1893

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0 - First release
*/
/**************************************************************************/

#include <Wire.h>

#include "MPL3115A2.h"
#define ALTBASIS 6


MPL3115A2 baro3115;

void doCalibration()
{
  //calculate pressure for current ALTBASIS altitude by averaging a few readings
  Serial.println("Starting pressure calibration...");

  // this function calculates a new sea level pressure ref value
  // I adde this function to the original library and it will NOT change the sensor registers
  // see below setBarometricInput() where that value is actually set
  // in the registers. the sensor will start using it just after.
  baro3115.runCalibration(ALTBASIS);

  Serial.print("calculated sea level pressure: ");
  Serial.print(baro3115.calculated_sea_level_press, 2);
  Serial.println(" Pa");

  Serial.print("calculated elevation_offset: ");
  Serial.print(baro3115.elevation_offset, 2);
  Serial.println(" Pa");

  // i originally had big errors in pressure and altitude readouts,
  // once i added the elevation_offset calculation the error
  // decreased and is now close to that of another barometer
  // (whose manual calibration for altitude should be checked anyway)

  // I initally implemented this code without using the calls to setModeStandby() and setModeActive()
  // the results were polluted by occasional weird behavior with exceedingly large and wrong values.
  // Also, the temperature measurements were constantly increasing, probably due to the 
  // lower oversample rate and faster reading loop
  // I decided to compare the various libraries on github for this sensor and found out that
  // both Sparkfun's and Adafruit's versions did NOT use a specific calls sequence when setting
  // mode and registers !
  // Michael Lange's sequence instead, is the one that finally gave me correct, smooth and repeatable measurement sessions. See below:
  
  baro3115.setModeStandby();    // <-- this one starts a config sequence
  baro3115.setModeBarometer();
  baro3115.setBarometricInput(baro3115.calculated_sea_level_press);
  baro3115.setOversampleRate(7);
  baro3115.enableEventFlags();
  baro3115.setModeActive();   // <-- this one ends the sequence and starts the measurement mode

  // calibration is now completed
  //
  // setBarometricInput() :
  // This configuration option calibrates the sensor according to
  // the sea level pressure for the measurement location (2 Pa per LSB)
  // The default value for "Barometric input for Altitude calculation" is 101,326 Pa

  // About the oversample rate:
  // Set the # of samples from 1 to 128. See datasheet.
  // Integer values between 0 < n < 7 give oversample ratios 2^n and 
  // sampling intervals of 0=6 ms , 1=10, 2=18, 3=34, 4=66, 5=130, 6=258, and 7=512
  // Seems that the suggested value is 7 and the measurement could take 512ms
  // I'm using it with good results, but I'm still trying to understand why the time taken
  // for the pressure reading is just 3ms (measured with the millis() function)
  
  // add temperature offset for my tested sensor
  // seems the temperature probe is within the ADC and should not be used
  // to measure the environment. Will try adding the offset i got by comparison
  // with another thermometer
  // you can enable this if you need it:
  // baro3115.setModeStandby();
  // baro3115.setOffsetTemperature((char) (0.65 / 0.0625) );
  // baro3115.setModeActive();

  Serial.println("Pressure calibration completed.");

  // let's have a look, just in case:
  //Serial.println("OFFSETS:");
  //Serial.print("  pressure: ");
  //Serial.println(baro3115.offsetPressure() ,2);
  //Serial.print("  altitude: ");
  //Serial.println((float)baro3115.offsetAltitude() ,2);
  //Serial.print("  temperature(C): ");
  //Serial.println(baro3115.offsetTemperature() ,2);

}

void setup() {
  Serial.begin(9600);
  Wire.begin();        // Join i2c bus
  Serial.begin(115200);  // Start serial for output
while (!Serial)
    delay(10);
  baro3115.begin(); // Get sensor online

  baro3115.setModeStandby();    // <-- this one starts a config sequence
  baro3115.setModeBarometer();
  baro3115.setOffsetPressure(0);
  baro3115.setOffsetTemperature(0);
  baro3115.setOffsetAltitude(0);
  baro3115.setBarometricInput(0.0);
  baro3115.elevation_offset = 0;
  baro3115.calculated_sea_level_press = 0;
  baro3115.setOversampleRate(7);
  baro3115.enableEventFlags();
  baro3115.setModeActive();   // <-- this one ends the sequence and starts the measurement mode

  Serial.println("BEFORE calibration...");

  Serial.println("Adafruit_MPL3115A2 test!");
  doCalibration();
  outData();

  
}
void outData()
{
  baro3115.setModeStandby();
  baro3115.setModeBarometer();
  baro3115.setOversampleRate(7);
  baro3115.enableEventFlags();
  baro3115.setModeActive();
  // when we are using the calibration then we also have to add the
  // calculated elevation related pressure offset to our readings:
  float pressure = baro3115.readPressure() + baro3115.elevation_offset;

  // output is in Pa
  // 1 kPa = 10 hPa = 1000 Pa
  // 1 hPa = 100 Pascals = 1 mb
  pressure = (pressure / 100) ;   //  ... / 1000 * 10 ;
  Serial.print("Pressure(hPa): ");
  Serial.print(pressure, 2);

  float temperature = baro3115.readTemp();
  Serial.print(" Temperature(C): ");
  Serial.print(temperature, 2);

  baro3115.setModeStandby();
  baro3115.setModeAltimeter();
  baro3115.setOversampleRate(7);
  baro3115.enableEventFlags();
  baro3115.setModeActive();
  float altitude = baro3115.readAltitude();
  Serial.print(" Altitude above sea(m): ");
  Serial.print(altitude, 2);

  Serial.println();
}

 
void loop() {
//  if (! baro3115.begin()) {
//    Serial.println("Couldnt find sensor");
//    return;
//  }
  

  float pascals = baro3115.readPressure();
  // Our weather page presents pressure in Inches (Hg)
  // Use http://www.onlineconversion.com/pressure.htm for other units
  Serial.print(pascals/3377); Serial.println(" Inches (Hg)");

  float altm = baro3115.readAltitude();
  Serial.print(altm); Serial.println(" meters");

  float tempC = baro3115.readTemp();
  Serial.print(tempC); Serial.println("*C");

  delay(250);
}
