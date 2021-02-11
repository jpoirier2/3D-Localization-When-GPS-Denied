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
#include <Adafruit_LSM6DSO32.h>
#define ALTBASIS 6

// Basic demo for accelerometer/gyro readings from Adafruit LSM6DS33

// For SPI mode, we need a CS pin
#define LSM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11

// Power by connecting Vin to 3-5V, GND to GND
// Uses I2C - connect SCL to the SCL pin, SDA to SDA pin
// See the Wire tutorial for pinouts for each Arduino
// http://arduino.cc/en/reference/wire

MPL3115A2 baro3115;

Adafruit_LSM6DSO32 dso32;

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

  
  // acccelerometer set up 
 Serial.println("Adafruit LSM6DSO32 test!");

 // if (!dso32.begin_I2C()) {
     if (!dso32.begin_SPI(LSM_CS)) {
    // if (!dso32.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    // Serial.println("Failed to find LSM6DSO32 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DSO32 Found!");

  dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (dso32.getAccelRange()) {
  case LSM6DSO32_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DSO32_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DSO32_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  case LSM6DSO32_ACCEL_RANGE_32_G:
    Serial.println("+-32G");
    break;
  }

  // dso32.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  Serial.print("Gyro range set to: ");
  switch (dso32.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DSO32
  }

  // dso32.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (dso32.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  // dso32.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (dso32.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }
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


// acelerometer loop
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  dso32.getEvent(&accel, &gyro, &temp);
  
  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();
   //  // serial plotter friendly format

  //  Serial.print(temp.temperature);
  //  Serial.print(",");

  //  Serial.print(accel.acceleration.x);
  //  Serial.print(","); Serial.print(accel.acceleration.y);
  //  Serial.print(","); Serial.print(accel.acceleration.z);
  //  Serial.print(",");

  // Serial.print(gyro.gyro.x);
  // Serial.print(","); Serial.print(gyro.gyro.y);
  // Serial.print(","); Serial.print(gyro.gyro.z);
  // Serial.println();
  //  delayMicroseconds(10000);
  delay(250);
}
