#include <SPI.h>
#include <Wire.h>
#include <Math.h>
#include <RH_RF95.h>
#include "MPL3115A2.h"

#define ALTBASIS 2


MPL3115A2 baro3115;


// Feather M0 pinout
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define LED 13

// Setting frequency (MHz)
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Device ID character
char ID[] = "B"; // This is a char array instead of a char because I spent an hour and a half trying to make it work as a char and it was a massive pain in my ass

float altitude = 0;

void broadcast(char *msg, int len, bool includeID = false);
// Broadcasts a signal via RF95
// msg is a pointer to a message
// len contains the length of msg (can be obtained using sizeof(msg)/sizeof(msg[0]))
// includeID determines whether the device ID is included at the beginning of the message

void blinkyBlinky(); // Literally just blinks the light to save space in main

void setup() {
  // serial setup and radio hard-reset
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT); // Sets the reset pin to output
  digitalWrite(RFM95_RST, HIGH); // Writes high to the reset pin

  Wire.begin();        // Join i2c bus

  Serial.begin(115200); // Starts serial transmission at a baud rate of 115200
  // Waits for the serial console to be opened
  // REMOVE THIS IF THE BOARD ISN'T CONNECTED TO A COMPUTER
  while (!Serial) {
    delay(1); // wait
  }
  delay(100);


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

  // RFM95 reset
  digitalWrite(RFM95_RST, LOW); // Resets RFM95
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("Something's wrong. I can feel it.\nLoRa radio init failed"); // Error detection that I decided to leave in
    while (1); // Stops the program in its tracks
  }
  Serial.println("LoRa radio successfully initialized");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed"); // Checks if a frequency is not sent
    while (1);
  }
  Serial.print("Broadcast frequency: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false); // Sets power level to 23 dBm with RFO disabled
  digitalWrite(LED, LOW);

}

uint8_t count = 0; // Will count the number of messages sent

void loop() {
  // put your main code here, to run repeatedly:
  
  outData();
  char message[50] = ""; // Contains the message that will be sent
  //  dtostrf(altitude,4,4,message);

  //ftoa(altitude,message,10);
  //dtostrf(altitude,3,3,message);
  int whole = floor(altitude);
  int decimal = floor(altitude * 100) - whole * 100;
  char mbuff[10] = "";
  itoa(whole, mbuff, 10);
  strcat(message, mbuff);
  strcat(message, ".");
  itoa(decimal, mbuff, 10);
  strcat(message, mbuff);

  int l = sizeof(message) / sizeof(message[0]);
  //itoa(count++, message+(l-15), 10);
  delay(100); // Wait 1 second between transmissions (unsure if this is a requirement)de
  broadcast(message, l, true);
  // blinkyBlinky(); // Blinky blinky
}

void broadcast(char *msg, int len, bool includeID) {
  delay(10); // Sample code does this. Probably wouldn't hurt to wait a bit before transmission. We can remove this later if needed
  char msgB[len + 2]; // Char that includes the ID designation and the broadcasted message
  if (includeID) {
    //msgB[0] = ID[0];
    //msgB[1] = '|';
    strcpy(msgB, ID);
    strcat(msgB, "|");
    strcat(msgB, msg);
    /*for (int i = 2; i<len+2; i++) {
      msgB[i] = msg[i-2];
      }*/

   // Serial.print("Sending "); Serial.println(msgB);
    rf95.send((uint8_t *)msgB, sizeof(msgB));
  }
  else {
    //Serial.print("msg size: "); Serial.println(sizeof(msg)/sizeof(char));
    //Serial.print("msgB size: "); Serial.println(sizeof(msgB)/sizeof(char));
    strcpy(msgB, msg);
 //   Serial.print("Sending "); Serial.println(msg);
    rf95.send((uint8_t *)msgB, sizeof(msgB));
    //char realmsg[] = "Hello there";
    //rf95.send((uint8_t *)realmsg, sizeof(realmsg));
  }
 // Serial.println("Waiting for packet to complete...");
 // delay(10);
  rf95.waitPacketSent(); // Waits for the package to send
  Serial.println("Packet completed");
}

void doCalibration()
{

  Serial.println("Starting pressure calibration...");

  baro3115.runCalibration(ALTBASIS);

  Serial.print("calculated sea level pressure: ");
  Serial.print(baro3115.calculated_sea_level_press, 2);
  Serial.println(" Pa");

  Serial.print("calculated elevation_offset: ");
  Serial.print(baro3115.elevation_offset, 2);
  Serial.println(" Pa");

  baro3115.setModeStandby();    // <-- this one starts a config sequence
  baro3115.setModeBarometer();
  baro3115.setBarometricInput(baro3115.calculated_sea_level_press);
  baro3115.setOversampleRate(7);
  baro3115.enableEventFlags();
  baro3115.setModeActive();   // <-- this one ends the sequence and starts the measurement mode

  Serial.println("Pressure calibration completed.");

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
  // Serial.print("Pressure(hPa): ");
  //Serial.print(pressure, 2);

  float temperature = baro3115.readTemp();
  // Serial.print(" Temperature(C): ");
  // Serial.print(temperature, 2);

  baro3115.setModeStandby();
  baro3115.setModeAltimeter();
  baro3115.setOversampleRate(7);
  baro3115.enableEventFlags();
  baro3115.setModeActive();
  altitude = baro3115.readAltitude();

  // Serial.print(" Altitude above sea(m): ");
  // Serial.print(altitude, 2);
  //Serial.println();
}

void blinkyBlinky() {
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
  delay(100);
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
}
