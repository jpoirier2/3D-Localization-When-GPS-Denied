/* Example implementation of an alarm using DS3231
 *
 * VCC and GND of RTC should be connected to some power source
 * SDA, SCL of RTC should be connected to SDA, SCL of arduino
 * SQW should be connected to CLOCK_INTERRUPT_PIN
 * CLOCK_INTERRUPT_PIN needs to work with interrupts
 */

#include <RTClib.h>
#include <Wire.h>

#include <SPI.h>
#include <RH_RF95.h>

RTC_DS3231 rtc;

// ESP32 w/ Wing pinout
 #define RFM95_RST     27   // "A"
 #define RFM95_CS      33   // "B"
 #define RFM95_INT     12   //  next to A

#define LED 13

// Setting frequency (MHz)
#define RF95_FREQ 915.0

// the pin that is connected to SQW
#define CLOCK_INTERRUPT_PIN 14

// Receiver ID
char ID[] = "B"; // Defined as a char array for consistency with the transmitter code

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


void receiveRaw(char **msg);
// recieves a raw LoRa transmission (Includes ID)
// msg is a pointer to an array with the max message length

bool isIdentified(char *msg, int len); 
// Determines if the message contained in msg is identified or not
// msg is a pointer to the message, len is the message's length 
// NOTE: Function will return true for an unidentified message if msg[0] is a letter and msg[1] = "|"

char getID(char *msg);
// Returns the ID of an identified message
// msg is the identified message


//Global Variables
double startCounter;
double midCounter;
double counterCPU;
double overhead = 23585814;

void setup() {
  // put your setup code here, to run once:

  pinMode(LED, OUTPUT); // Setting inputs and outputs
  pinMode(RFM95_RST, OUTPUT);

  digitalWrite(LED,HIGH);
    Serial.begin(115200);

    // initializing the rtc
    if(!rtc.begin()) {
        Serial.println("Couldn't find RTC!");
        Serial.flush();
        abort();
    }
    

        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    
    
    //we don't need the 32K Pin, so disable it
    rtc.disable32K();
    
    // Making it so, that the alarm will trigger an interrupt
    pinMode(CLOCK_INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CLOCK_INTERRUPT_PIN), onAlarm, FALLING);
    
    // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
    // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
    rtc.clearAlarm(1);
    rtc.clearAlarm(2);
    
    // stop oscillating signals at SQW Pin
    // otherwise setAlarm1 will fail
    rtc.writeSqwPinMode(DS3231_OFF);
    
    // turn off alarm 2 (in case it isn't off already)
    // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
    rtc.disableAlarm(2);
    
    // schedule an alarm 10 seconds in the future
    if(!rtc.setAlarm1(
            rtc.now(),
            DS3231_A1_PerSecond // this mode triggers the alarm every second. See Doxygen for other options
    )) {
        Serial.println("Error, alarm wasn't set!");
    }else {
        Serial.println("Alarm will happen every second!");  
    }

    // END RTC Setup
    delay(100);
    // BEGIN LoRa Wing Setup
    // Reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

while (!rf95.init()) {
    Serial.println("Something's wrong, I can feel it.\nLoRa radio init failed.");
    while (1);  
  }
  Serial.println("LoRa radio successfully initialized");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while(1);  
  }
  Serial.print("Listening frequency: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
  
  digitalWrite(LED, LOW);

  // LoRa Wing Initialization Complete
  
}

void loop() {
  // put your main code here, to run repeatedly:
    char *message;
    char date[10] = "hh:mm:ss";
  int len;
  digitalWrite(LED, HIGH);
  receiveRaw(&message);
  digitalWrite(LED, LOW);
  len = strlen(message);
  if (isIdentified(message, len) && (getID(message) == ID[0])) {
    getCycles();
    Serial.print("Raw message received: "); Serial.println(message);
    rtc.now().toString(date);
    Serial.print(date);
    Serial.print("\nCycles Between Seconds: "); Serial.println(counterCPU); 
  }
   
    if(rtc.alarmFired(1)) {
        rtc.clearAlarm(1);
    }
}

void onAlarm() {
startCounter = ESP.getCycleCount();
midCounter = 0;
counterCPU = 0;
}

void receiveRaw(char **msg) {
    if (rf95.waitAvailableTimeout(1000)) {
      char *rec = (char*)malloc(RH_RF95_MAX_MESSAGE_LEN);
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; // Creating a buffer and defining its length
      uint8_t len = sizeof(buf);
  
      if (rf95.recv(buf, &len)) {
        digitalWrite(LED, HIGH); 
        //RH_RF95::printBuffer("Received: ", buf, len);
        //Serial.print("Got: "); Serial.println((char*)buf);
        strcpy(rec, (char*)buf);
        *msg = rec;
      }
      else {
        Serial.println("Receive failed");  
      }
    }
    else {
      *msg = "No message";  
    }
}

void getCycles(){
  midCounter = ESP.getCycleCount();
  counterCPU = midCounter - startCounter - 2*overhead;
}

bool isIdentified(char *msg, int len) {
  return (isalpha(msg[0])&&(msg[1]=='|'));  
}

char getID(char *msg) {
  return msg[0];  
}
