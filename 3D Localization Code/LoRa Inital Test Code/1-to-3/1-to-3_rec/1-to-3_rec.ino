#include <SPI.h>
#include <RH_RF95.h>

// Feather M0 pinout
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define LED 13

// Setting frequency (MHz)
#define RF95_FREQ 915.0

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

void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT); // Setting inputs and outputs
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH); // Resetting radio

  Serial.begin(115200); // Initiating serial
  // Waiting for serial console to be opened
  // IF THE MICROCONTROLLER IS NOT CONNECTED TO A COMPUTER, COMMENT THIS OUT
  while(!Serial) {
    delay(1);  
  }
  delay(100);

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
}

void loop() {
  // put your main code here, to run repeatedly:
  char *message;
  digitalWrite(LED, HIGH);
  receiveRaw(&message);
  digitalWrite(LED, LOW);
  Serial.print("Raw message received: "); Serial.println(message);
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

bool isIdentified(char *msg, int len) {
  return (isalpha(msg[0])&&(msg[1]=='|'));  
}

char getID(char *msg) {
  return msg[0];  
}
