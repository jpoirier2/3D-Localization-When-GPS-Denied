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

// Device ID character
char ID[] = "B"; // This is a char array instead of a char because I spent an hour and a half trying to make it work as a char and it was a massive pain in my ass

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

  Serial.begin(115200); // Starts serial transmission at a baud rate of 115200
  // Waits for the serial console to be opened
  // REMOVE THIS IF THE BOARD ISN'T CONNECTED TO A COMPUTER
  while (!Serial) {
    delay(1); // wait
 }
  delay(100);

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
    while(1);
  }
  Serial.print("Broadcast frequency: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false); // Sets power level to 23 dBm with RFO disabled
  digitalWrite(LED, LOW);
}

uint8_t count = 0; // Will count the number of messages sent

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000); // Wait 1 second between transmissions (unsure if this is a requirement)
  char message[] = "hello everyone #                "; // Contains the message that will be sent
  int l = sizeof(message)/sizeof(message[0]);
  itoa(count++, message+(l-15), 10);
  broadcast(message, l, true);
  blinkyBlinky(); // Blinky blinky
}

void broadcast(char *msg, int len, bool includeID) {
  delay(10); // Sample code does this. Probably wouldn't hurt to wait a bit before transmission. We can remove this later if needed
  char msgB[len+2]; // Char that includes the ID designation and the broadcasted message
  if (includeID) {
    //msgB[0] = ID[0];
    //msgB[1] = '|';
    strcpy(msgB, ID);
    strcat(msgB, "|");
    strcat(msgB, msg);
    /*for (int i = 2; i<len+2; i++) {
      msgB[i] = msg[i-2];
    }*/
    
    Serial.print("Sending "); Serial.println(msgB);
    rf95.send((uint8_t *)msgB, sizeof(msgB));  
  }
  else {
    //Serial.print("msg size: "); Serial.println(sizeof(msg)/sizeof(char));
    //Serial.print("msgB size: "); Serial.println(sizeof(msgB)/sizeof(char));
    strcpy(msgB, msg);
    Serial.print("Sending "); Serial.println(msg);
    rf95.send((uint8_t *)msgB, sizeof(msgB));
    //char realmsg[] = "Hello there";
    //rf95.send((uint8_t *)realmsg, sizeof(realmsg));
  }
  Serial.println("Waiting for packet to complete...");
  delay(10);
  rf95.waitPacketSent(); // Waits for the package to send
  Serial.println("Packet completed");
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
