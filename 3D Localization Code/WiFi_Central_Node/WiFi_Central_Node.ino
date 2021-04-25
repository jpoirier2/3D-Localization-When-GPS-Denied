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
#include <WiFi.h>
#include <HTTPClient.h>

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


// Wifi Information
const char* ssid = "Ana WiFi";
const char* password = "5196295abc";

//Your Domain name with URL path or IP address with path
const char* serverName = "http://10.0.0.172:8080/track";

// Receiver ID
char ID[] = "1"; // Defined as a char array for consistency with the transmitter code


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
int64_t counterCPU;
double overhead = 23585814;
double temp = 0;
uint16_t firstMessage = 0;
uint8_t countUnsentMessage = 0; //Counts the number of messages that have not been sent to serial monitor or webserver


//char date[10] = "hh:mm:ss";
char Central_TS[50] = "";
char Altitude[25] = "";
char finalMessage[255] = "";


// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)
unsigned long timerDelay = 5000;

void setup() {
  // put your setup code here, to run once:

  pinMode(LED, OUTPUT); // Setting inputs and outputs
  pinMode(RFM95_RST, OUTPUT);

  digitalWrite(LED,HIGH);
    Serial.begin(115200);

      WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

    // initializing the rtc
    if(!rtc.begin()) {
        Serial.println("Couldn't find RTC!");
        Serial.flush();
        abort();
    }
    

   if(rtc.lostPower()) {
        // this will adjust to the date and time at compilation
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    
    
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
  digitalWrite(LED, HIGH);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  char *message;
  int len;
  
  receiveRaw(&message);
  //digitalWrite(LED, LOW);
  len = strlen(message);
  if (isIdentified(message, len)){
  //Serial.print("ID is: "); Serial.print(getID(message));Serial.print(" "); Serial.println(isalpha(getID(message)));
  if(isDigit(getID(message))) {
    int i = strlen(Central_TS);
    Central_TS[i++] = ';';
    Central_TS[i] = '\0';
    temp = counterCPU;
    strcpy(Altitude,message);
   // Serial.println(Altitude);
    
   // Serial.print(date);
   // Serial.print("\nCycles Between Seconds: "); Serial.println(counterCPU); 
  }
  else if(isalpha(getID(message))){
   // Serial.println("In else If");
    strcat(finalMessage,message);
    int i = strlen(finalMessage);
    finalMessage[i++] = ',';
    finalMessage[i] = '\0';
//    firstMessage++;
//    if(firstMessage == 1){
//      char buf[2] = "";
//      uint8_t hh = 00;
//      uint8_t mm = 00;
//      uint8_t ss = 00;
//      buf[0] = message[2];
//      buf[1] = message[3];
//      hh = atoi(buf);
//      buf[0] = message[5];
//      buf[1] = message[6];
//      mm = atoi(buf);
//      buf[0] = message[8];
//      buf[1] = message[9];
//      ss = atoi(buf);
//      Serial.print("RTC Old Time:"); Serial.println(rtc.now().toString(date));
//    rtc.adjust(DateTime((uint16_t)2021,(uint8_t)4,(uint8_t)22,hh,mm,ss));
//    Serial.print("RTC new Time:"); Serial.println(rtc.now().toString(date));
//    }
    countUnsentMessage++;
  }
  
  }

  if(countUnsentMessage == 3){
    countUnsentMessage = 0;

     if(WiFi.status()== WL_CONNECTED){
      HTTPClient http;
      
      // Your Domain name with URL path or IP address with path
      http.begin(serverName);

      // Specify content-type header
      //http.addHeader("Content-Type", "application/x-www-form-urlencoded");
      // Data to send with HTTP POST
     // String httpRequestData = "api_key=tPmAT5Ab3j7F9&sensor=BME280&value1=24.25&value2=49.54&value3=1005.14";           
      // Send HTTP POST request
      //int httpResponseCode = http.POST(httpRequestData);
      
      // If you need an HTTP request with a content type: application/json, use the following:
      //http.addHeader("Content-Type", "application/json");
      //int httpResponseCode = http.POST("{\"api_key\":\"tPmAT5Ab3j7F9\",\"sensor\":\"BME280\",\"value1\":\"24.25\",\"value2\":\"49.54\",\"value3\":\"1005.14\"}");

 //     If you need an HTTP request with a content type: text/plain
      int i = strlen(finalMessage);
      finalMessage[i] = '\0';
      char Buffer[100] = "";
      char Buffer2[10] = "";
      strcat(Buffer,Altitude);
      strcat(Buffer,";");
      strcat(Buffer,finalMessage);
      strcat(Buffer,",D|");
      strcat(Buffer,Central_TS);
      strcat(Buffer,";");
      itoa(temp,Buffer2,10);
      strcat(Buffer,Buffer2);
      
      http.addHeader("Content-Type", "text/plain");
      int httpResponseCode = http.POST(Buffer);
     
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
        
      // Free resources
      http.end();
    }
    else {
      Serial.println("WiFi Disconnected");
          Serial.print(Altitude);
    Serial.print(';');
    Serial.print(Central_TS);
    Serial.print(';');
    Serial.println(temp);
    Serial.println(finalMessage);
    Serial.println("");
    }
    strcpy(finalMessage,"");
  //  finalMessage[255] = "empty";
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
    char date[10] = "hh:mm:ss";
    if (rf95.waitAvailableTimeout(1000)) {
      char *rec = (char*)malloc(RH_RF95_MAX_MESSAGE_LEN);
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; // Creating a buffer and defining its length
      uint8_t len = sizeof(buf);
  
      if (rf95.recv(buf, &len)) {
       // digitalWrite(LED, HIGH); 
        //RH_RF95::printBuffer("Received: ", buf, len);
        //Serial.print("Got: "); Serial.println((char*)buf);
        strcpy(rec, (char*)buf);
        *msg = rec;
        getCycles();
   // Serial.print("Raw message received: "); Serial.println(rec);
        rtc.now().toString(date);
        strcpy(Central_TS,date);
       // strcpy(date,"hh:mm:ss");
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
  counterCPU = midCounter - startCounter;
}

bool isIdentified(char *msg, int len) {
  return (isAlphaNumeric(msg[0])&&(msg[1]=='|'));  
}

char getID(char *msg) {
  return msg[0];  
}
