
double startCounter;
double counter;
double cpu_cycles;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

  startCounter = ESP.getCycleCount();

delayMicroseconds(1);

counter = ESP.getCycleCount();

cpu_cycles = (counter - startCounter);

Serial.println(cpu_cycles);

delay(250);

}
