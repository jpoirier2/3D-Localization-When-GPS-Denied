void setup() {
  // put your setup code here, to run once:
  // sets pin 13 (the LED) to output
  pinMode(13, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // digitalWrite() writes a value to the appropriate pin
  // F() is used to print typical strings
  int del;
  Serial.print(F("Now decreasing\n"));
  for(del = 1000; del >= 50; del-=50)
  {
    digitalWrite(13, HIGH);
    delay(del);
    digitalWrite(13, LOW);
    delay(del);
    Serial.println(del);
    //Serial.print('\n');
    }
  Serial.print(F("Now increasing\n"));
  for(del = 50; del <= 1000; del+=50)
  {
    digitalWrite(13, HIGH);
    delay(del);
    digitalWrite(13, LOW);
    delay(del);
    Serial.println(del);
    //Serial.print('\n');
    }
}
