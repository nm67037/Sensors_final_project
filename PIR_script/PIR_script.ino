#define pirPin 3

void setup() {
  Serial.begin(115200);
  pinMode(pirPin, INPUT);
}

void loop() {
  int pirState = digitalRead(pirPin);
  Serial.println(pirState);

  if (pirState == HIGH) {
  //  Serial.println("Motion detected!");
  } else {
    //Serial.println("No motion.");
  }

  delay(100); // Check every 100 milliseconds
}
