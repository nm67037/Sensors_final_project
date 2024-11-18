#define pirPin 3

//ultrasonic parameters:
const int trig_pin = 4;
const int echo_pin = 6;
float timing = 0.0;
float distance = 0.0;

void setup() {
  Serial.begin(115200);
  pinMode(pirPin, INPUT);
  pinMode(echo_pin, INPUT);
  pinMode(trig_pin, OUTPUT);
  digitalWrite(trig_pin, LOW);
}

void loop() {
  int pirState = digitalRead(pirPin);
  

  digitalWrite(trig_pin, LOW);
  delay(2);
  
  digitalWrite(trig_pin, HIGH);
  delay(10);
  digitalWrite(trig_pin, LOW);
  
  timing = pulseIn(echo_pin, HIGH);
  distance = (timing * 0.034) / 2;
  distance = constrain(distance, 0, 20);
  
  // Serial.print("Distance: ");
  Serial.print(distance);Serial.print(",");
  Serial.println(pirState);
  //Serial.println("cm | ");


  delay(100); // Check every 100 milliseconds
}
