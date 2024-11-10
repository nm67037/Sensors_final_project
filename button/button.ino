//this code was taken from the Adafruit website

int fsrAnalogPin = 5; 
int fsrReading; 
int threshold = 600;
bool state = false;
bool running = false;

void setup(void) {
  Serial.begin(115000);   
}
 
void loop(void) {
  while (!running) {
  fsrReading = analogRead(fsrAnalogPin);
  Serial.print("Analog reading = ");
  Serial.println(fsrReading);

   // Check if FSR is pressed
  if (fsrReading >= threshold) {
    state = true; // FSR is pressed
    Serial.println("State: ON");
  } else {
    state = false; // FSR is not pressed
    Serial.println("State: OFF");
  }  
  delay(500);
  }
  delay(1000);
}