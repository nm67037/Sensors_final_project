//this code was sourced from the Adafruit website

int fsrAnalogPin = 5; 
int Hall_Sensor_Pin = 10;
int fsrReading;       
int threshold = 600;  
bool state = false;   
bool running = false; 

void setup() {
  pinMode(Hall_Sensor_Pin, INPUT);
  Serial.begin(115200);
}

void loop() {
  // Polling algorithm to check FSR state
  while (!running) { // Keep polling until "running" is true
    fsrReading = analogRead(fsrAnalogPin);

    if (fsrReading >= threshold) {
      state = true;
      running = true; 
      Serial.println("FSR pressed. Running = true");
    } else {
      state = false;
      Serial.println("FSR not pressed. Polling...");
    }

    delay(500); // Short delay for stability
  }

  // Code that runs once "running" is true
  if (running) {
    Serial.println("Running part of the code...");
    
    float hall_voltage;
    hall_voltage = analogRead(Hall_Sensor_Pin);
    Serial.println(hall_voltage);


    delay(1000); // Simulate processing time
  }
}
