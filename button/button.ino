//the code for the FSR was sourced from the Adafruit website
//the ultrasonic code was sourced from arduino's website

//ultrasonic parameters:
const int trig_pin = 4;
const int echo_pin = 6;
float timing = 0.0;
float distance = 0.0;

//Hall:
#define Hall_Sensor_Pin 10

int fsrAnalogPin = 5; 
//int Hall_Sensor_Pin = 10;
int fsrReading;       
int threshold = 600;
 
bool state = false;   
bool running = false;


void setup() {
  pinMode(Hall_Sensor_Pin, INPUT);
  pinMode(echo_pin, INPUT);
  pinMode(trig_pin, OUTPUT);
  pinMode(Hall_Sensor_Pin,INPUT);
  digitalWrite(trig_pin, LOW);
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
    //Serial.println("Running part of the code...");
    
    float voltage;
    voltage = analogRead(Hall_Sensor_Pin);
    voltage = analogRead(Hall_Sensor_Pin);
    Serial.println(voltage);
    if (voltage<=3500)
    {
      Serial.println("HIGH");
    }
    else if (voltage>4500 && voltage<5000)
    {
      Serial.println("None");
    }
    else 
    {
      Serial.println("LOW");
    }
    delay(1000);
   // Serial.println(hall_voltage);

    digitalWrite(trig_pin, LOW);
    delay(2);
    
    digitalWrite(trig_pin, HIGH);
    delay(10);
    digitalWrite(trig_pin, LOW);
    
    timing = pulseIn(echo_pin, HIGH);
    distance = (timing * 0.034) / 2;
    
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println("cm | ");
    
    delay(750); // Simulate processing time
  }
}

