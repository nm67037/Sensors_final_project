//the code for the FSR was sourced from the Adafruit website
//the ultrasonic code was sourced from arduino's website

#include <ESP32Servo.h>

//potentiometer parameters:
#define potPin 12
#define motor1Pin 1
Servo servo1;
//#define motor2Pin 10


//ultrasonic parameters:
const int trig_pin = 4;
const int echo_pin = 6;
float timing = 0.0;
float distance = 0.0;

//Hall parameters:
#define Hall_Sensor_Pin 10

int fsrAnalogPin = 5; 
//int Hall_Sensor_Pin = 10;
int fsrReading;       
int threshold = 600;
 
bool state = false;   
bool running = false;
bool motor_condition = false;


void setup() {
  Serial.begin(115200);
  pinMode(Hall_Sensor_Pin, INPUT);
  pinMode(echo_pin, INPUT);
  pinMode(trig_pin, OUTPUT);
  pinMode(Hall_Sensor_Pin,INPUT);
  digitalWrite(trig_pin, LOW);
  servo1.attach(motor1Pin);
}

void loop() {
  // Polling algorithm to check FSR state
  while (!running) { // Keep polling until "running" is true
    fsrReading = analogRead(fsrAnalogPin);
    Serial.println(fsrReading);
    if (fsrReading >= threshold) {
      state = true;
      running = true;
      motor_condition = true; 
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
    //voltage = analogRead(Hall_Sensor_Pin);
    Serial.println(voltage);
    if (voltage>=3700)
    {
      Serial.println("Bait is gone!");
    }
    delay(100);
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

    if (motor_condition) { 
      int motor1 = (analogRead(potPin)); //adjust the 2 depending out how pot control looks
      Serial.print("ADC Motor: ");
      Serial.println(motor1);
      int motor1Val= map(motor1, 0, 4095, 0, 180);
      servo1.write(motor1Val);
      delay(1);
    }
    delay(500); // Simulate processing time
  }
}

