#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

const int dirPin = 7;
const int stepPin = 6;
const int dirPin2 = 16;
const int stepPin2 = 15;
const int dirPin3 = 5;
const int stepPin3 = 4;
const int stepsPerRevolution = 200;
const int slow_wait = 7000;

void setup() {
  // Start serial communication with the computer
  Serial.begin(115200);  // Set baud rate to 115200 (same as in PC-side code)
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);

  // Dist sensor
//  Wire.begin();
//  Wire.setClock(400000); // use 400 kHz I2C
//
//  sensor.setTimeout(500);
//  if (!sensor.init())
//  {
//    Serial.println("Failed to detect and initialize sensor!");
//  }
//
//  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
//  // You can change these settings to adjust the performance of the sensor, but
//  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
//  // medium and long distance modes. See the VL53L1X datasheet for more
//  // information on range and timing limits.
//  sensor.setDistanceMode(VL53L1X::short);
//  sensor.setMeasurementTimingBudget(50000);
//
//  // Start continuous readings at a rate of one measurement every 50 ms (the
//  // inter-measurement period). This period should be at least as long as the
//  // timing budget.
//  sensor.startContinuous(50);
}

void get_dist(){
  sensor.read();

  Serial.println("range: " + String(sensor.ranging_data.range_mm));  
}


void rot_z(bool clockwise, int steps, int wait){
  if (clockwise){
    digitalWrite(dirPin3, HIGH);
  } else {
    digitalWrite(dirPin3, LOW);
  }
  for (int x = 0; x < steps; x++){
    digitalWrite(stepPin3, HIGH);
    delayMicroseconds(wait);
    digitalWrite(stepPin3, LOW);
    delayMicroseconds(wait);
  }
  
}

void rot_a(bool clockwise, int steps, int wait){
  if (clockwise){
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }
  for (int x = 0; x < steps; x++){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(wait);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(wait);
  }
  
}

void rot_b(bool clockwise, int steps, int wait){
  if (clockwise){
    digitalWrite(dirPin2, HIGH);
  } else {
    digitalWrite(dirPin2, LOW);
  }
  for (int x = 0; x < steps; x++){
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(wait);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(wait);
  }
  
}

void comb_rot(bool clockwise_a, int steps_a, bool clockwise_b, int steps_b, int wait){
  if (clockwise_a){
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  if (clockwise_b){
    digitalWrite(dirPin2, HIGH);
  } else {
    digitalWrite(dirPin2, LOW);
  }

  int comb_step = min(steps_a, steps_b);
  
  Serial.println(String(comb_step) + String(steps_a) + String(steps_b));
//  for (int x = 0; x < comb_step; x++){
//    digitalWrite(stepPin, HIGH);
//    digitalWrite(stepPin2, HIGH);
//    delayMicroseconds(wait);
//    digitalWrite(stepPin, LOW);
//    digitalWrite(stepPin2, LOW);
//    delayMicroseconds(wait);
//  }
//  for (int x = 0; x < steps_a - comb_step; x++){
//    digitalWrite(stepPin, HIGH);
//    delayMicroseconds(wait);
//    digitalWrite(stepPin, LOW);
//    delayMicroseconds(wait);
//  }
//  
//  for (int x = 0; x < steps_b - comb_step; x++){
//    digitalWrite(stepPin2, HIGH);
//    delayMicroseconds(wait);
//    digitalWrite(stepPin2, LOW);
//    delayMicroseconds(wait);
//  }
  int max_steps = max(steps_a, steps_b);
  if (steps_a < max_steps) {
    int steps_done = 0;
    for (int x = 0; x < max_steps; x++){
      digitalWrite(stepPin2, HIGH);
      if ((steps_done * (float(steps_b)/float(steps_a))) < x) {
        digitalWrite(stepPin, HIGH);
      }
      delayMicroseconds(wait);
      digitalWrite(stepPin2, LOW);
      if ((steps_done * (float(steps_b)/float(steps_a))) < x) {
        digitalWrite(stepPin, LOW);
        steps_done ++;
      }
      delayMicroseconds(wait);
      Serial.println(steps_done);
      Serial.println(x);
    }
  } else if (steps_a <= max_steps) {
    int steps_done = 0;
    for (int x = 0; x < max_steps; x++){
      digitalWrite(stepPin, HIGH);
      if ((steps_done * (float(steps_a)/float(steps_b))) < x) {
        digitalWrite(stepPin2, HIGH);
      }
      delayMicroseconds(wait);
      digitalWrite(stepPin, LOW);
      if ((steps_done * (float(steps_a)/float(steps_b))) < x) {
        digitalWrite(stepPin2, LOW);
        steps_done ++;
      }
      delayMicroseconds(wait);
      Serial.println(steps_done);
      Serial.println(x);
    }
  }
}
void comb_rot2(bool clockwise_a, int steps_a, bool clockwise_b, int steps_b, int wait){
  if (clockwise_a){
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  if (clockwise_b){
    digitalWrite(dirPin2, HIGH);
  } else {
    digitalWrite(dirPin2, LOW);
  }

  int comb_step = min(steps_a, steps_b);
  
  Serial.println(String(comb_step) + String(steps_a) + String(steps_b));
  for (int x = 0; x < comb_step; x++){
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(wait);
    digitalWrite(stepPin, LOW);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(wait);
  }
  for (int x = 0; x < steps_a - comb_step; x++){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(wait);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(wait);
  }
  
  for (int x = 0; x < steps_b - comb_step; x++){
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(wait);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(wait);
  }

}


void loop() {
  if (Serial.available() > 0) {
    // Read the incoming message from the PC
    String message = Serial.readStringUntil('\n');
    
    // Create a response message
    String response = message + " received";
    
    // Send the response back to the PC
    Serial.println(response);

    if (message.startsWith("rot_z")) {
      // Example message format: rot_z 1 100 5000 (clockwise, steps, wait time)
      bool clockwise;
      int steps, wait;
      int numArgs = sscanf(message.c_str(), "rot_z %d %d %d", &clockwise, &steps, &wait);

      if (numArgs == 3) {
        // Call the rot_z function with the extracted values
        rot_z(clockwise, steps, wait);
        
        // Send a confirmation message back to the PC
        Serial.println("Motor control executed.");
      } else {
        Serial.println("Error: Invalid message format.");
      }
    } else if (message.startsWith("rot_a")) {
      bool clockwise;
      int steps, wait;
      int numArgs = sscanf(message.c_str(), "rot_a %d %d %d", &clockwise, &steps, &wait);

      if (numArgs == 3) {
        // Call the rot_z function with the extracted values
        rot_a(clockwise, steps, wait);
        
        // Send a confirmation message back to the PC
        Serial.println("Motor control executed.");
      } else {
        Serial.println("Error: Invalid message format.");
      }
    } else if (message.startsWith("rot_b")) {
      bool clockwise;
      int steps, wait;
      int numArgs = sscanf(message.c_str(), "rot_b %d %d %d", &clockwise, &steps, &wait);

      if (numArgs == 3) {
        // Call the rot_z function with the extracted values
        rot_b(clockwise, steps, wait);
        
        // Send a confirmation message back to the PC
        Serial.println("Motor control executed.");
      } else {
        Serial.println("Error: Invalid message format.");
      }
    } else if (message.startsWith("get_dist")) {
      get_dist();
    } else if (message.startsWith("comb_rot")) {
      int clockwise_a, clockwise_b;
      int steps_a, steps_b, wait;
      int numArgs = sscanf(message.c_str(), "comb_rot %d %d %d %d %d", &clockwise_a, &steps_a, &clockwise_b, &steps_b, &wait);

      if (numArgs == 5) {
        // Call the comb_rot function with the extracted values
        comb_rot(static_cast<bool>(clockwise_a), steps_a, static_cast<bool>(clockwise_b), steps_b, wait);
        
        // Send a confirmation message back to the PC
        Serial.println("Motor control executed.");
      } else {
        Serial.println("Error: Invalid message format.");
      }
    }
  }
  delay(100);  // Small delay to avoid hogging the CPU
}
