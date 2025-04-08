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
const int HIGHPin = 42;

void setup() {
  // Start serial communication with the computer
  Serial.begin(512000);  // Set baud rate to 115200 (same as in PC-side code)
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);

  pinMode(HIGHPin, OUTPUT);
  digitalWrite(HIGHPin, HIGH);

  // Dist sensor
  Wire.begin(8,9);
  Wire.setClock(400000); // use 400 kHz I2C

  sensor.setTimeout(100);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
  }
//
//  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
//  // You can change these settings to adjust the performance of the sensor, but
//  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
//  // medium and long distance modes. See the VL53L1X datasheet for more
//  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(50000);
  sensor.setROISize(4, 4);
//
//  // Start continuous readings at a rate of one measurement every 50 ms (the
//  // inter-measurement period). This period should be at least as long as the
//  // timing budget.
  sensor.startContinuous(50);
}

float get_dist(){
  float avg_dist;
  avg_dist = sensor.read();
  for (int x = 0; x < 4; x++){
    avg_dist = (avg_dist + sensor.read()) * 0.5;
    
  }
  return avg_dist;  
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
void executeCommand(String message) {
  String all_command[100];
  int i = 0;
  
  while (not(message.startsWith("DONE"))){
    if (message.startsWith("CMD")){
      all_command[i] = message;
      i ++;
    }
    message = Serial.readStringUntil('\n');
  }
  
  for (int x = 0; x < sizeof(all_command); x++) {
    if (all_command[x].startsWith("CMD")) {
      // Serial.println(all_command[x]);
      motionCommand(all_command[x]);
    } else {
      return;
    }
    
  }
  
  
}
void motionCommand(String message) {
  // Basic validation
  if (!message.startsWith("CMD ")) return;

  // Extract directions (CMD 0 1 ...)
  int firstSpace = message.indexOf(' ');
  int secondSpace = message.indexOf(' ', firstSpace + 1);
  int thirdSpace = message.indexOf(' ', secondSpace + 1);
  int fourthSpace = message.indexOf(' ', thirdSpace + 1);

  // Validate we have at least directions
  if (secondSpace == -1 || thirdSpace == -1) {
    Serial.println("##ERROR: Missing directions");
    return;
  }

  // Get direction values
  int dirA = message.substring(firstSpace + 1, secondSpace).toInt();
  int dirB = message.substring(secondSpace + 1, thirdSpace).toInt();
  int dirZ = message.substring(thirdSpace + 1, fourthSpace).toInt();

  // Set motor directions
  digitalWrite(dirPin, dirA);
  digitalWrite(dirPin2, dirB);
  digitalWrite(dirPin3, dirZ);

  // Process step commands
  int currentPos = fourthSpace;
  const int pulseWidth = 2500; // microseconds

  while (currentPos != -1 && currentPos < message.length()) {
    int nextSpace = message.indexOf(' ', currentPos + 1);
    String command = message.substring(currentPos + 1, nextSpace);
    
    int colonIndex = command.indexOf(':');
    if (colonIndex == -1) {
      currentPos = nextSpace;
      continue;
    }

    String motors = command.substring(0, colonIndex);
    int delayTime = command.substring(colonIndex + 1).toInt();

    // Determine which motors to step
    bool stepA = motors.indexOf('A') != -1;
    bool stepB = motors.indexOf('B') != -1;
    bool stepZ = motors.indexOf('Z') != -1;

    // Pulse motors
    if (stepA) digitalWrite(stepPin, HIGH);
    if (stepB) digitalWrite(stepPin2, HIGH);
    if (stepZ) digitalWrite(stepPin3, HIGH);
    
    if (stepZ) {
      delayMicroseconds(500);
      } else {
        delayMicroseconds(delayTime);
    }
    
    if (stepA) digitalWrite(stepPin, LOW);
    if (stepB) digitalWrite(stepPin2, LOW);
    if (stepZ) digitalWrite(stepPin3, LOW);

    // Handle remaining delay

    if (stepZ) {
      delayMicroseconds(500);
    } else {
      delayMicroseconds(delayTime);
    }


    currentPos = nextSpace;
  }
  
  Serial.println("##ACTION COMPLETED");
}

// Example command format:
// "CMD 1 0 A:12000 AB:10000 B:15000"
// Breakdown:
// - Directions: A=HIGH (1), B=LOW (0)
// - First step: Motor A with 12ms delay
// - Second step: Both motors with 10ms delay
// - Third step: Motor B with 15ms delay

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
      int clockwise;
      int steps, wait;
      int numArgs = sscanf(message.c_str(), "rot_z %d %d %d", &clockwise, &steps, &wait);

      if (numArgs == 3) {
        // Call the rot_z function with the extracted values
        rot_z(static_cast<bool>(clockwise), steps, wait);
        
        // Send a confirmation message back to the PC
        Serial.println("Motor control executed.");
      } else {
        Serial.println("Error: Invalid message format.");
      }
    } else if (message.startsWith("rot_a")) {
      int clockwise;
      int steps, wait;
      int numArgs = sscanf(message.c_str(), "rot_a %d %d %d", &clockwise, &steps, &wait);

      if (numArgs == 3) {
        // Call the rot_z function with the extracted values
        rot_a(static_cast<bool>(clockwise), steps, wait);
        
        // Send a confirmation message back to the PC
        Serial.println("Motor control executed.");
      } else {
        Serial.println("Error: Invalid message format.");
      }
    } else if (message.startsWith("rot_b")) {
      int clockwise;
      int steps, wait;
      int numArgs = sscanf(message.c_str(), "rot_b %d %d %d", &clockwise, &steps, &wait);

      if (numArgs == 3) {
        // Call the rot_z function with the extracted values
        rot_b(static_cast<bool>(clockwise), steps, wait);
        
        // Send a confirmation message back to the PC
        Serial.println("Motor control executed.");
      } else {
        Serial.println("Error: Invalid message format.");
      }
    } else if (message.startsWith("get_dist")) {
      char verify_string[50]; // Adjust size as needed
      int numArgs = sscanf(message.c_str(), "get_dist %49s", verify_string);
  
      if (numArgs == 1) {
          float dist_float = get_dist();
          String output = "DIST ";
          output.concat(verify_string);
          output.concat(" ");
          output.concat(dist_float);
          Serial.println(output);
      } else {
          Serial.println("ERROR: Invalid command format");
      }

      
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
    } else if (message.startsWith("CMD")) {
      executeCommand(message);
    }
  }
  delay(10);  // Small delay to avoid hogging the CPU
}
