/* * COMBINED ROBOT & STEPPER CODE
 * * Hardware:
 * 1. MD25 Motor Driver (I2C Address 0x58)
 * 2. HC-SR04 Ultrasonic Sensor (Trig: 3, Echo: 2)
 * 3. Stepper Motor (Dir: 4, Step: 5, Enable: 6)
 */

#include <Wire.h>  // Library for I2C communication with the MD25 board

// ==========================================
// 1. STEPPER MOTOR DEFINITIONS
// ==========================================
#define dirPin 4 
#define stepPin 5 
#define enablePin 6 
#define stepsPerRevolution 200 

// ==========================================
// 2. MD25 ROBOT & ULTRASONIC DEFINITIONS
// ==========================================
#define CMD (byte)0x00      // Values of 0 being sent using write have to be cast as a byte 
#define MD25ADDRESS 0x58    // Address of the MD25
#define SPEED1 (byte)0x00   // Byte to send speed to first motor
#define SPEED2 0x01         // Byte to send speed to second motor
#define ENCODERONE 0x02     // Byte to read motor encoder 1
#define ENCODERTWO 0x06     // Byte to read motor encoder 2
#define RESETENCODERS 0x10  // Byte to reset encoder values
#define MODESELECTOR 0xF    // Byte to change between control MODES

#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 3 // attach pin D3 Arduino to pin Trig of HC-SR04

// Global variables for MD25
int mode = 0;           // int mode stores the mode in which the MD25 will operate
int motor1Speed = 128;  // 0 is full reverse, 128 is stop, 255 is full forward
int motor2Speed = 128;  
int motor1Dist = 0;     // distance in encoder units motor 1 will travel 
int motor2Dist = 0;     

// Global variables for Ultrasonic
long duration; 
int distance; 

// Global variables for Calculations
float encoder_to_mm = 0.9;  
float angle_to_dist = 2.5;  

// ==========================================
// SETUP
// ==========================================
void setup() {
  // --- Init Serial & I2C ---
  Serial.begin(9600);
  Wire.begin();
  delay(100);

  // --- Init Stepper Pins ---
  pinMode(stepPin, OUTPUT); 
  pinMode(dirPin, OUTPUT); 
  pinMode(enablePin, OUTPUT); 
  digitalWrite(enablePin, LOW); // enable must be low to enable motor 

  // --- Init Ultrasonic Pins ---
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 

  // --- Init MD25 Driver ---
  Wire.beginTransmission(MD25ADDRESS); 
  Wire.write(MODESELECTOR);
  Wire.write(mode);
  Wire.endTransmission();

  encoderReset();  // Reset encoders to 0
  
  delay(1000); // Wait before starting
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  
  // 1. Read Distance Sensor
  readDistance();

  // 2. Move Robot (One leg of the square)
  // Note: This function blocks until movement is finished
  runRobotPattern();

  // 3. Move Stepper Motor
  // Note: This function blocks until spinning is finished
  runStepperPattern();
  
  // Wait a moment before repeating the whole cycle
  delay(1000);
}

// ==========================================
// CUSTOM FUNCTIONS
// ==========================================

// --- Function to read HC-SR04 ---
void readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2; 
  
  Serial.print("Current Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

// --- Function to run the Robot Movement (MD25) ---
void runRobotPattern() {
  Serial.println("--- Robot Moving ---");
  my_straight(200, 20); // go forward 200 steps at speed 20
  my_turn(90, 10);      // turn 90 degrees clockwise at speed 10 
  delay(500);
}

// --- Function to run the Stepper Sequence ---
void runStepperPattern() {
  Serial.println("--- Stepper Moving ---");
  
  // 1. Clockwise 1 rev slow
  digitalWrite(dirPin, HIGH); 
  for (int i = 0; i < stepsPerRevolution; i++) { 
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(2000); 
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(2000); 
  } 
  delay(500); 
  
  // 2. Counter-clockwise 1 rev quick
  digitalWrite(dirPin, LOW); 
  for (int i = 0; i < stepsPerRevolution; i++) { 
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(1000); 
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(1000); 
  } 
  delay(500); 

  // 3. Clockwise 5 revs fast
  digitalWrite(dirPin, HIGH); 
  for (int i = 0; i < 5 * stepsPerRevolution; i++) { 
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(500); 
  } 
  delay(500); 
  
  // 4. Counter-clockwise 5 revs fast
  digitalWrite(dirPin, LOW); 
  for (int i = 0; i < 5 * stepsPerRevolution; i++) { 
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(500); 
  } 
}

// ==========================================
// MD25 HELPER FUNCTIONS
// ==========================================

void my_turn(int angle, int speed) {
  int ang_dist = angle_to_dist * angle; 
  if (speed > 127) speed = 127; 
  if (speed < -127) speed = -127;
  if (angle < 0) speed = -speed; 

  motor1Speed = 128 + speed; 
  motor2Speed = 128 - speed; 
  motor1Dist = abs(ang_dist); 
  motor2Dist = abs(ang_dist); 
  moveMotor();
}

void my_straight(int dist, int speed) {
  dist = -dist; // motors are backwards usually
  if (speed > 127) speed = 127; 
  if (speed < -127) speed = -127;
  if (dist < 0) speed = -speed; 

  motor1Speed = 128 + speed; 
  motor2Speed = 128 + speed; 
  motor1Dist = dist; 
  motor2Dist = dist; 

  moveMotor();
}

void encoderReset() {
  delay(250); 
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(RESETENCODERS);
  Wire.write(0x20);
  Wire.endTransmission();
  delay(50);
}

long encoder1() {
  Wire.beginTransmission(MD25ADDRESS); 
  Wire.write(ENCODERONE);
  Wire.endTransmission();
  
  Wire.requestFrom(MD25ADDRESS, 4); 
  while (Wire.available() < 4); 
  long poss1 = Wire.read(); 
  poss1 <<= 8;
  poss1 += Wire.read(); 
  poss1 <<= 8;
  poss1 += Wire.read(); 
  poss1 <<= 8;
  poss1 += Wire.read(); 
  delay(5); 
  return (poss1); 
}

long encoder2() {
  Wire.beginTransmission(MD25ADDRESS); 
  Wire.write(ENCODERTWO);
  Wire.endTransmission();

  Wire.requestFrom(MD25ADDRESS, 4); 
  while (Wire.available() < 4); 
  long poss2 = Wire.read(); 
  poss2 <<= 8;
  poss2 += Wire.read(); 
  poss2 <<= 8;
  poss2 += Wire.read(); 
  poss2 <<= 8;
  poss2 += Wire.read(); 
  delay(5); 
  return (poss2); 
}

void displayEncoderDist() {
  Serial.print("Enc1: ");
  Serial.print(encoder1()); 
  Serial.print(" | Enc2: ");
  Serial.print(encoder2()); 
  Serial.println(" ");
}

void displayIntendedDist() {
  Serial.print("Target1: ");
  Serial.print(motor1Dist); 
  Serial.print(" | Target2: ");
  Serial.print(motor2Dist); 
  Serial.println(" ");
}

void set_speed1(char my_speed) {
   Wire.beginTransmission(MD25ADDRESS);
   Wire.write(SPEED1);
   Wire.write(my_speed); 
   Wire.endTransmission();
}

void set_speed2(char my_speed) {
   Wire.beginTransmission(MD25ADDRESS);
   Wire.write(SPEED2);
   Wire.write(my_speed); 
   Wire.endTransmission();
}

void moveMotor() {
  encoderReset();
  displayEncoderDist(); 
  displayIntendedDist(); 

  // Wait until targets are reached
  while ((abs(encoder1()) < abs(motor1Dist)) || (abs(encoder2()) < abs(motor2Dist))) { 
    // displayEncoderDist(); // Optional: Uncomment to debug position live
    set_speed1(motor1Speed); 
    set_speed2(motor2Speed); 
    
    // Individual stop conditions if one motor finishes early
    if (((motor1Speed != 128) && abs(encoder1()) >= abs(motor1Dist))) { 
      set_speed1(128); 
    }
    if ((motor2Speed != 128) && (abs(encoder2()) >= abs(motor2Dist))) { 
      set_speed2(128); 
    }
  }
  
  // Full stop
  set_speed1(128); 
  set_speed2(128); 
  Serial.println("Done Moving.");
  displayEncoderDist(); 
}
