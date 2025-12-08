/* * INTERPLANETARY ROBOT - FULL INTEGRATED CODE v1.0
 * * Hardware Map:
 * 1. MD25 Motor Driver (I2C Address 0x58) -> SDA/SCL pins
 * 2. HC-SR04 Ultrasonic -> Trig: Pin 3, Echo: Pin 2
 * 3. Stepper Motor Driver -> Dir: 4, Step: 5, Enable: 6
 * * Behavior:
 * - Scans area (spins in increments).
 * - If Object < 30cm: Stops and runs Stepper Sequence (Pickup).
 * - If No Object: Drives forward (Patrols) while tracking X/Y coordinates.
 * - If Boundary reached: Turns around to stay in arena.
 */

#include <Wire.h> 

// ==========================================
// 1. PIN DEFINITIONS
// ==========================================
// Stepper motor, directions, movement, and on/off
#define dirPin 4 
#define stepPin 5 
#define enablePin 6 
#define stepsPerRevolution 200 

// Ultrasonic echoPin: send sound, trigPin: listen for echo
#define echoPin 2 
#define trigPin 3 

// MD25 Driver For movement of wheels
#define MD25ADDRESS 0x58    
#define SPEED1 (byte)0x00   
#define SPEED2 0x01         
#define ENCODERONE 0x02     
#define ENCODERTWO 0x06     
#define RESETENCODERS 0x10  
#define MODESELECTOR 0xF    

// ==========================================
// 2. GLOBAL VARIABLES & SETTINGS
// ==========================================

// --- CALIBRATION (YOU MUST ADJUST THESE!) ---
// Measure how many mm the robot moves for 1 encoder click.
// Test: Drive 1000 clicks, measure distance. Result = Distance / 1000. ******will have to change for calibration******
float mm_per_click = 0.9;      

// Measure how many clicks needed to turn 1 degree.
// Test: Tell it to turn 360 degrees. If it only turns 300, increase this number.  ********will have to change for calibration *******
float clicks_per_degree = 5.5; 

// --- ODOMETRY (Position Tracking) ---
float posX = 200.0;     // Start X (mm) - assumes starting near a corner
float posY = 200.0;     // Start Y (mm)
float heading = 0.0;    // Heading in degrees (0 = East/Forward, 90 = North) *direction it is facing*

// --- ARENA BOUNDARIES (mm) ---
// Arena is 3000 x 1500. We set limits slightly smaller to be safe.
const float MAX_X = 2500.0; 
const float MAX_Y = 1200.0;
const float MIN_X = 100.0;
const float MIN_Y = 100.0;

// --- SENSORS ---
long duration; 
int distance; 

// ==========================================
// 3. SETUP
// ==========================================
void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(100);

  // Init Pins
  pinMode(stepPin, OUTPUT); 
  pinMode(dirPin, OUTPUT); 
  pinMode(enablePin, OUTPUT); 
  digitalWrite(enablePin, LOW); // Enable Stepper
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 

  // Init MD25         ****mode0: tank steering, stop(wheels spin in opposite directions),    mode1: turn and steering at the same time********
  Wire.beginTransmission(MD25ADDRESS); 
  Wire.write(MODESELECTOR);
  Wire.write(1); // Mode 1: Speed and Turn control (Easier for steering)
  Wire.endTransmission();

  encoderReset();  
  
  Serial.println("--- WAITING 2 SECONDS (RULE 3.1) ---");   //****2 second delay before starting mvoement****
  delay(2000); // Required by Brief
  Serial.println("--- GO ---");
}

// ==========================================
// 4. MAIN LOOP
// ==========================================
void loop() {
  
  // STEP 1: SCAN SURROUNDINGS
  // The robot looks around to find a sample
  bool objectFound = scanAndLook();

  if (objectFound) {
    // --- STATE: OBJECT DETECTED --- ****pick up object mode
    Serial.println("TARGET ACQUIRED. INITIATING PICKUP.");
    
    // 1. Move slightly forward to grab range (Optional refinement)
    // drive_straight(50); 
    
    // 2. Run your specific Stepper/Arm sequence
    runStepperPattern();
    
    Serial.println("PICKUP COMPLETE. RESUMING PATROL.");
  } 
  else {
    // --- STATE: PATROL / SEARCH --- ****patrol/ search mode
    Serial.println("AREA CLEAR. MOVING TO NEXT SECTOR.");
    
    // Check if the next move puts us outside the arena
    // We look 500mm ahead
    float nextX = posX + (500 * cos(radians(heading)));
    float nextY = posY + (500 * sin(radians(heading)));
    
    if (nextX > MAX_X || nextX < MIN_X || nextY > MAX_Y || nextY < MIN_Y) {
       Serial.println("BOUNDARY WARNING! TURNING AROUND.");
       drive_turn(135); // Turn away from wall
    } else {
       // Safe to drive forward
       drive_straight(500); // Drive 50cm
    }
  }
  
  delay(500); // Brief pause between decisions
}

// ==========================================
// 5. INTELLIGENT BEHAVIORS
// ==========================================

// Function: Spins in increments and checks sensor
// Returns: TRUE if object is close (< 30cm), FALSE if clear
bool scanAndLook() {
  Serial.println("Scanning...");
  int checks = 8; // How many times to stop and check in a circle
  
  for(int i=0; i<checks; i++) {
    readDistance();
    
    // If we see something within 30cm (and not error 0)
    if(distance < 30 && distance > 0) {
      Serial.print("Object at "); Serial.print(distance); Serial.println("cm!");
      return true; // Stop scanning, we found it
    }
    
    // If nothing, turn 45 degrees and try again
    drive_turn(45);
    delay(100); // Stabilize before reading
  }
  return false; // Scanned whole circle, found nothing
}

// ==========================================
// 6. MOTOR MOVEMENTS (WITH ODOMETRY)
// ==========================================

void drive_straight(float dist_mm) {
  encoderReset();
  long targetClicks = dist_mm / mm_per_click;
  
  Serial.print("Driving: "); Serial.print(dist_mm); Serial.println("mm");

  // Send Speed Command
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(SPEED1);
  Wire.write(150); // Speed > 128 is Forward
  Wire.write(150); 
  Wire.endTransmission();
  
  // Wait loop
  while(abs(encoder1()) < targetClicks) {
    // Safety Brake: If object appears suddenly within 10cm, STOP
    readDistance();
    if(distance < 10 && distance > 0) {
      Serial.println("EMERGENCY STOP!");
      break; 
    }
  }
  
  stopMotors();
  
  // UPDATE COORDINATES
  // Trigonometry: X moves by Cos(angle), Y moves by Sin(angle)
  float rads = radians(heading);
  float movedDist = abs(encoder1()) * mm_per_click; // Actual distance moved
  
  posX += movedDist * cos(rads);
  posY += movedDist * sin(rads);
  
  printStats();
}

void drive_turn(float angle_deg) {
  encoderReset();
  long targetClicks = angle_deg * clicks_per_degree;
  
  Serial.print("Turning: "); Serial.print(angle_deg); Serial.println(" deg");
  
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(SPEED1);
  
  // For MD25 Mode 1:
  // Register Speed1 is Speed (128 = Stop)      ****128 = stop, <128 = reverse. >128 = forward)
  // Register Speed2 is Turn (128 = Straight)
  Wire.write(128); // Speed 0
  Wire.write(160); // Turn Right (>128)
  Wire.endTransmission();
  
  // Note: We only check one encoder for turning duration
  while(abs(encoder1()) < targetClicks);
  
  // Reset turn register to straight
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(SPEED2);
  Wire.write(128);
  Wire.endTransmission();
  
  stopMotors();
  
  // UPDATE HEADING
  heading += angle_deg;
  if(heading >= 360) heading -= 360;
  printStats();
}

void stopMotors() {
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(SPEED1);
  Wire.write(128); // Full Stop
  Wire.write(128); // No Turn
  Wire.endTransmission();
  delay(100);
}

// Your original Stepper Code (Integrated)
void runStepperPattern() {
  Serial.println("--- ACTIVATING ARM ---");
  
  // Enable Motor
  digitalWrite(enablePin, LOW); 
  
  // 1. Grab (Example: Clockwise)
  digitalWrite(dirPin, HIGH);    // *high = clockwise, low = counter clockwise***//
  for (int i = 0; i < stepsPerRevolution * 2; i++) { 
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(2000); 
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(2000); 
  } 
  
  delay(500); 

  // 2. Lift/Retract (Example: Counter-Clockwise)
  digitalWrite(dirPin, LOW); 
  for (int i = 0; i < stepsPerRevolution * 2; i++) { 
    digitalWrite(stepPin, HIGH); 
    delayMicroseconds(1000); 
    digitalWrite(stepPin, LOW); 
    delayMicroseconds(1000); 
  } 
  
  // Disable to save power (Optional, depends on if arm needs holding torque)
  // digitalWrite(enablePin, HIGH); 
}

// ==========================================
// 7. UTILITIES
// ==========================================

void readDistance() {    //**ultrasonic sensor**
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2; 
}

void printStats() {
  Serial.print("POS X:"); Serial.print(posX);
  Serial.print(" Y:"); Serial.print(posY);
  Serial.print(" HEAD:"); Serial.println(heading);
}

void encoderReset() {
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(RESETENCODERS);
  Wire.endTransmission();
  delay(20);
}

long encoder1() {
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(ENCODERONE);
  Wire.endTransmission();
  Wire.requestFrom(MD25ADDRESS, 4);
  while(Wire.available() < 4);
  long poss1 = Wire.read();
  poss1 <<= 8; poss1 += Wire.read();
  poss1 <<= 8; poss1 += Wire.read();
  poss1 <<= 8; poss1 += Wire.read();
  return(poss1);
}
