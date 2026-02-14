
---

## 💻 **2. Main Code** (`src/autoguard_v1.ino`)


```cpp
/*
 * IGNIPOTENT — AutoGuard-X
 * Autonomous Thermal Response Node
 * 
 * Team: Ignipotent (Having Dominion Over Flame)
 * Hardware: ESP32 + IR + DHT22 + Ultrasonic + Servo + L298N
 * 
 * State Machine: PATROL → SCAN → TRIANGULATE → APPROACH → SUPPRESS → RETREAT
 */

#include <DHT.h>
#include <ESP32Servo.h>
#include <NewPing.h>

// ============ PIN CONFIGURATION ============
// Motors (L298N)
#define MOTOR_LEFT_EN   25
#define MOTOR_LEFT_1    26
#define MOTOR_LEFT_2    27
#define MOTOR_RIGHT_EN  14
#define MOTOR_RIGHT_1   12
#define MOTOR_RIGHT_2   13

// Sensors
#define IR_PIN          34      // Analog
#define DHT_PIN         4       // Digital
#define DHT_TYPE        DHT22
#define TRIG_PIN        5
#define ECHO_PIN        18
#define MAX_DISTANCE    200     // cm

// Actuators
#define SERVO_PIN       19
#define PUMP_PIN        21      // Via MOSFET/Relay

// ============ CONSTANTS ============
#define IR_THRESHOLD        600
#define IR_CONFIRM          400
#define TEMP_RISE_THRESHOLD 3.0     // °C per minute
#define MAX_AMBIENT_TEMP    55.0    // Safety cutoff
#define OPTIMAL_DISTANCE    30      // cm
#define TANK_CAPACITY       500     // ml (for monitoring)

// Motor Speeds (PWM 0-255)
#define SPEED_PATROL        150
#define SPEED_APPROACH      100
#define SPEED_TURN          120

// Timing
#define PUMP_PULSE_DURATION 3000    // ms
#define SCAN_DELAY          15      // ms per degree
#define SAFETY_CHECK_INTERVAL 1000  // ms

// ============ STATE ENUM ============
enum SystemState {
  PATROL,
  SCAN,
  TRIANGULATE,
  APPROACH,
  SUPPRESS,
  RETREAT,
  STANDBY
};

// ============ GLOBAL OBJECTS ============
DHT dht(DHT_PIN, DHT_TYPE);
Servo turretServo;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

SystemState currentState = PATROL;
SystemState lastState = PATROL;

// Sensor Variables
int irValue = 0;
float currentTemp = 0;
float previousTemp = 0;
unsigned long lastTempCheck = 0;
float distance = 0;

// Fire Tracking
int fireAngle = 90;           // 0-180 degrees
int maxIRReading = 0;
int scanDirection = 1;        // 1 = right, -1 = left

// Safety & Counters
bool pumpActive = false;
unsigned long pumpStartTime = 0;
int suppressionCycles = 0;
float waterLevel = 100;       // Percentage

// ============ SETUP ============
void setup() {
  Serial.begin(115200);
  Serial.println("IGNIPOTENT — AutoGuard-X Initializing...");
  
  // Pin Modes
  pinMode(MOTOR_LEFT_EN, OUTPUT);
  pinMode(MOTOR_LEFT_1, OUTPUT);
  pinMode(MOTOR_LEFT_2, OUTPUT);
  pinMode(MOTOR_RIGHT_EN, OUTPUT);
  pinMode(MOTOR_RIGHT_1, OUTPUT);
  pinMode(MOTOR_RIGHT_2, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  
  // Initialize Low
  digitalWrite(PUMP_PIN, LOW);
  stopMotors();
  
  // Initialize Sensors
  dht.begin();
  turretServo.attach(SERVO_PIN);
  turretServo.write(90);  // Center position
  
  // Initial Reading
  previousTemp = dht.readTemperature();
  delay(2000);  // DHT stabilization
  
  Serial.println("System Ready. State: PATROL");
  currentState = PATROL;
}

// ============ MAIN LOOP ============
void loop() {
  // 1. Update Sensors (Non-blocking where possible)
  updateSensors();
  
  // 2. Safety Check (Highest Priority)
  if (checkOverheating()) {
    if (currentState != RETREAT) {
      enterRetreatMode();
    }
  }
  
  // 3. State Machine Execution
  switch (currentState) {
    case PATROL:
      handlePatrol();
      break;
      
    case SCAN:
      handleScan();
      break;
      
    case TRIANGULATE:
      handleTriangulate();
      break;
      
    case APPROACH:
      handleApproach();
      break;
      
    case SUPPRESS:
      handleSuppress();
      break;
      
    case RETREAT:
      handleRetreat();
      break;
      
    case STANDBY:
      // Manual override / Debug mode
      break;
  }
  
  // 4. Status Report (Every 2 seconds)
  static unsigned long lastReport = 0;
  if (millis() - lastReport > 2000) {
    reportStatus();
    lastReport = millis();
  }
  
  delay(50);  // Main loop tick
}

// ============ STATE HANDLERS ============

void handlePatrol() {
  // Wall-following behavior
  if (distance < 15) {
    // Obstacle detected, turn right
    turnRight(90);  // 90 degree turn
    delay(500);
  } else {
    moveForward(SPEED_PATROL);
  }
  
  // Check for fire
  if (irValue > IR_CONFIRM) {
    stopMotors();
    lastState = PATROL;
    currentState = SCAN;
    Serial.println("Fire suspected. Entering SCAN mode.");
  }
}

void handleScan() {
  // Sweep turret to find fire direction
  static int angle = 0;
  static bool scanningRight = true;
  
  if (scanningRight) {
    angle += 5;
    if (angle >= 180) scanningRight = false;
  } else {
    angle -= 5;
    if (angle <= 0) {
      scanningRight = true;
      // Completed full scan without confirmation
      if (maxIRReading < IR_THRESHOLD) {
        currentState = PATROL;  // False alarm
        Serial.println("False alarm. Returning to PATROL.");
        return;
      }
    }
  }
  
  turretServo.write(angle);
  delay(SCAN_DELAY);
  
  // Read IR at this angle
  int currentIR = analogRead(IR_PIN);
  
  if (currentIR > maxIRReading) {
    maxIRReading = currentIR;
    fireAngle = angle;
  }
  
  // If strong signal confirmed
  if (maxIRReading > IR_THRESHOLD) {
    Serial.print("Fire confirmed at angle: ");
    Serial.println(fireAngle);
    currentState = TRIANGULATE;
  }
}

void handleTriangulate() {
  // Align chassis with fire angle
  // Assuming servo is mounted on chassis center
  // If fireAngle < 90, turn left; if > 90, turn right
  
  int turnDegrees = fireAngle - 90;
  
  if (abs(turnDegrees) > 10) {
    if (turnDegrees < 0) {
      turnLeft(map(abs(turnDegrees), 0, 90, 0, 1000));
    } else {
      turnRight(map(turnDegrees, 0, 90, 0, 1000));
    }
    delay(100);
    stopMotors();
  } else {
    // Aligned
    turretServo.write(90);  // Center turret relative to chassis
    Serial.println("Alignment complete. Approaching.");
    currentState = APPROACH;
  }
}

void handleApproach() {
  // Move toward fire while monitoring distance
  if (distance > OPTIMAL_DISTANCE && distance > 0) {
    moveForward(SPEED_APPROACH);
    
    // Dynamic adjustment: if IR drops, slight turn to reacquire
    if (irValue < IR_THRESHOLD * 0.8) {
      slightAdjust();
    }
  } else if (distance <= OPTIMAL_DISTANCE && distance > 0) {
    stopMotors();
    Serial.println("Optimal distance reached. Suppressing.");
    suppressionCycles = 0;
    currentState = SUPPRESS;
  } else {
    // Invalid reading, stop and retry
    stopMotors();
    delay(500);
  }
}

void handleSuppress() {
  // Check if we still see fire
  if (irValue < IR_CONFIRM && suppressionCycles > 0) {
    // Fire is out
    digitalWrite(PUMP_PIN, LOW);
    pumpActive = false;
    Serial.println("Fire extinguished. Mission complete.");
    currentState = PATROL;
    suppressionCycles = 0;
    return;
  }
  
  // Check water level (simulated or via sensor)
  if (waterLevel < 20) {
    digitalWrite(PUMP_PIN, LOW);
    pumpActive = false;
    Serial.println("WARNING: Water low! Retreat!");
    currentState = RETREAT;
    return;
  }
  
  // Pulse suppression
  if (!pumpActive) {
    digitalWrite(PUMP_PIN, HIGH);
    pumpActive = true;
    pumpStartTime = millis();
    suppressionCycles++;
    waterLevel -= 5;  // Simulate usage
    Serial.print("Suppression cycle: ");
    Serial.println(suppressionCycles);
  } else {
    if (millis() - pumpStartTime > PUMP_PULSE_DURATION) {
      digitalWrite(PUMP_PIN, LOW);
      pumpActive = false;
      delay(500);  // Wait between pulses to check if fire out
    }
  }
}

void handleRetreat() {
  // Back away from fire
  moveBackward(SPEED_PATROL);
  delay(1000);  // Back up 1 meter approx
  stopMotors();
  
  // Wait for cooldown
  Serial.println("Retreating... waiting for thermal cooldown.");
  delay(5000);
  
  // Check if safe to return
  if (currentTemp < 45.0) {
    Serial.println("Temperature normal. Resuming mission.");
    currentState = lastState;  // Return to previous task
  }
}

// ============ SENSOR FUNCTIONS ============

void updateSensors() {
  // IR Sensor (Analog)
  irValue = analogRead(IR_PIN);
  
  // Ultrasonic (NewPing library)
  distance = sonar.ping_cm();
  
  // DHT22 (Limit reading frequency)
  if (millis() - lastTempCheck > 2000) {
    float newTemp = dht.readTemperature();
    float newHum = dht.readHumidity();
    
    if (!isnan(newTemp)) {
      previousTemp = currentTemp;
      currentTemp = newTemp;
    }
    lastTempCheck = millis();
  }
}

bool checkOverheating() {
  if (currentTemp > MAX_AMBIENT_TEMP) {
    Serial.println("CRITICAL: Ambient temperature too high!");
    return true;
  }
  return false;
}

void enterRetreatMode() {
  stopMotors();
  if (pumpActive) {
    digitalWrite(PUMP_PIN, LOW);
    pumpActive = false;
  }
  lastState = currentState;
  currentState = RETREAT;
  Serial.println("SAFETY OVERRIDE: Entering RETREAT mode.");
}

// ============ MOTOR CONTROL ============

void moveForward(int speed) {
  analogWrite(MOTOR_LEFT_EN, speed);
  analogWrite(MOTOR_RIGHT_EN, speed);
  digitalWrite(MOTOR_LEFT_1, HIGH);
  digitalWrite(MOTOR_LEFT_2, LOW);
  digitalWrite(MOTOR_RIGHT_1, HIGH);
  digitalWrite(MOTOR_RIGHT_2, LOW);
}

void moveBackward(int speed) {
  analogWrite(MOTOR_LEFT_EN, speed);
  analogWrite(MOTOR_RIGHT_EN, speed);
  digitalWrite(MOTOR_LEFT_1, LOW);
  digitalWrite(MOTOR_LEFT_2, HIGH);
  digitalWrite(MOTOR_RIGHT_1, LOW);
  digitalWrite(MOTOR_RIGHT_2, HIGH);
}

void turnLeft(int duration) {
  analogWrite(MOTOR_LEFT_EN, SPEED_TURN);
  analogWrite(MOTOR_RIGHT_EN, SPEED_TURN);
  digitalWrite(MOTOR_LEFT_1, LOW);
  digitalWrite(MOTOR_LEFT_2, HIGH);
  digitalWrite(MOTOR_RIGHT_1, HIGH);
  digitalWrite(MOTOR_RIGHT_2, LOW);
  delay(duration);
  stopMotors();
}

void turnRight(int duration) {
  analogWrite(MOTOR_LEFT_EN, SPEED_TURN);
  analogWrite(MOTOR_RIGHT_EN, SPEED_TURN);
  digitalWrite(MOTOR_LEFT_1, HIGH);
  digitalWrite(MOTOR_LEFT_2, LOW);
  digitalWrite(MOTOR_RIGHT_1, LOW);
  digitalWrite(MOTOR_RIGHT_2, HIGH);
  delay(duration);
  stopMotors();
}

void stopMotors() {
  digitalWrite(MOTOR_LEFT_1, LOW);
  digitalWrite(MOTOR_LEFT_2, LOW);
  digitalWrite(MOTOR_RIGHT_1, LOW);
  digitalWrite(MOTOR_RIGHT_2, LOW);
  analogWrite(MOTOR_LEFT_EN, 0);
  analogWrite(MOTOR_RIGHT_EN, 0);
}

void slightAdjust() {
  // Micro-adjustment if fire moves slightly
  stopMotors();
  delay(100);
  // Small left turn then continue
  turnLeft(100);
}

// ============ DEBUG & REPORTING ============

void reportStatus() {
  Serial.println("=== IGNIPOTENT STATUS ===");
  Serial.print("State: ");
  switch(currentState) {
    case PATROL: Serial.println("PATROL"); break;
    case SCAN: Serial.println("SCAN"); break;
    case TRIANGULATE: Serial.println("TRIANGULATE"); break;
    case APPROACH: Serial.println("APPROACH"); break;
    case SUPPRESS: Serial.println("SUPPRESS"); break;
    case RETREAT: Serial.println("RETREAT"); break;
    default: Serial.println("UNKNOWN"); break;
  }
  Serial.print("IR: "); Serial.print(irValue);
  Serial.print(" | Temp: "); Serial.print(currentTemp);
  Serial.print("°C | Dist: "); Serial.print(distance);
  Serial.print("cm | Water: "); Serial.print(waterLevel);
  Serial.println("%");
  Serial.println("=========================");
}
