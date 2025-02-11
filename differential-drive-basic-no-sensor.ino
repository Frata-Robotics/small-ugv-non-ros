/*
  Author:     Harminder Singh Nijjar
  Date:       2025-02-10
  File:       differential-drive-basic-no-sensor.ino

  Description:
    This sketch provides basic differential-drive control using RC input
    signals (throttle and steering) and the RoboClaw motor controller. It
    also includes a safety “arming” feature so that the motors remain at
    neutral until a valid RC input is received after power‑up.

    All code related to the HC-SR04 ultrasonic sensor has been removed for now.
    The buzzer pin remains defined, but is not used in this version.
    
    Connections (adjust as necessary):
      - RC throttle signal: Pin 2
      - RC steering signal: Pin 3
      - Buzzer: Pin 7
      - RoboClaw connected on Serial1 at 38400 baud.
*/

// ---------------------- Library Includes ----------------------
#include <RoboClaw.h>

// ---------------------- RC Input Definitions ----------------------
#define THROTTLE_PIN 2        // RC throttle signal (input)
#define STEERING_PIN 3        // RC steering signal (input)

// ---------------------- Buzzer Pin (currently unused) ----------------------
#define BUZZER_PIN 7          // Buzzer for audible alert (not used in this version)

// ---------------------- RoboClaw & Motor Parameters ----------------------
#define ROBOCLAW_ADDRESS 0x80 // Address for the RoboClaw motor controller
#define NEUTRAL 64            // Neutral value (motors stopped)
#define MAX_SPEED 20          // Maximum speed value for RC mapping (in our unit scale)
#define PPM_MIN 1000          // Minimum pulse width from RC (in microseconds)
#define PPM_MAX 2000          // Maximum pulse width from RC (in microseconds)
#define PPM_CENTER 1500       // Expected neutral value from RC (in microseconds)
#define DEADZONE 5            // Deadzone for small RC signal variations

// ---------------------- Global Objects & Variables ----------------------
/*
   The RoboClaw object allows us to communicate with the RoboClaw motor controller
   via the specified serial interface (Serial1 in this case).
   The timeout parameter (10000) is typically how long to wait for serial data.
*/
RoboClaw roboclaw(&Serial1, 10000);

// Safety flag: the motors remain "disarmed" until valid RC input is detected.
bool motorsArmed = false;

void setup() {
  // Initialize the primary serial port for debugging messages.
  Serial.begin(115200);

  // Initialize Serial1 for RoboClaw communication.
  Serial1.begin(38400);
  roboclaw.begin(38400);

  // Configure RC input pins.
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(STEERING_PIN, INPUT);
  
  // Configure buzzer pin (not used in this code, but left as a placeholder).
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  // Initialize motors to neutral (stopped).
  roboclaw.ForwardBackwardM1(ROBOCLAW_ADDRESS, NEUTRAL);
  roboclaw.ForwardBackwardM2(ROBOCLAW_ADDRESS, NEUTRAL);

  Serial.println("Rover ready. Motors are DISARMED until valid RC input is received.");
}

void loop() {
  // ---- Read RC Inputs ----
  // pulseIn() measures the width of the incoming pulse on the pin.
  // A timeout of 25ms (25000 microseconds) is used to avoid blocking forever.
  // If pulseIn returns 0 (i.e. no signal), substitute the neutral value.
  int throttlePulse = pulseIn(THROTTLE_PIN, HIGH, 25000);
  if (throttlePulse == 0) throttlePulse = PPM_CENTER;

  int steeringPulse = pulseIn(STEERING_PIN, HIGH, 25000);
  if (steeringPulse == 0) steeringPulse = PPM_CENTER;

  // Reverse-mapping the throttle if needed: 
  // Here we map the incoming pulse (1000-2000) to (2000-1000), effectively flipping the direction.
  throttlePulse = map(throttlePulse, PPM_MIN, PPM_MAX, PPM_MAX, PPM_MIN);
  throttlePulse = constrain(throttlePulse, PPM_MIN, PPM_MAX);
  steeringPulse = constrain(steeringPulse, PPM_MIN, PPM_MAX);

  // Convert RC pulse widths to a speed range (-MAX_SPEED to +MAX_SPEED).
  int rcFwd  = map(throttlePulse, PPM_MIN, PPM_MAX, -MAX_SPEED, MAX_SPEED);
  int rcTurn = map(steeringPulse, PPM_MIN, PPM_MAX, -MAX_SPEED, MAX_SPEED);

  // Apply deadzone filtering for small input fluctuations.
  if (abs(rcFwd) < DEADZONE)  rcFwd = 0;
  if (abs(rcTurn) < DEADZONE) rcTurn = 0;

  // ---- Motor Arming Logic ----
  // Motors will be disarmed (neutral) until a valid (non-neutral) input is detected.
  if (!motorsArmed && (abs(rcFwd) != 0 || abs(rcTurn) != 0)) {
    motorsArmed = true;
    Serial.println("Motors ARMED by RC input.");
  }

  // ---- Final Target Values for the Motors ----
  // If not armed, we force the rover to remain stopped.
  int targetFwd  = motorsArmed ? rcFwd  : 0;
  int targetTurn = motorsArmed ? rcTurn : 0;

  // ---- Compute Motor Commands (Differential Drive) ----
  /*
     For a differential-drive rover, one common approach to mixing the
     forward and turn signals is:
       leftMotor  = NEUTRAL + forward + turn
       rightMotor = NEUTRAL + forward - turn
     This gives intuitive “tank style” control based on the single throttle
     and single steering input from the RC transmitter.
  */
  int leftMotor  = NEUTRAL + targetFwd + targetTurn;
  int rightMotor = NEUTRAL + targetFwd - targetTurn;

  // Constrain motor commands to the valid limits for RoboClaw (0 to 127).
  leftMotor  = constrain(leftMotor, 0, 127);
  rightMotor = constrain(rightMotor, 0, 127);

  // ---- Send Commands to Motors ----
  roboclaw.ForwardBackwardM1(ROBOCLAW_ADDRESS, leftMotor);
  roboclaw.ForwardBackwardM2(ROBOCLAW_ADDRESS, rightMotor);

  // ---- Debug Output ----
  // Print out RC inputs and motor outputs for troubleshooting.
  Serial.print("RC Fwd: ");
  Serial.print(rcFwd);
  Serial.print(" | RC Turn: ");
  Serial.print(rcTurn);
  Serial.print(" | Left Motor: ");
  Serial.print(leftMotor);
  Serial.print(" | Right Motor: ");
  Serial.println(rightMotor);

  // Short delay before the next loop iteration.
  delay(20);
}
