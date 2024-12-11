#include <Arduino.h>
#include <Encoder.h>

// DC Motor
const int PWM = 10; // MD10C PWM pin
const int DIR = 9; // MD10C direction pin 

// Encoders
#define ENCODER_OPTIMIZE_INTERRUPTS
Encoder EncDC(2, 3); // DC Motor Encoder for cart position
Encoder EncPendulum(4, 5); // Pendulum Encoder for pendulum angle
long oldPositionDC  = -999; 
long newPositionDC;
long oldPositionPendulum = -999;
long newPositionPendulum;

// System parameters (Initialize at 0)
float pendulumAngle = 0.0;        // Pendulum angle (radians)
float cartPosition = 0.0;         // Cart position (meters)
float cartVelocity = 0.0;         // Cart velocity (meters/sec)
float pendulumAngularVelocity = 0.0; // Pendulum angular velocity (rad/sec)

// Encoder constants (update based on your setup)
const float encoderResolution = 2400.0; // Counts per revolution (For quadrature encoder: CPR = 4xPPR)
const float wheelRadius = 0.0158;       // Radius of timing belt pulley (meters)
const float dt = 0.005;                 // Sampling time (seconds)

// Desired setpoints
const float desiredCartPosition = 0.0; // Target position for the cart (meters)
const float desiredPendulumAngle = 0.0; // Target angle for the pendulum (radians)

// LQR gain matrix
float K[4] = {-580, -560, 920, 4}; // Gains: [k1, k2, k3, k4] = [cart position, cart velocity, pendulum angle, pendulum velocity]

// Rate Limiter Variables
static float lastU = 0.0; // Last control input (initialize at 0)
const float rateLimit = 50.0; // Maximum allowable change per control step (u error)

// Moving Average Filter for Angular Velocity
const int windowSize = 5; // Average of last 5 values
float angularVelocityHistory[windowSize] = {0};
int angularVelocityIndex = 0;

void setup() {
  // Setup motor control pins
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);

  // Initialize serial monitor for debugging & value display
  Serial.begin(115200);
  delay(5000); // Once uploaded, immediately turn on power supply. Added delay for power supply to come to full 24V for accurate encoder readings. 

  // Reset encoder positions
  EncDC.write(0); // Set initial cart position
  EncPendulum.write(-1200); // Set initial pendulum position (-1200 = hanging down at rest)
  Serial.println("System initialized.");
  delay(5000); // Added delay before LQR balancing loop for user to raise pendulum to inverted state (0 = inverted)
}

void loop() {
  ENCPendulum();
  ENCDC();

  // Read encoder values
  long ENCPendulumCount = newPositionPendulum;
  long ENCDCCount = newPositionDC;

  // Convert encoder counts to translational & rotational physical measurements
  pendulumAngle = (ENCPendulumCount / encoderResolution) * 2 * PI; // radians
  cartPosition = (ENCDCCount / encoderResolution) * wheelRadius;  // meters
  
  // Compute derivatives
  static float lastCartPosition = 0.0;
  static float lastPendulumAngle = 0.0;
  cartVelocity = (cartPosition - lastCartPosition) / dt;
  pendulumAngularVelocity = (pendulumAngle - lastPendulumAngle) / dt;
  lastCartPosition = cartPosition;
  lastPendulumAngle = pendulumAngle;

  // Apply Moving Average Filter to Angular Velocity
  angularVelocityHistory[angularVelocityIndex] = pendulumAngularVelocity;
  angularVelocityIndex = (angularVelocityIndex + 1) % windowSize;
  float smoothedAngularVelocity = 0.0;
  for (int i = 0; i < windowSize; i++) {
    smoothedAngularVelocity += angularVelocityHistory[i];
  }
  pendulumAngularVelocity = smoothedAngularVelocity / windowSize;

  // State error vector
  float xError[4] = {
    cartPosition - desiredCartPosition,   // Cart position error
    cartVelocity,                        // Cart velocity
    pendulumAngle - desiredPendulumAngle, // Pendulum angle error
    pendulumAngularVelocity              // Pendulum angular velocity
  };

  // Compute control input u = -K * xError
  float u = 0.0;
  for (int i = 0; i < 4; i++) {
    u -= K[i] * xError[i];
  }

  // Add damping term for pendulum angle
  float pendulumDampingGain = 50.0; // Fine-tune based on performance
  float dampingTerm = -pendulumDampingGain * pendulumAngularVelocity;
  u += dampingTerm;

  // Apply rate limiter to control input
  if (abs(u - lastU) > rateLimit) {
    u = lastU + (u > lastU ? rateLimit : -rateLimit);
  }
  lastU = u;

  // Apply control input to motor
  float motorSpeed = constrain(abs(u), 0, 200); // Map error to PWM output (PWM range: 0-255)
  digitalWrite(DIR, u >= 0 ? LOW : HIGH);    // Set direction based on sign of u
  analogWrite(PWM, motorSpeed);             // Set speed 

  // Debugging(Value) output to Serial Monitor
  Serial.print("Pendulum Angle: ");
  Serial.print(pendulumAngle);
  Serial.print(" | Cart Position: ");
  Serial.print(cartPosition);
  Serial.print(" | Control Input: ");
  Serial.println(u);

  // LQR balancing loop speed interval 
  delay(5); // Adjust for control loop timing (5ms = 200Hz) 
}

// Pendulum Encoder value reader
void ENCPendulum() {
  newPositionPendulum = EncPendulum.read();
  if (newPositionPendulum != oldPositionPendulum) {
    oldPositionPendulum = newPositionPendulum;
  }
}

// DC Motor Encoder value reader
void ENCDC() {
  newPositionDC = EncDC.read();
  if (newPositionDC != oldPositionDC) {
    oldPositionDC = newPositionDC;
  }
}
