#include <Servo.h>
 
const int trigPinBall = 3;
const int echoPinBall = 2;
const int servoPin = 30;
 
const int trigPinHand = 4;
const int echoPinHand = 5;
 
Servo throttleServo;
 
// Ultrasonic and ball position
unsigned long durationB;
double distanceB;
double distanceHand = 0;
unsigned long durationHand;
double ballPos = 0;  // Ball position in cm
double ref = distanceHand;   // Reference position in cm
 
// Geometry
const double upperGap = 1;
const double columnL = 75;
const double ballDiam = 4;
const int maxWaitTime = (int)(columnL * 3.5 / 0.034);
 
// Servo motion
const double equilibriumAngle = 34;
const double minAngle = 0;
const double maxAngle = 90;
 
// PID
double controlP = 0;
double controlI = 0;
double controlD = 0;
double control = 0;
double servoAngle = equilibriumAngle;
double err = 0;
double prevErr = 0;
double prevControlI = 0;
double prevBallPos = 0;
bool flagBallPosErr = false;
 
const double samplingTime = 0.025;
const double Kp = 0.7;
const double Ki = 0.0;
const double Kd = 0.8;
 
void setup() {
  pinMode(trigPinBall, OUTPUT);
  pinMode(echoPinBall, INPUT);
  pinMode(trigPinHand, OUTPUT);
  pinMode(echoPinHand, INPUT);
  throttleServo.attach(servoPin);
  throttleServo.write(equilibriumAngle);
 
  Serial.begin(9600);
  Serial.println("Ball Position Control System with Servo Throttle");
  Serial.print("Initial Reference (cm): ");
  Serial.println(ref);
  Serial.print("Equilibrium angle: ");
  Serial.println(equilibriumAngle);
}
 
void loop() {
  unsigned long elapsedTime = millis();
 
  // Keep throttle closed for first 10 seconds
  if (elapsedTime < 5000) {
    throttleServo.write(minAngle);
    Serial.println("Startup phase: Throttle closed to stabilize the system.");
    delay(50);
    return;
  }
 
  // Update reference from serial input (in cm)
  if (Serial.available() > 0) {
    double newRef = Serial.parseFloat();
    if (newRef >= 0 && newRef <= columnL) {
      ref = newRef;
      Serial.print("New reference set (cm): ");
      Serial.println(ref);
    } else {
      Serial.print("Invalid reference. Enter a value between 0 and ");
      Serial.println(columnL);
    }
    while (Serial.available() > 0) Serial.read();
  }
 
  // Measure ultrasonic distance
  digitalWrite(trigPinBall, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinBall, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinBall, LOW);
 
  durationB = pulseIn(echoPinBall, HIGH, maxWaitTime);
  distanceB = durationB * 0.034 / 2;
 
 
  delay(10);
 
  digitalWrite(trigPinHand, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinHand, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinHand, LOW);
 
 
  durationHand = pulseIn(echoPinHand, HIGH);  
  distanceHand = durationHand * 0.034  /2;
 
  if (distanceB == 0) {
    Serial.println("Ultrasonic sensor failed: Ball may be too close or blocked.");
    flagBallPosErr = true;
  } else {
    flagBallPosErr = false;
  }
 
  double rawBallPos = (columnL + upperGap) - (distanceB + ballDiam / 2);
 
  // double rawHandPos = distanceHand;
 
  if (flagBallPosErr || rawBallPos < 0 || rawBallPos > columnL) {
    ballPos = prevBallPos;
  } else {
    ballPos = rawBallPos;
    prevBallPos = ballPos;
  }
 
  calculatePID();
 
  if (ballPos > columnL - 5) {
    Serial.println("Ball near sensor! Capping control to prevent overshoot.");
    control = 20;
    controlI = prevControlI;
  }
 
  servoAngle = mapControlToServoAngle(control);
  throttleServo.write(servoAngle);
 
  if (millis() % 100 < 50) {
    // Serial.print("Ref: ");
    // Serial.print(ref, 1);
    // Serial.print(" cm  Ball: ");
    // Serial.println(ballPos, 1);
    Serial.println(distanceHand);
    // Serial.print(" cm  Echo Dist: ");
    // Serial.print(distanceB, 1);
    // Serial.print(" cm  Control: ");
    // Serial.print(control, 1);
    // Serial.print("%  Servo: ");
    // Serial.print(servoAngle, 1);
    // Serial.print("°  P: ");
    // Serial.print(controlP, 1);
    // Serial.print(" I: ");
    // Serial.print(controlI, 1);
    // Serial.print(" D: ");
    // Serial.println(controlD, 1);
  }
 
  delay(50);
 
}
 
 
double mapControlToServoAngle(double controlValue) {
  if (controlValue < 50) {
    return map(controlValue * 100, 0, 5000, minAngle * 100, equilibriumAngle * 100) / 100.0;
  } else {
    return map(controlValue * 100, 5000, 10000, equilibriumAngle * 100, maxAngle * 100) / 100.0;
  }
}
 
void calculatePID() {
  err = ref - ballPos;
 
  controlP = Kp * err;
  controlI = prevControlI + Ki * err * samplingTime;
  controlD = Kd * (-ballPos + prevBallPos) / samplingTime;
 
  control = 50 + controlP + controlI + controlD;
 
  if (control > 100) {
    control = 100;
    controlI = prevControlI;
  } else if (control < 0) {
    control = 0;
    controlI = prevControlI;
  }
 
  prevControlI = controlI;
  prevErr = err;
}
 
 
