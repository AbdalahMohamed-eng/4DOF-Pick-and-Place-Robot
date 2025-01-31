#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define IR_SENSOR_RIGHT A0
#define IR_SENSOR_LEFT A1
#define ECHO A2
#define TRIG A3
#define MOTOR_SPEED 90

#define SERVOMIN 150
#define SERVOMAX 600
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

int angletoPWM(int angle){
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  Serial.println(pulse);
  return pulse;
}

long duration;
int distance;

// Right motor
int enableRightMotor = 10;
int rightMotorPin1 = 9;
int rightMotorPin2 = 8;

// Left motor
int enableLeftMotor = 11;
int leftMotorPin1 = 7;
int leftMotorPin2 = 6;

// IR sensor variables
int rightIRSensorValue;
int leftIRSensorValue;

bool armlowered = false;
bool pickedUp = false;
unsigned long pickupTime = 0;

// Function prototypes
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed);
void linefollower();
void placeObject();

void setup()
{
  TCCR0B = TCCR0B & B11111000 | B00000010;

  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  rotateMotor(0, 0);

  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);

  pwm.setPWM(0, 0, angletoPWM(0));
  pwm.setPWM(1, 0, angletoPWM(0));
  pwm.setPWM(2, 0, angletoPWM(90));
  pwm.setPWM(3, 0, angletoPWM(100));
}

void loop()
{
  rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delay(50);
  digitalWrite(TRIG, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance = 0.034 / 2 * duration;

  if (distance < 22 && !armlowered && !pickedUp)
  {
    placeObject();
    pickedUp = true;
    pickupTime = millis();
  }
  else if (pickedUp && millis() - pickupTime > 4000)
  {
    pickedUp = false;
    rotateMotor(0, 0);

    pwm.setPWM(0, 0, angletoPWM(0));
    pwm.setPWM(1, 0, angletoPWM(0));
    pwm.setPWM(2, 0, angletoPWM(90));
    pwm.setPWM(3, 0, angletoPWM(35));
  }
  else
  {
    linefollower();
  }
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  }
  else
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  }
  else
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));
}

void linefollower()
{
  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
  {
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
  }
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW)
  {
    rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
  }
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH)
  {
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
  }
  else
  {
    rotateMotor(0, 0);
  }
}

void placeObject()
{
  rotateMotor(0, 0);

  pwm.setPWM(3, 0, angletoPWM(100)); // Open the last servo first
  delay(3000); // Wait for the servo to reach its position
  
  pwm.setPWM(0, 0, angletoPWM(90));
  delay(3000); // Wait for the servo to reach its position
  
  pwm.setPWM(1, 0, angletoPWM(35));
  delay(1000); // Wait before the next servo movement
  
  pwm.setPWM(2, 0, angletoPWM(80));
  
  
  pwm.setPWM(3, 0, angletoPWM(100));
  delay(3000); // Wait for the servo to reach its position
   
  pwm.setPWM(3, 0, angletoPWM(35));
  delay(3000); // Wait before the next servo movement
  

  delay(1000);
  
  pwm.setPWM(0, 0, angletoPWM(0));
  delay(3000); // Wait for the servo to reach its position
  
  pwm.setPWM(1, 0, angletoPWM(0));
  delay(3000); // Wait for the servo to reach its position
  
  pwm.setPWM(2, 0, angletoPWM(90));
  delay(3000); // Wait for the servo to reach its position
  
  pwm.setPWM(3, 0, angletoPWM(35));
  delay(4000); // Keep the last servo closed for 4 seconds
}

