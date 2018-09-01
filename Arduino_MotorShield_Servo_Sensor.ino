// Andy's Sensor Robot
//
// Arduino Uno Chip
// Motor shield
// SG90 Micro Servo
// HC-SR04 Untrasonic Sensor
// 2x DC Motors 
// 


#include <AFMotor.h>
#include <Servo.h> 

/*
 * Motor Setup
 */
// Initialise motors
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);

// Motor Speeds
const int fullSpeed = 250;
const int fullSpeedM1 = 250;
const int fullSpeedM2 = 200;
const int halfSpeed = 125;
const int quarterSpeed = 62;
const int stopSpeed= 0;
bool forwarding = true;


/*
 * Helper Methods
 * For Robot Movement
 */
void stopRobot() {
  Serial.println("Motor Stop");
//  motor1.run(BRAKE);
//  motor2.run(BRAKE);
  motor1.setSpeed(stopSpeed);  
  motor2.setSpeed(stopSpeed); 
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  delay(10);
}

void forwardRobot() {
  Serial.println("Motor Forward");
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor1.setSpeed(fullSpeedM1);  
  motor2.setSpeed(fullSpeedM2);  
}

void forwardLeftRobot() {
  Serial.println("Motor forward Left");
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor1.setSpeed(fullSpeed);
  motor2.setSpeed(quarterSpeed);  
}

void forwardRightRobot() {
  Serial.println("Motor forward Right");
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor1.setSpeed(quarterSpeed);
  motor2.setSpeed(fullSpeed);  
}

void reverseRobot() {
  Serial.println("Motor Reverse");
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor1.setSpeed(fullSpeed);
  motor2.setSpeed(fullSpeed);  
}

void reverseLeftRobot() {
  Serial.println("Motor Reverse Left");
  motor1.setSpeed(fullSpeed);
  motor2.setSpeed(quarterSpeed); 
  motor1.run(BACKWARD);
  motor2.run(BACKWARD); 
}

void reverseRightRobot() {
  Serial.println("Motor Reverse Right");
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor1.setSpeed(quarterSpeed);
  motor2.setSpeed(fullSpeed);  
}

/* 
 *  Helper Methods
 *  For Sensor
 */
// Check for significant jumps and outliers in sensor reading
bool outlierCheck(int lastSensorDistance, int thisSensorDistance) {

  // if the difference between measures is greater then 20cm, outlier found
  if ((lastSensorDistance - thisSensorDistance) > 20) {
    return true;
  } else {
    return false;
  }
}



/*
 * Sensor Setup
 */
#include <Ultrasonic.h>
#define TRIGGER_PIN   13
#define ECHO_PIN      12
Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);
int lastSensorDistance = 0;
int thisSensorDistance = 0;


/* 
 *  Servo Setup
 */
Servo servo1;

/* 
 * Servo Helper Functions
 * It takes the Servo ~500ms to move from 0 to 180
 */
int servo_full_right() {
  servo1.write(0);
  delay(500);
  Serial.println("Servo Full Right");
  return ultrasonic.Ranging(CM);
}

int servo_mid_right() {
  servo1.write(45);
  delay(500);
  Serial.println("Servo Mid Right");
  return ultrasonic.Ranging(CM);
}

int servo_middle() {
  servo1.write(90);
  delay(500);
  Serial.println("Servo Middle");
  return ultrasonic.Ranging(CM);
}

int servo_mid_left() {
  servo1.write(135);
  delay(500);
  Serial.println("Servo Mid Left");
  return ultrasonic.Ranging(CM);
}

int servo_full_left() {
  servo1.write(180);
  delay(500);
  Serial.println("Servo Full Left");
  return ultrasonic.Ranging(CM);
}

void servo_look_around() {

  int mid = servo_middle();
  int mid_left = servo_mid_left();
  int full_left = servo_full_left();
  servo_mid_left();
  servo_middle();
  int mid_right = servo_mid_right();
  int full_right = servo_full_right();
  servo_mid_right();
  servo_middle();

  if ( (mid + mid_left + full_left) > (mid + mid_right + full_right) ) {
    Serial.print(mid + mid_left + full_left);
    Serial.println(" --> Danger to the left");
  } else {
    Serial.print(mid + mid_right + full_right);
    Serial.println(" --> Danger to the right");
  }

  delay(3000);
  exit(0);

}

void sweep() {
  
  servo_middle();
  
  for(int pos = 90; pos <= 180; pos += 1) {                                  
    servo1.write(pos);              
    delay(10);                      
  } 
  
  for(int pos = 180; pos>=0; pos-=1) {                                
    servo1.write(pos);              
    delay(10);                      
  } 

  for(int pos = 0; pos <= 90; pos += 1) {                                  
    servo1.write(pos);              
    delay(10);                      
  } 
  
}

/*
 * Global Setup
 */
void setup() {

  // Set up Serial output
  Serial.begin(9600);           
  Serial.println("**********\nMotor Setup...");

  // turn on and stop motors
  Serial.println("Motors on, stop, and release");
  stopRobot();

  // turn on servo and center it
  Serial.println("Servo on, center, and delay");
  servo1.attach(9);
  servo1.write(90);
  delay(500);
  sweep();
  
  Serial.println("Setup Complete\n**********");
}

/*
 * Main Method
 */
void loop() {
  Serial.println("##########\nSTART LOOP");

  // Copy over last distance
  lastSensorDistance = thisSensorDistance;
  
  // Get new this distance
  thisSensorDistance = ultrasonic.Ranging(CM);

  // Check for outliers from sensor
  if (outlierCheck(lastSensorDistance, thisSensorDistance)) {
    Serial.println("Outlier found: " + String(thisSensorDistance));
  } else {
    Serial.println(String(thisSensorDistance) + " cm");

    // React if we're closer than 25cm
    if (thisSensorDistance <= 25) {

      // Scan full area here
      stopRobot();
      servo_look_around();
      



      // If you've been moving forward then stop before reversing
      if (forwarding) {
        Serial.println("Forwarding changed to reversing");
        stopRobot();
        forwarding = false;
//        delay(1000); 
      } else {
        Serial.println("Already reversing, continuing reverse");
        reverseLeftRobot();
        delay(500); 
      }
      
    } else {

      // If you've been moving backwards then stop before forwarding
      if (!forwarding) {
        stopRobot();
        forwarding = true;
        Serial.println("Reversing changed to forwarding");
      }
      
      forwardRobot();
      
    }
  
  }

  Serial.println("END LOOP\n##########");
}
