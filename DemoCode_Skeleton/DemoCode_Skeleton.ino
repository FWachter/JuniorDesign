// Programmer: Frederick Wachter - wachterfreddy@gmail.com
// Date Created: November 25th, 2016

#include <Servo.h>
#include <Wire.h>
#include <NewPing.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define MOTOR1_PWM 11
#define MOTOR2_PWM 3
#define MOTOR3_PWM 6
#define MOTOR4_PWM 5
#define SERVO1_PWM 10
#define SERVO2_PWM 9

#define SERVO1_FULLSPEED_C 135
#define SERVO1_FULLSPEED_CC 45
#define SERVO1_MAXTURN 180
#define SERVO1_180DEG 315
#define SERVO1_STOP 90

#define US_TRIGGER_PIN  5  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define US_ECHO_PIN     6  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define US_MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define ACCEL_SAMPLERATE_DELAY_MS 100

Servo servo1; // setup servo
NewPing sonar(US_TRIGGER_PIN, US_ECHO_PIN, US_MAX_DISTANCE); // setup ultrasonic sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55); // setup accelerometer

void setup() {

  Serial.begin(9600);

  // Initialize Servo
  servo1.attach(SERVO1_PWM);

  // Initilaize Accelerometer
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);

}

void loop() {

  

}

// ---------------------------------------
// ---------- EXAMPLE FUNCTIONS ----------
// ---------------------------------------

void readAccelerometerExample() {
// Programmer: Frederick Wachter - wachterfreddy@gmail.com
// Date Created: November 25th, 2016
// Purpose: Example of reading from the accelerometer

  sensors_event_t event = getAccelerometerData(bno);
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println();
  
}

void readUltrasonicExample() {
// Programmer: Frederick Wachter - wachterfreddy@gmail.com
// Date Created: November 25th, 2016
// Purpose: Example of reading from the ultrasonic sensor

  unsigned int distance = getUltrasonicData(sonar);
  Serial.println(distance); 
  
}

void moveServoExample() {
// Programmer: Frederick Wachter - wachterfreddy@gmail.com
// Date Created: November 25th, 2016
// Purpose: Example of using the moveServo() function
 
  moveServo(servo1, 180);
  Serial.println("Position 1");
  delay(1000);
  moveServo(servo1, -180);
  Serial.println("Position 2");
  delay(1000);
  
}

// --------------------------------------
// ---------- SENSOR FUNCTIONS ----------
// --------------------------------------

sensors_event_t getAccelerometerData(Adafruit_BNO055 &bno) {
// Programmer: Frederick Wachter - wachterfreddy@gmail.com
// Date Created: November 25th, 2016
// Purpose: Get data from accelerometer

  delay(ACCEL_SAMPLERATE_DELAY_MS);
  sensors_event_t event;
  bno.getEvent(&event);

  return event;
 
}

unsigned int getUltrasonicData(NewPing &sonar) {
// Programmer: Frederick Wachter - wachterfreddy@gmail.com
// Date Created: November 25th, 2016
// Purpose: Get data from ultrasonic sensor

  delay(30); // ensure that this function is not being called too often
  unsigned int distance = sonar.ping(); // send ping, get ping time in microseconds
  distance /= US_ROUNDTRIP_CM; // convert ping time to distance

  if (distance == 0) { distance = US_MAX_DISTANCE; } // if the distance is maxed out, set the distance value to max
  return distance;
 
}

void moveServo(Servo &servo, long int turn_degrees) {
// Programmer: Frederick Wachter - wachterfreddy@gmail.com
// Date Created: November 25th, 2016
// Purpose: Move the provided servo to a provided position

  // Initilaize Variables
  int wait_time;

  // Calculations
  turn_degrees = min(max(-SERVO1_MAXTURN, turn_degrees), SERVO1_MAXTURN); // make sure the provided turn degree is within +-180 degrees
  wait_time = abs((turn_degrees*SERVO1_180DEG)/180); // calculate the amount of time the servo needs to be moving to get to position

  // Set Direction of Servo
  if (turn_degrees > 0) {
    // turn the servo clockwise
    servo.write(SERVO1_FULLSPEED_C);
  } else {
    // turn the servo counter clockwise
    servo.write(SERVO1_FULLSPEED_CC);
  }

  // Wait for Servo to be at Desired Position then Stop
  delay(wait_time);
  servo.write(SERVO1_STOP);
  
}

