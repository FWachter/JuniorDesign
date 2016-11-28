// Programmer: Frederick Wachter - wachterfreddy@gmail.com
// Date Created: November 25th, 2016

#include <Servo.h>
#include <Wire.h>
#include <NewPing.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define SERVO1_PWM 10
#define SERVO2_PWM 11
#define SERVO1_START 120
#define SERVO1_MAX 0
#define SERVO2_START 0
#define SERVO2_MAX 100

#define IR_PIN_LONG 2
#define IR_PIN_SHORT 3

#define US_TRIGGER_PIN  5  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define US_ECHO_PIN     6  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define US_MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define ACCEL_SAMPLERATE_DELAY_MS 100
#define ORIENT_Z_FLIPPED 30

Servo servo1, servo2; // setup servos
NewPing sonar(US_TRIGGER_PIN, US_ECHO_PIN, US_MAX_DISTANCE); // setup ultrasonic sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55); // setup accelerometer

void setup() {

  Serial.begin(9600);

  // Initialize Servo
  servo1.attach(SERVO1_PWM);
  servo2.attach(SERVO2_PWM);

  // Initialize IR Sensor
  pinMode(IR_PIN_LONG, INPUT);
  pinMode(IR_PIN_SHORT, INPUT);

  // Initilaize Accelerometer
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);

  resetMotors();

}

void loop() {

  controlLoop();

}

// ---------------------------------------
// ---------- EXAMPLE FUNCTIONS ----------
// ---------------------------------------

void readIRSensorExample(int pin) {
// Programmer: Frederick Wachter - wachterfreddy@gmail.com
// Date Created: November 26th, 2016
// Purpose: Example of reading from the IR sensor

  bool value = getIRSensorData(pin);
  Serial.println(value);
  
}

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


// -------------------------------------
// ---------- LOGIC FUNCTIONS ----------
// -------------------------------------

void flipOver() {
// Programmer: Frederick Wachter - wachterfreddy@gmail.com
// Date Created: November 27th, 2016
// Purpose: Run through sequence to flip over crab

  moveServo(servo2, SERVO2_MAX);
  delay(2000);
  moveServo(servo1, SERVO1_MAX);
  delay(200);
  moveServo(servo2, SERVO2_START);
  delay(2000);
  resetMotors();
  
}

void flipOverFast() {
// Programmer: Frederick Wachter - wachterfreddy@gmail.com
// Date Created: November 27th, 2016
// Purpose: Run through sequence to flip over crab fast

  moveServo(servo2, SERVO2_MAX);
  delay(500);
  moveServo(servo1, SERVO1_MAX);
  delay(100);
  moveServo(servo2, SERVO2_START);
  delay(1000);
  resetMotors();
  
}

void controlLoop() {
// Programmer: Frederick Wachter - wachterfreddy@gmail.com
// Date Created: November 25th, 2016
// Purpose: Control loop for flipping

  if (isFlipped()) {
    if (getIRSensorData(IR_PIN_SHORT)) {
      flipOverFast();
    } else if (getIRSensorData(IR_PIN_LONG)) {
      bool flipFast = 0;
      unsigned long time_start = millis();
      
      while ((millis() - time_start) < 500) {
        if (getIRSensorData(IR_PIN_SHORT)) {
          flipFast = 1;
          break;
        }
      }

      if (flipFast) {
        flipOverFast();
      } else {
        flipOver();
      }
    }
  }
}

bool isFlipped() {
// Programmer: Frederick Wachter - wachterfreddy@gmail.com
// Date Created: November 25th, 2016
// Purpose: Check if the horseshoe crab is flipped upside down

  bool is_flipped = 0;

  sensors_event_t event = getAccelerometerData(bno);
  if (abs(event.orientation.z) < ORIENT_Z_FLIPPED) {
    is_flipped = 1;
  }

  return is_flipped;
  
}

void resetMotors() {
// Programmer: Frederick Wachter - wachterfreddy@gmail.com
// Date Created: November 27th, 2016
// Purpose: Reset motor positions

  servo1.write(SERVO1_START);
  servo2.write(SERVO2_START);
  delay(1000);
  
}


// --------------------------------------
// ---------- SENSOR FUNCTIONS ----------
// --------------------------------------

bool getIRSensorData(int pin) {
// Programmer: Frederick Wachter - wachterfreddy@gmail.com
// Date Created: November 26th, 2016
// Purpose: Get data from the IR sensor

  bool value = !digitalRead(pin);
  return value;

}

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

void moveServo(Servo &servo, int turn_degrees) {
// Programmer: Frederick Wachter - wachterfreddy@gmail.com
// Date Created: November 25th, 2016
// Purpose: Move the provided servo to a provided position

  turn_degrees = max(0, turn_degrees);

  servo.write(turn_degrees);
  
}


// -----------------------------------------
// ---------- DEPICATED FUNCTIONS ----------
// -----------------------------------------

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
