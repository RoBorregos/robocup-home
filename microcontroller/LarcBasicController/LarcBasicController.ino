#include <Arduino.h>
#include <math.h>

bool CHECK_MOTORS = false;
bool CHECK_ENCODERS = false;
bool CHECK_LINES = false;
bool CHECK_BNO = false;

// START: BNO
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
#define BNO055_SAMPLERATE_DELAY_MS (100)
void logAngles() {
  sensors_event_t event;
  bno.getEvent(&event);
  
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
void initBNO() {
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}
// END: BNO

// START: MOTORS

// FRONT LEFT PINOUT
static constexpr uint8_t kDigitalPinsFrontLeftMotor[2] = {49, 48};
static constexpr uint8_t kAnalogPinFrontLeftMotor = 7;
static constexpr uint8_t kEncoderPinsFrontLeftMotor[2] = {2, 5};
// BACK LEFT PINOUT
static constexpr uint8_t kDigitalPinsBackLeftMotor[2] = {50, 51};
static constexpr uint8_t kAnalogPinBackLeftMotor = 6;
static constexpr uint8_t kEncoderPinsBackLeftMotor[2] = {3, 4};
// FRONT RIGHT PINOUT
static constexpr uint8_t kDigitalPinsFrontRightMotor[2] = {47, 46};
static constexpr uint8_t kAnalogPinFrontRightMotor = 8;
static constexpr uint8_t kEncoderPinsFrontRightMotor[2] = {18, 17};
// BACK RIGHT PINOUT
static constexpr uint8_t kDigitalPinsBackRightMotor[2] = {44,45};
static constexpr uint8_t kAnalogPinBackRightMotor = 9;
static constexpr uint8_t kEncoderPinsBackRightMotor[2] = {19, 16};

void initMotor() {
  pinMode(kDigitalPinsFrontLeftMotor[0], OUTPUT);
  pinMode(kDigitalPinsFrontLeftMotor[1], OUTPUT);
  pinMode(kAnalogPinFrontLeftMotor, OUTPUT);
  pinMode(kEncoderPinsFrontLeftMotor[0], INPUT);
  pinMode(kEncoderPinsFrontLeftMotor[1], INPUT);
  pinMode(kDigitalPinsBackLeftMotor[0], OUTPUT);
  pinMode(kDigitalPinsBackLeftMotor[1], OUTPUT);
  pinMode(kAnalogPinBackLeftMotor, OUTPUT);
  pinMode(kEncoderPinsBackLeftMotor[0], INPUT);
  pinMode(kEncoderPinsBackLeftMotor[1], INPUT);
  pinMode(kDigitalPinsFrontRightMotor[0], OUTPUT);
  pinMode(kDigitalPinsFrontRightMotor[1], OUTPUT);
  pinMode(kAnalogPinFrontRightMotor, OUTPUT);
  pinMode(kEncoderPinsFrontRightMotor[0], INPUT);
  pinMode(kEncoderPinsFrontRightMotor[1], INPUT);
  pinMode(kDigitalPinsBackRightMotor[0], OUTPUT);
  pinMode(kDigitalPinsBackRightMotor[1], OUTPUT);
  pinMode(kAnalogPinBackRightMotor, OUTPUT);
  pinMode(kEncoderPinsBackRightMotor[0], INPUT);
  pinMode(kEncoderPinsBackRightMotor[1], INPUT);
}

// ID: 0 = Back Left, 1 = Front Left, 2 = Back Right, 3 = Front Right
// DIRECTION: -1 = Backward, 0 = Stop, 1 = Forward
// PWM: 0-255
#define BACK_LEFT 0
#define FRONT_LEFT 1
#define BACK_RIGHT 2
#define FRONT_RIGHT 3
#define STOP 0
#define FORWARD 1
#define BACKWARD -1
void moveMotor(int id, int direction, int pwm) {
  uint8_t digital_pins[2];
  uint8_t analog_pin;
  switch (id) {
    case BACK_LEFT:
      digital_pins[0] = kDigitalPinsBackLeftMotor[0];
      digital_pins[1] = kDigitalPinsBackLeftMotor[1];
      analog_pin = kAnalogPinBackLeftMotor;
    break;
    case FRONT_LEFT:
      digital_pins[0] = kDigitalPinsFrontLeftMotor[0];
      digital_pins[1] = kDigitalPinsFrontLeftMotor[1];
      analog_pin = kAnalogPinFrontLeftMotor;
    break;
    case BACK_RIGHT:
      digital_pins[0] = kDigitalPinsBackRightMotor[0];
      digital_pins[1] = kDigitalPinsBackRightMotor[1];
      analog_pin = kAnalogPinBackRightMotor;
    break;
    case FRONT_RIGHT:
      digital_pins[0] = kDigitalPinsFrontRightMotor[0];
      digital_pins[1] = kDigitalPinsFrontRightMotor[1];
      analog_pin = kAnalogPinFrontRightMotor;
    break;
  }
  if (direction == FORWARD) {
    digitalWrite(digital_pins[0], HIGH);
    digitalWrite(digital_pins[1], LOW);
  } else if (direction == BACKWARD) {
    digitalWrite(digital_pins[0], LOW);
    digitalWrite(digital_pins[1], HIGH);
  } else {
    digitalWrite(digital_pins[0], LOW);
    digitalWrite(digital_pins[1], LOW);
  }
  analogWrite(analog_pin, pwm);  
}
void moveAll(int direction, int pwm) {
  moveMotor(FRONT_LEFT, direction, pwm);
  moveMotor(FRONT_RIGHT, direction, pwm);
  moveMotor(BACK_LEFT, direction, pwm);
  moveMotor(BACK_RIGHT, direction, pwm);
}
// END: MOTORS

// START: ENCODERS
static constexpr double kPulsesPerRevolution = 300.0;
const int front_left_dir_sign = 1;
int front_left_encoder = 0;
int front_left_dir = 0;

const int front_right_dir_sign = 1;
int front_right_encoder = 0;
int front_right_dir = 0;

const int back_left_dir_sign = 1;
int back_left_encoder = 0;
int back_left_dir = 0;

const int back_right_dir_sign = -1;
int back_right_encoder = 0;
int back_right_dir = 0;
void frontLeftEncoderCallback() {
  front_left_dir = (int)(digitalRead(kEncoderPinsFrontLeftMotor[1]) == HIGH ? front_left_dir_sign : front_left_dir_sign * -1);
  front_left_encoder++;
}
void frontRightEncoderCallback() {
  front_right_dir = (int)(digitalRead(kEncoderPinsFrontRightMotor[1]) == HIGH ? front_right_dir_sign : front_right_dir_sign * -1);
  front_right_encoder++;
}
void backLeftEncoderCallback() {
  back_left_dir = (int)(digitalRead(kEncoderPinsBackLeftMotor[1]) == HIGH ? back_left_dir_sign : back_left_dir_sign * -1);
  back_left_encoder++;
}
void backRightEncoderCallback() {
  back_right_dir = (int)(digitalRead(kEncoderPinsBackRightMotor[1]) == HIGH ? back_right_dir_sign : back_right_dir_sign * -1);
  back_right_encoder++;
}
void initEncoders() {
  attachInterrupt(digitalPinToInterrupt(kEncoderPinsFrontLeftMotor[0]), frontLeftEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(kEncoderPinsFrontRightMotor[0]), frontRightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(kEncoderPinsBackLeftMotor[0]), backLeftEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(kEncoderPinsBackRightMotor[0]), backRightEncoderCallback, RISING);
}
void resetEncoders() {
  front_left_encoder = 0;
  front_right_encoder = 0;
  back_left_encoder = 0;
  back_right_encoder = 0;
}
// END: ENCODERS

// START: LINES
// Order: Middle, Front, Back
static constexpr uint8_t kLineFrontLines[3] = {A7, A8, A9};
static constexpr uint8_t kLineBackLines[3] = {A4, A5, A6};
static constexpr uint8_t kLineLeftLines[3] = {A13, A14, A15};
static constexpr uint8_t kLineRightLines[3] = {A10, A11, A12};
// Get values to string
void logFrontLines() {
  Serial.print("Front: ");
  Serial.print(analogRead(kLineFrontLines[0]));
  Serial.print(" ");
  Serial.print(analogRead(kLineFrontLines[1]));
  Serial.print(" ");
  Serial.print(analogRead(kLineFrontLines[2]));
  Serial.println(" ");
}
void logBackLines() {
  Serial.print("Back: ");
  Serial.print(analogRead(kLineBackLines[0]));
  Serial.print(" ");
  Serial.print(analogRead(kLineBackLines[1]));
  Serial.print(" ");
  Serial.print(analogRead(kLineBackLines[2]));
  Serial.println(" ");
}
void logLeftLines() {
  Serial.print("Left: ");
  Serial.print(analogRead(kLineLeftLines[0]));
  Serial.print(" ");
  Serial.print(analogRead(kLineLeftLines[1]));
  Serial.print(" ");
  Serial.print(analogRead(kLineLeftLines[2]));
  Serial.println(" ");
}
void logRightLines() {
  Serial.print("Right: ");
  Serial.print(analogRead(kLineRightLines[0]));
  Serial.print(" ");
  Serial.print(analogRead(kLineRightLines[1]));
  Serial.print(" ");
  Serial.print(analogRead(kLineRightLines[2]));
  Serial.println(" ");
}
// END: LINES

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // wait for serial port to open!
  initBNO();
  initMotor();
  initEncoders();
}

// Used if ROS disabled.
void loop() {
    if (CHECK_ENCODERS) {
        bool direction = false;
        long long start_time = millis();
        while(1) {
          if (millis() - start_time > 3*1000) {
            start_time = millis();
            direction = !direction;
            Serial.println("Direction Changed");
          }
          if (direction) {
            moveMotor(FRONT_LEFT, FORWARD, 150);
            moveMotor(BACK_LEFT, FORWARD, 150);
            moveMotor(FRONT_RIGHT, FORWARD, 150);
            moveMotor(BACK_RIGHT, FORWARD, 150);
          } else {
            moveMotor(FRONT_LEFT, BACKWARD, 150);
            moveMotor(BACK_LEFT, BACKWARD, 150);
            moveMotor(FRONT_RIGHT, BACKWARD, 150);
            moveMotor(BACK_RIGHT, BACKWARD, 150);
          }
               
          Serial.print("BL ");
          Serial.print((back_left_encoder / kPulsesPerRevolution) * 60 );
          Serial.print(" ");
          Serial.println(back_left_dir);
          Serial.print("FL ");
          Serial.print((front_left_encoder / kPulsesPerRevolution) * 60 );
          Serial.print(" ");
          Serial.println(front_left_dir);
          Serial.print("BR ");
          Serial.print((back_right_encoder / kPulsesPerRevolution) * 60 );
          Serial.print(" ");
          Serial.println(back_right_dir);
          Serial.print("FR ");
          Serial.print((front_right_encoder / kPulsesPerRevolution) * 60 );
          Serial.print(" ");
          Serial.println(front_right_dir);
          Serial.println("\n");
        }
    }
    
    if (CHECK_MOTORS) {
        // Check Motors
        bool direction = false;
        long long start_time = millis();
        while(1) {
          if (direction) {
            moveMotor(FRONT_LEFT, FORWARD, 150);
            moveMotor(BACK_LEFT, FORWARD, 150);
            moveMotor(FRONT_RIGHT, FORWARD, 150);
            moveMotor(BACK_RIGHT, FORWARD, 150);
          } else {
            moveMotor(FRONT_LEFT, BACKWARD, 150);
            moveMotor(BACK_LEFT, BACKWARD, 150);
            moveMotor(FRONT_RIGHT, BACKWARD, 150);
            moveMotor(BACK_RIGHT, BACKWARD, 150);
          }
          delay(1000);
          moveMotor(FRONT_LEFT, STOP, 150);
          moveMotor(BACK_LEFT, STOP, 150);
          moveMotor(FRONT_RIGHT, STOP, 150);
          moveMotor(BACK_RIGHT, STOP, 150);
          delay(1000);
          direction = !direction;
        }
    }
    if (CHECK_BNO) {
        while(1) {
          logAngles();
        }
    }
    if (CHECK_LINES) {
        while(1) {
          logFrontLines();
          logBackLines();
          logLeftLines();
          logRightLines();
          Serial.println();
          delay(1000);
        }
    }
}
