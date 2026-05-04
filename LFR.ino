#include <QTRSensors.h>         // QTR sensor library

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


float Kp = 0.07;     // Proportional term
float Ki = 0.0008;    // Integral term
float Kd = 0.7;       // Derivative term
int P, I, D;
int lastError = 0;


const uint8_t maxspeeda = 80;
const uint8_t maxspeedb = 80;
const uint8_t basespeeda = 50;
const uint8_t basespeedb = 50;


#define AIN1 7    // Motor A direction pin 1
#define AIN2 5    // Motor A direction pin 2
#define PWMA 9    // Motor A PWM speed control
#define BIN1 4    // Motor B direction pin 1
#define BIN2 3    // Motor B direction pin 2
#define PWMB 6    // Motor B PWM speed control
#define STBY 1    // Standby pin (not used in L298N)


const int offsetA = 1;
const int offsetB = 1;


int buttoncalibrate = 11;     // Calibration button
int buttonstart = 10;         // Start/stop button
boolean onoff = false;


void setup() {
  // QTR sensor setup with your pin connections
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(2);  // QTR LED emitter control pin

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);

  // Use internal pull-up resistors for buttons
  pinMode(buttoncalibrate, INPUT_PULLUP);
  pinMode(buttonstart, INPUT_PULLUP);

  // Motor pins for L298N
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);   // Always enabled

  // Calibration process
  boolean Ok = false;
  while (!Ok) {  
    if (digitalRead(buttoncalibrate) == LOW) {  // Button pressed (LOW)
      calibration();
      Ok = true;
    }
  }
  stopMotors();  // Stop motors after calibration
}


void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}


void loop() {
  if (digitalRead(buttonstart) == LOW) {  // Button pressed
    onoff = !onoff;
    if (onoff) {
      delay(1000);  // 1 sec delay when robot starts
    } else {
      delay(50);    // Short delay when robot stops
    }
  }

  if (onoff) {
    PID_control();
  } else {
    stopMotors();
  }
}


void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 3500;  // Center position is 3500

  P = error;
  I += error;
  D = error - lastError;
  lastError = error;

  int motorspeed = P * Kp + I * Ki + D * Kd;

  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;

  // Constrain motor speeds
  motorspeeda = constrain(motorspeeda, 0, maxspeeda);
  motorspeedb = constrain(motorspeedb, 0, maxspeedb);

  forward_drive(motorspeeda, motorspeedb);
}

void forward_drive(int posa, int posb) {
  // Left Motor
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, posa);

  // Right Motor
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, posb);
}

// Renamed brake function to avoid conflict
void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}