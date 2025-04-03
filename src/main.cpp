#include <Arduino.h>
#include <QTRSensors.h>
#include <SparkFun_TB6612.h>
#include <TaskScheduler.h>
#include <QuickPID.h>
//#include <jled.h>

#define HISTORY_SIZE 20
#define NO_LINE_COUNT 10
//#define AGGRESSIVE_PID 1

// Button pin definitions
#define CALIBRATION_BUTTON_PIN 3
#define RUN_BUTTON_PIN 2

// Motor Pin definitions
#define AIN1 7
#define BIN1 9
#define AIN2 6
#define BIN2 10
#define PWMA 5
#define PWMB 11
#define STBY 8

const int offsetA = 1;
const int offsetB = 1;

// Create motor objects
Motor motorRight = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motorLeft = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// Speed limits
int leftMinSpeed = 0;
int rightMinSpeed = 0;
int leftMaxSpeed = 150;  // Limited to 100 to minimize oscillations
int rightMaxSpeed = 150;

// Sensor definitions
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t adjustedValue[SensorCount];
float sensorWeights[SensorCount] = { -12.5, -2.5, -0.5, -0.1, 0.1, 0.5, 2.5,12.5 };


uint16_t readingHistory[SensorCount][HISTORY_SIZE];

QTRSensors qtr;

Scheduler runner;

int32_t lineError = 0;
uint16_t lineThreshold = 0;
uint8_t noLineCount = 0;
int lastKnownDirection = 0;

// PID variables
float Kp = 10;
float Ki = 0;
float Kd = 2.5;
/*
#ifdef AGGRESSIVE_PID
float AggressiveKp = 5.5;
float AggressiveKi = 0.1;
float AggressiveKd = 1;
#endif
*/

float setPointLeft = 0;
float inputLeft = 0;
float outputLeft = 0;

float setPointRight = 0;
float inputRight = 0;
float outputRight = 0;

// QuickPID instances
QuickPID leftPid(&inputLeft, &outputLeft, &setPointLeft, Kp, Ki, Kd, QuickPID::Action::direct);
QuickPID rightPid(&inputRight, &outputRight, &setPointRight, Kp, Ki, Kd, QuickPID::Action::direct);

// Robot state
enum RobotState { IDLE, CALIBRATION, RUNNING };
RobotState currentState = IDLE;

// Debouncing variables
unsigned long lastCalibrationButtonPress = 0;
unsigned long lastRunButtonPress = 0;
const unsigned long debounceDelay = 50;


// Previous motor speeds
int prevLeftMotorSpeed = 0;
int prevRightMotorSpeed = 0;

// Previous line error
int prevLineError = 0;

// Maximum allowed difference in error to detect sharp turns
const int maxErrorDiffThreshold = 2000;

bool isWhiteLine = false;

// Function declarations
void detectBackground();
void readAndCalculateError();
void calculateMotorSpeed();
void setMotorSpeed();
void recordHistory();
void print();

// Task definitions with reduced intervals for faster response
Task detectBackgroundTask(5, TASK_FOREVER, &detectBackground, &runner, true);
Task readAndCalculateErrorTask(20, TASK_FOREVER, &readAndCalculateError, &runner, true);
Task calculateMotorSpeedTask(10, TASK_FOREVER, &calculateMotorSpeed, &runner, true);
Task setMotorSpeedTask(10, TASK_FOREVER, &setMotorSpeed, &runner, true);
Task recordHistoryTask(10, TASK_FOREVER, &recordHistory, &runner, true);
Task printTask(1, TASK_FOREVER, &print, &runner, true);



// Button press handlers
void handleCalibrationButtonPress() {
  if (millis() - lastCalibrationButtonPress > debounceDelay) {
    lastCalibrationButtonPress = millis();
    if (currentState == IDLE) {  // Calibration is only allowed in the IDLE state
      currentState = CALIBRATION;
    }
  }
}

void handleRunButtonPress() {
  if (millis() - lastRunButtonPress > debounceDelay) {
    lastRunButtonPress = millis();
    if (currentState == IDLE) {  // Transition from IDLE to RUNNING
      currentState = RUNNING;
      Serial.println("Switching to RUNNING mode...");
    }
     else if (currentState == RUNNING) {  // Transition from RUNNING to IDLE (stop robot)
      currentState = IDLE;
      motorLeft.drive(0);
      motorRight.drive(0);
      Serial.println("Switching to IDLE mode...");
    }
  }
}




// Function to set PWM frequency for Timer2 (PWMA - pin 11)
void setPWMFrequency_Timer2() {
  // Set Timer2 prescaler to 8 for ~7.8 kHz PWM frequency on pin 11
  TCCR2B = (TCCR2B & 0b11111000) | 0x02;  // Prescaler 8
}

void print() {
  Serial.print("LE: ");
  Serial.print(inputLeft);
  Serial.print("\t");
  Serial.print(outputLeft);
  Serial.print("\t RE:");
  Serial.print(inputRight);
  Serial.print("\t");
  Serial.print(outputRight);
  Serial.print("\t LMS: ");
  Serial.print(150 - prevLeftMotorSpeed);
  Serial.print("\t RMS: ");
  Serial.print(150 - prevRightMotorSpeed);
  Serial.print("\t\t\t");
  for (char i = 0; i < SensorCount; i++) {
    Serial.print("\t ");
    Serial.print(sensorValues[i]);
  }
  Serial.print("\t\t");
  Serial.print(lineError);
  Serial.println();
}
int findWhereLastLineWentFromHistory() {
  int32_t historyLineError = 0;
  for (int l = HISTORY_SIZE - 1; l >= 0; l--) {
    for (int i = 0; i < SensorCount; i++) {
      historyLineError += sensorValues[i] * sensorWeights[i] * (l / 10.0);
    }
  }
  if (historyLineError > 50) return 1;
  if (historyLineError < -50) return -1;
  return 0;
}





void detectBackground() {
    //int sensorSum = 0;
/*
    // Read all sensor values
    for (int i = 0; i < SensorCount; i++) {
        sensorSum += sensorValues[i];  // Replace with your sensor array readings
    }

    // Calculate average sensor reading
    int average = sensorSum / SensorCount;
*/
  // Determine the background based on average sensor reading
    if( (sensorValues[0] > 900) &&(sensorValues[7] > 900)){
      isWhiteLine = true;
    }
    else if((sensorValues[0] < 900) &&(sensorValues[7] < 900)) {
      isWhiteLine = false;
    }

    Serial.println(isWhiteLine);
}


void readAndCalculateError() {
    qtr.readCalibrated(sensorValues);
    bool lineExists = false;
    lineError = 0;

       // Process sensor readings
    for (int i = 0; i < SensorCount; i++) {
        adjustedValue[i] = isWhiteLine ? ((lineThreshold * 2) - sensorValues[i]) : sensorValues[i];

        if (adjustedValue[i] > lineThreshold) {
            lineError += adjustedValue[i] * sensorWeights[i];
            lineExists = true;
        }
    }
    
  if (!lineExists) {  // No line detected
    noLineCount++;
    if (noLineCount > NO_LINE_COUNT) {
      // Rely on history only when we lose the line
      lineError = findWhereLastLineWentFromHistory() * 5000;
    } else {
      // Retain last known direction to prevent abrupt changes
      lineError = lastKnownDirection * 5000;
    }
  } else {
    noLineCount = 0;  // Reset no-line counter

    // Use a minimum threshold to avoid setting lineError to zero abruptly
    if (abs(lineError) < 500) {              // Small error likely means a straight line
      lineError = lastKnownDirection * 500;  // Retain a small error for stability
    } else {
      lastKnownDirection = (lineError > 0) ? 1 : -1;  // Update direction for curves
    }
  }
}


void recordHistory() {
  uint8_t count = 0;

  for (int i = 0; i < SensorCount; i++)
    if (sensorValues[i] > lineThreshold)
      count++;

  if (count == 0) return; // Skip if no line detected

  for (int i = 0; i < SensorCount; i++) {
    for (int j = HISTORY_SIZE - 1; j > 0; j--) {
      readingHistory[i][j] = readingHistory[i][j - 1];
    }
    readingHistory[i][0] = sensorValues[i]; // Add the latest value to history
  }
}


void setMotorSpeed() {
  // Calculate the difference between the current error and the previous error
  int errorDifference = abs(lineError - prevLineError);

  // Constrain motor outputs to defined limits
  int leftMotorSpeed = constrain(outputLeft, leftMinSpeed, leftMaxSpeed);
  int rightMotorSpeed = constrain(outputRight, rightMinSpeed, rightMaxSpeed);

  // Check if error difference exceeds the threshold (indicating a sharp turn)
  if (errorDifference > maxErrorDiffThreshold) {
    // Reduce speeds for smoother turning
    leftMotorSpeed /= 2;   // Decrease speed for left motor
    rightMotorSpeed /= 2;  // Decrease speed for right motor
  }

  // Update motors
  motorLeft.drive(150 - leftMotorSpeed);
  motorRight.drive(150 - rightMotorSpeed);

  // Store current motor speeds and error for future use
  prevLeftMotorSpeed = leftMotorSpeed;
  prevRightMotorSpeed = rightMotorSpeed;
  prevLineError = lineError;
}

void calculateMotorSpeed() {
  // Map lineError for scaled output (-100 to 100 range)
  int scaledError=0;
  if (isWhiteLine){
    scaledError = map(lineError, -15000, 15000, 150, -150);
  }else{
    scaledError = map(lineError, -15000, 15000, -150, 150);
  }
  
/*
  // Apply aggressive tuning for large errors
  if (abs(lineError) > 20000) {
    rightPid.SetTunings(AggressiveKp, AggressiveKi, AggressiveKd);
    leftPid.SetTunings(AggressiveKp, AggressiveKi, AggressiveKd);
  } else {
    rightPid.SetTunings(Kp, Ki, Kd);
    leftPid.SetTunings(Kp, Ki, Kd);
  }
*/
    rightPid.SetTunings(Kp, Ki, Kd);
    leftPid.SetTunings(Kp, Ki, Kd);


  // Set PID inputs
  inputRight = scaledError;
  inputLeft = -scaledError;

  // Compute PID outputs
  rightPid.Compute();
  leftPid.Compute();

  // Adjust output for smoother curves
  if ((lineError > 0) && (!isWhiteLine)){  // Turn right
    outputLeft += abs(scaledError / 4);
    outputRight -= abs(scaledError / 4);
  } else if((lineError < 0)&& (!isWhiteLine)) {  // Turn left
    outputRight += abs(scaledError / 4);
    outputLeft -= abs(scaledError / 4);
  } else if ((lineError > 0) && (isWhiteLine)){  // Turn right
    outputRight += abs(scaledError / 4);
    outputLeft -= abs(scaledError / 4);
  } else if((lineError < 0)&& (isWhiteLine)) {  // Turn left
    outputLeft += abs(scaledError / 4);
    outputRight -= abs(scaledError / 4);
  }

  // Apply a dead zone to prevent minor oscillations
  if (abs(lineError) < 1500) {
    outputLeft = 0;
    outputRight = 0;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
  pinMode(CALIBRATION_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RUN_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CALIBRATION_BUTTON_PIN), handleCalibrationButtonPress, FALLING);
  attachInterrupt(digitalPinToInterrupt(RUN_BUTTON_PIN), handleRunButtonPress, FALLING);
 
  // Set PWM frequency for Timer2 (PWMA - pin 11)
  setPWMFrequency_Timer2();

  // Initialize sensors
  qtr.setTypeRC();
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A7, A6, A5, A4, A3, A2, A1, A0 }, SensorCount);
  qtr.setEmitterPin(13);

  // led.Stop(JLed::eStopMode::FULL_OFF);

  // Initialize PID controllers
  leftPid.SetMode(QuickPID::Control::automatic);
  rightPid.SetMode(QuickPID::Control::automatic);

}
void loop() {
  if (currentState == IDLE) {
    // Do nothing or maintain an idle state
  } 
  else if (currentState == CALIBRATION) {
    // Calibrate sensors
    Serial.println("Calibrating...");
    for (uint8_t i = 0; i < 200; i++) {
      qtr.calibrate();
      delay(10);
    }
    Serial.println("Calibration done.");

    // Calculate line detection threshold
    uint16_t minThreshold = 0, maxThreshold = 0;
    for (uint8_t i = 0; i < SensorCount; i++) {
      minThreshold += qtr.calibrationOn.minimum[i];
      maxThreshold += qtr.calibrationOn.maximum[i];
    }
    lineThreshold = (minThreshold + maxThreshold) / (2 * SensorCount);
    Serial.println(lineThreshold);
    
    

    // Transition back to IDLE after calibration
    currentState = IDLE;
  } 
  else if (currentState == RUNNING) {
    // Execute tasks for the RUNNING state
    runner.execute();
  }
}
