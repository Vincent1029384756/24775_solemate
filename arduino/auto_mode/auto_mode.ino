#include <Arduino.h>
#include <Basicmicro.h>

// Define the serial port for communication with the motor controller
// On boards like Mega, Due, Leonardo, etc., use Serial1, Serial2, Serial3.
#define CONTROLLER_SERIAL   Serial1 // pin 18 (Tx1) -> S1 (RX) pin 19 (RX) <- S2 (TX)

// Optional: Use SoftwareSerial on AVR boards if HardwareSerial is not available
// #include <SoftwareSerial.h>
// #define RX_PIN 10 // Connect to controller's TX pin
// #define TX_PIN 11 // Connect to controller's RX pin
// SoftwareSerial controllerSerial_SW(RX_PIN, TX_PIN);
// #define CONTROLLER_SERIAL   controllerSerial_SW // Use this define instead of Serial1

// Define the address of your motor controller
#define MOTOR_ADDRESS       128

// Define the library's internal read timeout in microseconds
#define LIBRARY_READ_TIMEOUT 10000

// Instantiate the Basicmicro library object
// If using SoftwareSerial, uncomment the #define above and use controllerSerial_SW
Basicmicro controller(&CONTROLLER_SERIAL, LIBRARY_READ_TIMEOUT);

// Define example target speeds for Motor 1 and Motor 2 in counts/sec (signed 32-bit)
int32_t motor2_speed = 960; // counts/sec 

int32_t motor1_speed = 150;
uint32_t motor1_accel = 1000;
uint32_t motor1_deccel = 1000;
uint8_t motor1_flag = 0;

const int32_t M1_POS_0   = 0;
const int32_t M1_POS_120 = 704;
const int32_t M1_POS_240 = 1408;

// instantiate control flag
uint8_t  control_flag = 0;

// Define buffer control flag (0 = Immediate Execution, 1 = Add to Buffer)
uint8_t buffer_mode = 1; // with buffer

// define state, 1 =  continuously running, 2 =  foot switching
int32_t state = 1;

// for sending M2 commands periodically
unsigned long lastM2CommandTime = 0;
const unsigned long m2CommandIntervalMs = 100;

int32_t currentSole = 0; // variable for storing current sole type
bool switchInProgress = false; // boolean for whether or not sole switching is in progress

// ------- Motor 3 functions------- //
volatile long encoderCount = 0;

const int INA1 = 2;
const int INA2 = 4;
const int PWMA = 5;
const int STBY = 9;

const int encA = 21;   // Yellow
const int encB = 20;   // White

const float countsPerRev = 400.0;
const float targetRPM = 50.0;   // Desired target speed

float Kp = 1.5;
float Ki = 3.0;

float integral = 0.0;
int pwmCmd = 0;

const int maxPWM = 180;         // Limit the maximum PWM to avoid excessive no-load speed
const int minEffectivePWM = 20; // Minimum effective PWM required to overcome static friction; tune as needed

const unsigned long sampleTimeMs = 50;
unsigned long lastControlTime = 0;
long lastEncoderCount = 0;

bool state1Started = false;
int32_t state1StartEnc2 = 0;

void stopMotor3() {
  analogWrite(PWMA, 0);
  integral = 0.0;
  pwmCmd = 0;
}

void setupMotor3Direction() {
  digitalWrite(STBY, HIGH);
  digitalWrite(INA1, LOW);
  digitalWrite(INA2, HIGH);
}

void updateMotor3Control() {
  unsigned long now = millis();

  if (now - lastControlTime >= sampleTimeMs) {
    float dt = (now - lastControlTime) / 1000.0;
    lastControlTime = now;

    long currentCount;
    noInterrupts();
    currentCount = encoderCount;
    interrupts();

    long deltaCount = currentCount - lastEncoderCount;
    lastEncoderCount = currentCount;

    float rpm = (deltaCount / countsPerRev) / dt * 60.0;

    float error = targetRPM - rpm;
    integral += error * dt;

    if (integral > 50.0) integral = 50.0;
    if (integral < -50.0) integral = -50.0;

    float control = Kp * error + Ki * integral;
    pwmCmd += (int)control;

    if (pwmCmd > maxPWM) pwmCmd = maxPWM;
    if (pwmCmd < 0) pwmCmd = 0;

    if (pwmCmd > 0 && pwmCmd < minEffectivePWM) {
      pwmCmd = minEffectivePWM;
    }

    analogWrite(PWMA, pwmCmd);
  }
}

void readA() {
  if (digitalRead(encA) == digitalRead(encB)) encoderCount++;
  else encoderCount--;
}

void readB() {
  if (digitalRead(encA) != digitalRead(encB)) encoderCount++;
  else encoderCount--;
}

// -------------------------------- //

void setup() {
  // Initialize debug serial port
  Serial.begin(9600);
  while (!Serial && millis() < 5000); // Wait for Serial port to connect (useful on some boards)

  Serial.println("State switching test");
  Serial.print("Connecting to controller on ");
  // Print the name of the serial port being used (if possible)
  #if defined(CONTROLLER_SERIAL) && !defined(RX_PIN) // Check if using HardwareSerial and not SoftwareSerial
    Serial.println("Hardware Serial");
  #elif defined(RX_PIN) // Check if SoftwareSerial pins are defined
    Serial.println("Software Serial");
  #else
    Serial.println("Unknown Serial type");
  #endif

  // -------- motor 3 --------//
  pinMode(INA1, OUTPUT);
  pinMode(INA2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);

  digitalWrite(STBY, HIGH);
  setupMotor3Direction();
  attachInterrupt(digitalPinToInterrupt(encA), readA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encB), readB, CHANGE);

  lastControlTime = millis();

  // ------------------------ //

  // Initialize the communication serial port for the controller
  controller.begin(38400); // Ensure baud rate matches your controller
  delay(100); // Short delay to let the controller initialize after power-up or serial init

  // Reset encoder counts to 0
  controller.ResetEncoders(MOTOR_ADDRESS);
  delay(100); // Give controller time to process reset


  controller.MixedSpeedPosition(MOTOR_ADDRESS,
                                  0, 0,
                                  0, 0,
                                  buffer_mode);
  delay(2000);

  // // Move hip motor by a certain amount to simulate state 1
  // control_flag = (0 << 0) | (1 << 1) | (0 << 2) | (1 << 3) | (1 << 4); // rotating in positive codition, execute immediatly
  // // sending 0 position command to motor 1 (switch plate), rotate motor 2 (hip) forward 10000 counts (a bit more than 2 rotations)
  // Serial.println("moving motor 2 forward");
  // controller.SpeedDistanceM1M2(MOTOR_ADDRESS, 0, 0, motor2_speed, 10000, control_flag);
  // delay(8500);

  // set state to 3 (not moving)
  state = 3;
}

void loop() {
  static unsigned long moveStartTime = 0;
  static bool commandSent = false;

  // encoder reading
  bool v1=false, v2=false;
  int32_t enc1 = controller.ReadEncM1(MOTOR_ADDRESS, nullptr, &v1);
  int32_t enc2 = controller.ReadEncM2(MOTOR_ADDRESS, nullptr, &v2);
  // Serial.print("Enc1: "); Serial.print(enc1);
  // Serial.print(" Enc2: "); Serial.println(enc2);

  if (state == 1) {
    stopMotor3();

    if (!state1Started) {
      state1StartEnc2 = enc2;
      state1Started = true;
      Serial.println("State 1: Motor 2 full cycle start");
    }

    if (labs((long)enc2 - (long)state1StartEnc2) < 7680) {
      controller.SpeedM1M2(MOTOR_ADDRESS, 0, motor2_speed);
    } else {
      controller.SpeedM1M2(MOTOR_ADDRESS, 0, 0);
      state1Started = false;
      state = 5;
      Serial.println("State 1 done -> switching to state 5");
    }
  }
  
  else if (state == 2) {
    // turn off motor 3
    stopMotor3();
    // Send the command only once on entry
    if (!commandSent) {
      // set sole switching status to true
      switchInProgress = true;
      // ial.println("Raising leg");
      // calculate how much motor 2 needs to turn
      // 32_t phase = enc2 % 3840;
      // 32_t motor2_dist = (3840 - phase) % 3840;
      // t8_t motor2_flag = 0 | (1 << 1) | (0 << 3); // bit0=0 (Forward), bit1=1 (Relative), bit3=0 (Immediate)
      // // motor 2 going home
      // controller.SpeedDistanceM2(MOTOR_ADDRESS, motor2_speed, motor2_dist, motor2_flag);
      // //controller.M2SpeedPosition(MOTOR_ADDRESS, (uint32_t)motor2_speed/2, enc2+motor2_dist, 0);
      // delay(5000); // wait for motor 2 to finish
      // controller.SpeedM2(MOTOR_ADDRESS, 0);
      // delay(500);

      // Switching foot
      Serial.println("Switching foot");
      controller.SpeedAccelDistanceM1(MOTOR_ADDRESS, motor1_accel, (uint32_t)motor1_speed, 610, buffer_mode);
      delay(3000);

      moveStartTime = millis();
      commandSent = true;
    }

    // After timer elapses, switch back to state 1
    if (millis() - moveStartTime > 8000) {
      commandSent = false;
      state = 1;
      // set sole switching status to false
      switchInProgress = false;
      currentSole = (currentSole + 1) % 3;
      Serial.println("DONE");
    }
  } 

  else if (state == 4) {
    // turn off motor 3
    stopMotor3();
    // Send the command only once on entry
    if (!commandSent) {
      // set sole switching status to true
      switchInProgress = true;
      // Serial.println("Raising leg");
      // // calculate how much motor 2 needs to turn
      // int32_t phase = enc2 % 3840;
      // int32_t motor2_dist = (3840 - phase) % 3840;
      // uint8_t motor2_flag = 0 | (1 << 1) | (0 << 3); // bit0=0 (Forward), bit1=1 (Relative), bit3=0 (Immediate)
      // // motor 2 going home
      // controller.SpeedDistanceM2(MOTOR_ADDRESS, motor2_speed, motor2_dist, motor2_flag);
      // //controller.M2SpeedPosition(MOTOR_ADDRESS, (uint32_t)motor2_speed/2, enc2+motor2_dist, 0);
      // delay(5000); // wait for motor 2 to finish
      // controller.SpeedM2(MOTOR_ADDRESS, 0);
      // delay(500);

      // Switching foot (240 deg)
      Serial.println("Switching foot");
      controller.SpeedAccelDistanceM1(MOTOR_ADDRESS, motor1_accel, (uint32_t)motor1_speed, 1320, buffer_mode);
      delay(3000);

      moveStartTime = millis();
      commandSent = true;
    }

    // After timer elapses, switch back to state 1
    if (millis() - moveStartTime > 8000) {
      commandSent = false;
      state = 1;
      // set sole switching status to false
      switchInProgress = false;
      Serial.println("DONE");
      currentSole = (currentSole + 2) % 3;
    }
  }
  
  else if (state == 3) {
      stopMotor3();
      controller.SpeedM1M2(MOTOR_ADDRESS, 0, 0);
      delay(500);
  }

  else if (state == 5) {
    controller.SpeedM1M2(MOTOR_ADDRESS, 0, 0);
    setupMotor3Direction();
    updateMotor3Control();
  }

  // Check if a character has arrived over Serial
  if (Serial.available() > 0) {
    char key = Serial.read();

    if (switchInProgress) {
      Serial.println("BUSY");
      return;
    }

    else if (key == 'a') {
      controller.SpeedM1M2(MOTOR_ADDRESS, 0, 0);
      delay(500);
      Serial.print("Switching to state 1");
      state1Started = false;
      commandSent = false; // reset so state 2 works next time
      state = 5;
    } 
    
    else if (key == 'b') {
      controller.SpeedM1M2(MOTOR_ADDRESS, 0, 0);
      delay(500);
      state = 2;
      Serial.print("Switching to state 2");
    } 
    
    else if (key == 'c') {
      controller.SpeedM1M2(MOTOR_ADDRESS, 0, 0);
      delay(500);
      state = 4;
      Serial.print("Switching to state 4");
    } 
    
    else if (key == 'd') {
      controller.SpeedM1M2(MOTOR_ADDRESS, 0, 0);
      delay(500);
      state = 3;
      Serial.print("Switching to state 3");
    }

    else if (key == 's') {
      Serial.print("SOLE=");
      Serial.print(currentSole);
      Serial.print(",BUSY=");
      Serial.println(switchInProgress ? 1 : 0);
    }
    
    else if (key >= '0' && key <= '2') {
      if (state == 1) {
        Serial.println("BUSY: MOTOR 2 RUNNING");
        return;
      }

      int targetSole = key - '0';

      if (targetSole == currentSole) {
        Serial.println("DONE");
        state1Started = false;
        state = 1;
        return;
      }

      // compute shortest forward rotation (only forward allowed)
      int delta = (targetSole - currentSole + 3) % 3;

      if (delta == 1) {
        state = 2;   // 120° move
      } else if (delta == 2) {
        state = 4;   // 240° move
      }

      Serial.print("Switching to sole ");
      Serial.println(targetSole);
    }
  }
}