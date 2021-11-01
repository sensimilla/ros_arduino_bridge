/*********************************************************************
 *  ROSArduinoBridge
 
    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default 
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using 
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

//#define USE_BASE      // Enable the base controller code
#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
   /* The Pololu VNH5019 dual motor driver shield */
   #define POLOLU_VNH5019

   /* The Pololu MC33926 dual motor driver shield */
   //#define POLOLU_MC33926

   /* The RoboGaia encoder shield */
   #define ROBOGAIA
   
   /* Encoders directly attached to Arduino board */
   //#define ARDUINO_ENC_COUNTER

   /* L298 Motor driver*/
   //#define L298_MOTOR_DRIVER
#endif

#define USE_ZUMO  // Enable use of ZUMO Shield hardware

//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS     // Disable use of PWM servos

/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

/* Include servo support if required */
#ifdef USE_SERVOS
   #include <Servo.h>
   #include "servos.h"
#endif

#ifdef USE_BASE
  /* Motor driver function definitions */
  #include "motor_driver.h"

  /* Encoder driver function definitions */
  #include "encoder_driver.h"

  /* PID parameters and functions */
  #include "diff_controller.h"

  /* Run the PID loop at 30 times per second */
  #define PID_RATE           30     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE;
  
  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

/* Include zumo shield support if required */
#ifdef USE_ZUMO
   #include <Wire.h>
   #include <ZumoShield.h>
   #include "zumo_driver.h"  
   /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
   #define AUTO_STOP_INTERVAL 2000
   long lastMotorCommand = AUTO_STOP_INTERVAL;
   // Character array for imu values
   char imu_report[120];
#endif

/* Initialise Zumo hardware if used */
#ifdef USE_ZUMO
  //Wire.begin();
  ZumoBuzzer buzzer;
  Pushbutton button(ZUMO_BUTTON);
  //ZumoMotors motors;
  ZumoIMU imu;

  //constants necessary to convert the acceleromter's raw values into millimiters per second squared 
  #define G_PER_LSB .000061
  #define MSS_PER_G 9.80665

  // magnetometer scaling LSB = least significant bit
  #define MGAUSS_PER_LSB 0.160

  // gyro scaling to degrees / second
  #define DEGREES_PER_LSB 0.00875

  #define ToRad(x) ((x)*0.01745329252)  // *pi/180

  // LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
  // the Pololu LSM303 or LIS3MDL library to find the right values for your board

  #define M_X_MIN -2330
  #define M_Y_MIN -1523
  #define M_Z_MIN -16698
  #define M_X_MAX +2028
  #define M_Y_MAX +3040
  #define M_Z_MAX -10790

  float gx, gy, gz, ax, ay, az, mx, my, mz; 

  // offsets for gyro and accelerometer
  int offset_ax = -5;
  int offset_ay = -83;
  int offset_az = -10;

  int offset_gx = -4;
  int offset_gy = -1;
  int offset_gz = -528;

    // Uncomment the below line to use this axis definition:
    // X axis pointing forward
    // Y axis pointing to the right
    // and Z axis pointing down.
  // Positive pitch : nose up
  // Positive roll : right wing down
  // Positive yaw : clockwise

  int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
#endif

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PING:
    Serial.println(Ping(arg1));
    break;
#ifdef USE_SERVOS
  case SERVO_WRITE:
    servos[arg1].setTargetPosition(arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
    Serial.println(servos[arg1].getServo().read());
    break;
#endif
#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
   case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    }
    else moving = 1;
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;
#endif
#ifdef USE_ZUMO
  case READ_BUTTON:
    Serial.println(button.getSingleDebouncedRelease());
    break;
  case BUZZER_WRITE:
    switch(arg1) {
      case 1:
        // bad sound
        buzzer.play("l8 fefdecdd");
        Serial.println("OK");
        break;
      default:
        // happy sound
        buzzer.play("l8 dbg");
        Serial.println("OK");
        break;
    }
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
    }
    setMotorSpeed(LEFT, arg1);
    setMotorSpeed(RIGHT, arg2);
    Serial.println("OK"); 
    break;
  case READ_IMU:
    imu.read();
    
    // convert gyro readings to radians per second

    gx = imu.g.x + offset_gx;
    gy = imu.g.y + offset_gy;
    gz = imu.g.z + offset_gy;

    // convert to degrees per second
    gx = gx * DEGREES_PER_LSB;
    gy = gy * DEGREES_PER_LSB;
    gz = gz * DEGREES_PER_LSB;

    gx = ToRad(gx);
    gy = ToRad(gy);
    gz = ToRad(gz);
    
    // convert acceleration to meters per second
    ax = imu.a.x + offset_ax;
    ay = imu.a.y + offset_ay;
    az = imu.a.z + offset_az;

    // convert to m/s
    ax = ax * G_PER_LSB * MSS_PER_G;
    ay = ay * G_PER_LSB * MSS_PER_G;
    az = az * G_PER_LSB * MSS_PER_G;

    // apply magnetic snsor calibration
    // adjust for LSM303 compass axis offsets/sensitivity differences by scaling to +/-0.5 range
    mx = (float)(imu.m.x - SENSOR_SIGN[6]*M_X_MIN) / (M_X_MAX - M_X_MIN) - SENSOR_SIGN[6]*0.5;
    my = (float)(imu.m.y - SENSOR_SIGN[7]*M_Y_MIN) / (M_Y_MAX - M_Y_MIN) - SENSOR_SIGN[7]*0.5;
    mz = (float)(imu.m.z - SENSOR_SIGN[8]*M_Z_MIN) / (M_Z_MAX - M_Z_MIN) - SENSOR_SIGN[8]*0.5; 

    // convert to uT microTesla
    mx = imu.m.x * MGAUSS_PER_LSB * 0.1;
    my = imu.m.y * MGAUSS_PER_LSB * 0.1;
    mz = imu.m.z * MGAUSS_PER_LSB * 0.1;

    Serial.print(gx);
    Serial.print(" ");
    Serial.print(gy);
    Serial.print(" ");
    Serial.print(gz);
    Serial.print(" ");
    Serial.print(ax);
    Serial.print(" ");
    Serial.print(ay);
    Serial.print(" ");
    Serial.print(az);
    Serial.print(" ");
    Serial.print(mx);
    Serial.print(" ");
    Serial.print(my);
    Serial.print(" ");
    Serial.println(mz);       
    

    // Serial.print(imu.g.x);
    // Serial.print(" ");
    // Serial.print(imu.g.y);
    // Serial.print(" ");
    // Serial.print(imu.g.z);
    // Serial.print(" ");
    // Serial.print(imu.a.x);
    // Serial.print(" ");
    // Serial.print(imu.g.y);
    // Serial.print(" ");
    // Serial.print(imu.g.z);
    // Serial.print(" ");
    // Serial.print(imu.m.x);
    // Serial.print(" ");
    // Serial.print(imu.m.y);
    // Serial.print(" ");
    // Serial.println(imu.m.z);    
    break;
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);

// Initialize the motor controller if used */
#ifdef USE_BASE
  #ifdef ARDUINO_ENC_COUNTER
    //set as inputs
    DDRD &= ~(1<<LEFT_ENC_PIN_A);
    DDRD &= ~(1<<LEFT_ENC_PIN_B);
    DDRC &= ~(1<<RIGHT_ENC_PIN_A);
    DDRC &= ~(1<<RIGHT_ENC_PIN_B);
    
    //enable pull up resistors
    PORTD |= (1<<LEFT_ENC_PIN_A);
    PORTD |= (1<<LEFT_ENC_PIN_B);
    PORTC |= (1<<RIGHT_ENC_PIN_A);
    PORTC |= (1<<RIGHT_ENC_PIN_B);
    
    // tell pin change mask to listen to left encoder pins
    PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
    // tell pin change mask to listen to right encoder pins
    PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);
    
    // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
    PCICR |= (1 << PCIE1) | (1 << PCIE2);
  #endif
  initMotorController();
  resetPID();
#endif

/* Attach servos if used */
  #ifdef USE_SERVOS
    int i;
    for (i = 0; i < N_SERVOS; i++) {
      servos[i].initServo(
          servoPins[i],
          stepDelay[i],
          servoInitPosition[i]);
    }
  #endif
  #ifdef USE_ZUMO
    Wire.begin();
    if (!imu.init()) {
        // Failed to detect the compass.
        while(1) {
         Serial.println(F("Failed to initialize IMU sensors."));
         delay(100);
         }
      }
    imu.enableDefault();
  #endif
}
/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  
// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    setMotorSpeeds(0, 0);
    moving = 0;
  }
#endif

// Sweep servos
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
#endif

#ifdef USE_ZUMO
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0);
    //moving = 0;
  }
#endif  
}