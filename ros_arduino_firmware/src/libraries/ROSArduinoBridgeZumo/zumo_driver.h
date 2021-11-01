
  /* Create the motor driver object */
  ZumoMotors drive;
  
  /* Wrap the motor driver initialization */
  void initMotorController() {
    //drive.init();
  }

  /* Wrap the drive motor set speed function */
  void setMotorSpeed(int i, int spd) {
    if (i == LEFT) drive.setLeftSpeed(spd);
    else drive.setRightSpeed(spd);
  }

  // A convenience function for setting both motor speeds
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }