
// This example shows how to make a Balboa balance on its two
// wheels and drive around while balancing.
//
// To run this demo, you will need to install the LSM6 library:
//
// https://github.com/pololu/lsm6-arduino
//
// To use this demo, place the robot on the ground with the
// circuit board facing up, and then turn it on.  Be careful to
// not move the robot for a few seconds after powering it on,
// because that is when the gyro is calibrated.  During the gyro
// calibration, the red LED is lit.  After the red LED turns off,
// turn the robot so that it is standing up.  It will detect that
// you have turned it and start balancing.
//
// Alternatively, you can press the A button while the robot is
// lying down and it will try to use its motors to kick up into
// the balancing position.
//
// This demo is tuned for the 50:1 high-power gearmotor with
// carbon brushes, 45:21 plastic gears, and 80mm wheels; you will
// need to adjust the parameters in Balance.h for your robot.
//
// After you have gotten the robot balance well, you can
// uncomment some lines in loop() to make it drive around and
// play a song.

#include <Balboa32U4.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include "Balance.h"

#define NEO_PIN 1
#define NUM_LEDS 8

LSM6 imu;
LIS3MDL mag;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
Balboa32U4Buzzer buzzer;
Balboa32U4ButtonA buttonA;
Balboa32U4ButtonB buttonB;
uint16_t lastSent = millis();
bool musicOn = false;
bool controlled = true;
bool driving = false;
uint32_t lastCommand = millis();
uint32_t lastMagRead = millis();
int16_t lastEncoderLeft = 0, lastEncoderRight = 0;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, NEO_PIN, NEO_RGBW + NEO_KHZ800);

String commandString = "";
char imuValues[60], magValues[40], odomValues[40], debugTelemetry[40];
void initMag() {
  if (!mag.init())
  {
    Serial.println("E: Failed to detect and initialize magnetometer!");
    while (1);
  }

  mag.enableDefault();
}

void setup()
{
  // Uncomment these lines if your motors are reversed.
  // motors.flipLeftMotor(true);
  // motors.flipRightMotor(true);
  //motors.allowTurbo(true);
  ledYellow(0);
  ledRed(1);
  balanceSetup();
  initMag();
  ledRed(0);
  Serial.begin(115200);
  strip.begin();
  strip.show();
}

const char song[] PROGMEM =
  "!O6 T240"
  "l32ab-b>cl8r br b-bb-a a-r gr g-4 g4"
  "a-r gr g-gg-f er e-r d4 e-4"
  "gr msd8d8ml d-4d4"
  "l32efg-gl8r msd8d8ml d-4d4"
  "<bcd-d e-efg- ga-ab- a4 gr";

void playSong()
{
  if (!buzzer.isPlaying())
  {
    buzzer.playFromProgramSpace(song);
  }
}

int8_t setAngularVelocity(float angularVelocity) {
  int8_t wheelSpeed = angularVelocity * GEAR_RATIO * 12 * 4 / 6 / 1000 / 20;
  return wheelSpeed;
}

int8_t setVelocity(float velocity) {
  int8_t wheelSpeed = velocity * GEAR_RATIO * 12 / 4 / 1000 * 7 / 5;
  return wheelSpeed;
}

void driveAround()
{
  uint16_t time = millis() % 8192;
  uint16_t leftSpeed, rightSpeed;
  if (time < 1900)
  {
    leftSpeed = 40;
    rightSpeed = 40;
  }
  else if (time < 4096)
  {
    leftSpeed = 45;
    rightSpeed = 25;
  }
  else if (time < 4096 + 1900)
  {
    leftSpeed = 40;
    rightSpeed = 40;
  }
  else
  {
    leftSpeed = 25;
    rightSpeed = 45;
  }

  balanceDrive(leftSpeed/8, rightSpeed/8);
}

void standUp()
{
  int standupvalue = 1;
  motors.setSpeeds(0, 0);
  buzzer.play("!>grms>a16>a16>g2");
  ledGreen(1);
  ledRed(1);
  ledYellow(1);
  int distanceDiff = 0;
  while (buzzer.isPlaying());

  if (angle > 70000) {
    standupvalue = 1;
  } else if (angle < -70000) {
    standupvalue = -1;
  }
  
  for(int speed = 0; speed < MOTOR_SPEED_LIMIT - 15; speed++){
      integrateEncoders();
      distanceDiff = distanceLeft - distanceRight;    
      motors.setSpeeds(standupvalue * -speed - 1 * distanceDiff, standupvalue * -speed + 1 * distanceDiff);
      delay(3);
    }
    
    for(int i = 0; i < 50; i++) {
      integrateEncoders();
      distanceDiff = distanceLeft - distanceRight;    
      motors.setSpeeds(standupvalue * -MOTOR_SPEED_LIMIT + 15 - 10 * distanceDiff, standupvalue * -MOTOR_SPEED_LIMIT + 15 + 10 * distanceDiff);
      delay(1);
    }

    motors.setSpeeds(standupvalue * MOTOR_SPEED_LIMIT, standupvalue * MOTOR_SPEED_LIMIT);
    

  
  for (uint8_t i = 0; i < 30; i++)
  {
    delay(UPDATE_TIME_MS);
    balanceUpdateSensors();
    if(angle < 50000 and angle > -50000)
    {
      Serial.println("D: Start balancing!");
      break;
    }
  }
  motorSpeed = MOTOR_SPEED_LIMIT; //150;
  balanceResetEncoders();
}

void layDown() {
  if(isBalancing()) {
    motors.setSpeeds(-100, -100);
    delay(300);
    motors.setSpeeds(0, 0);
    delay(2000);
  }
  return;
}

void executeCommand() {
  Serial.print("D: command received: ");
  Serial.println(commandString);
  if (commandString[0] == 'S') {
    if(angle > 50000) {
      Serial.println("D: standUp");
      standUp();
    }
  } else if (commandString[0] == 'L') {
    Serial.println("D: layDown");
    layDown();
  } else if (commandString[0] == 'C') {
    String colorStrings[3];
    int j=0;
    for (int i = 2; i<commandString.length(); i++) {
      char sign = commandString[i];
      if (sign == '\n') {
        break;
      } else if (sign == ' ') {
        j++;
      } else if (isDigit(sign)) {
        colorStrings[j] += sign;
      } else {
        Serial.println("E: Error in command parsing. Sign unknown.");
        return;
      }
      if (j > 2) {
        Serial.println("E: Error in command parsing. Too many spaces.");
        return;
      }
    }
    if (j < 2) {
      Serial.println("E: Error in command parsing. Too few values.");
      return;
    }
    int rgb[3];
    for (int i = 0; i<3; i++) {
      rgb[i] = colorStrings[i].toInt();
    }
    for (int i = 0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, rgb[0], rgb[1], rgb[2]);
    }
    strip.show();
    
  } else if (commandString[0] == 'V') {
    String velocityStrings[2];
    int j=0;
    for (int i = 2; i<commandString.length(); i++) {
      char sign = commandString[i];
      if (sign == '\n') {
        break;
      } else if (sign == ' ') {
        j++;
      } else if (isDigit(sign) || (sign == '-' && velocityStrings[j].length() == 0)) {
        velocityStrings[j] += sign;
      } else {
        Serial.println("E: Error in command parsing. Sign unknown.");
        return;
      }
      if (j > 1) {
        Serial.println("E: Error in command parsing. Too many spaces.");
        return;
      }
    }
    if (j < 1) {
      Serial.println("E: Error in command parsing. Too few values.");
      return;
    }
    int velocities[2];
    for (int i = 0; i<2; i++) {
      velocities[i] = velocityStrings[i].toInt();
    }
    int16_t speedL, speedR, angularVelocity, velocity;
    angularVelocity = velocities[0];
    velocity = velocities[1];
    speedL = -setAngularVelocity(angularVelocity) - setVelocity(velocity);
    speedR = setAngularVelocity(angularVelocity) - setVelocity(velocity);
    driving = true;

    balanceDrive(speedL, speedR);
    lastCommand = millis();
  } else {
    Serial.println("E: Command not known.");
  }
}

void stop() {
  balanceDrive(0, 0);
}

char readSerial() {
  while (Serial.available() > 0) {
    char inSign = Serial.read();
    commandString += inSign;
    if (inSign == '\n') {
      executeCommand();
      commandString = "";
    }
  }
}

void publishIMU() {
  snprintf(imuValues, sizeof(imuValues), "I: %6d %6d %6d %6d %6d %6d",
    imu.a.x, imu.a.y, imu.a.z, imu.g.x, imu.g.y, imu.g.z);
  Serial.println(imuValues);

  snprintf(magValues, sizeof(magValues), "M: %6d %6d %6d",
    mag.m.x, mag.m.y, mag.m.z);
  Serial.println(magValues);
}

void publishOdometry() {
  int16_t encoderLeft = encoders.getCountsLeft();
  int16_t encoderRight = encoders.getCountsRight();
  snprintf(odomValues, sizeof(odomValues), "O: %6d %6d %6d %6d",
    encoderLeft - lastEncoderLeft, encoderRight - lastEncoderRight,
    encoderLeft, encoderRight);
  Serial.println(odomValues);
  lastEncoderLeft = encoderLeft;
  lastEncoderRight = encoderRight;
}

void publishTelemetry() {
  publishIMU();
  publishOdometry();
}

void loop()
{
  balanceUpdate();
  uint32_t timeNow = millis();
  
  if (timeNow - lastSent > 20) {
    if(timeNow - lastMagRead > 100) {
      mag.read();
      lastMagRead = timeNow;
    }
    //publishTelemetry();
    lastSent = timeNow;
  }
  
  if(controlled) {
    int8_t speedL, speedR, angularVelocity, velocity;
    
    angularVelocity = 0;
    velocity = 0;
    readSerial();
    timeNow = millis();
    if (timeNow - lastCommand > 1000 && driving) {
      driving = false;
      balanceResetEncoders();
      stop();
    }
  }
  if (isBalancing() && musicOn) {
    // Once you have it balancing well, uncomment these lines for
    // something fun.

    playSong();
    driveAround();
  } else {
    buzzer.stopPlaying();

    if (buttonA.getSingleDebouncedPress())
    {
      musicOn = false;
      balanceDrive(0, 0);
      standUp();
    }
    if (buttonB.getSingleDebouncedPress())
    {
      musicOn = true;
      standUp();
    }
  }

  // Illuminate the red LED if the last full update was too slow.
  ledRed(balanceUpdateDelayed());

  // Display feedback on the yellow and green LEDs depending on
  // the variable fallingAngleOffset.  This variable is similar
  // to the risingAngleOffset used in Balance.cpp.
  //
  // When the robot is rising toward vertical (not falling),
  // angleRate and angle have opposite signs, so this variable
  // will just be positive or negative depending on which side of
  // vertical it is on.
  //
  // When the robot is falling, the variable measures how far off
  // it is from a trajectory starting it almost perfectly
  // balanced then falling to one side or the other with the
  // motors off.
  //
  // Since this depends on ANGLE_RATE_RATIO, it is useful for
  // calibration.  If you have changed the wheels or added weight
  // to your robot, you can try checking these items, with the
  // motor power OFF (powered by USB):
  //
  // 1. Try letting the robot fall with the Balboa 32U4 PCB up.
  //    The green LED should remain lit the entire time.  If it
  //    sometimes shows yellow instead of green, reduce
  //    ANGLE_RATE_RATIO.
  //
  // 2. If it is tilted beyond vertical and given a push back to
  //    the PCB-up side again, the yellow LED should remain lit
  //    until it hits the ground.  If you see green, increase
  //    ANGLE_RATE_RATIO.
  //
  // In practice, it is hard to achieve both 1 and 2 perfectly,
  // but if you can get close, your constant will probably be
  // good enough for balancing.
  int32_t fallingAngleOffset = angleRate * ANGLE_RATE_RATIO - angle;
  if (fallingAngleOffset > 0)
  {
    ledYellow(1);
    ledGreen(0);
  }
  else
  {
    ledYellow(0);
    ledGreen(1);
  }
}
