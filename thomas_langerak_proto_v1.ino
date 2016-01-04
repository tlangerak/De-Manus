/*VERSION: 0.1b   
 * tested succesful on:
 * -  Arduino Uno rev 3
*/

/**-----------------------------------------------------------------------------------------------------------**/
/**                                                LICENSE                                                    **/
/**-----------------------------------------------------------------------------------------------------------**/
/**-----------------------------------------------------------------------------------------------------------**/
/** "THE BEER-WARE LICENSE" (Revision 42):                                                                    **/
/** Pepijn Verburg (www.pepijnverburg.nl) & Thomas Langerak(www.thomaslangerak.nl) wrote this file.           **/ 
/** As long as you retain this notice you can do whatever you want with this stuff.                           **/
/**  If we meet some day, and you thinkthis stuff is worth it, you can buy me a beer in return.               **/
/** I hope it is useful,                                                                                      **/
/** Thomas Langerak                                                                                           **/
/**-----------------------------------------------------------------------------------------------------------**/

/**-----------------------------------------------------------------------------------------------------------**/
/**                                                INTRODUCTION                                               **/
/**-----------------------------------------------------------------------------------------------------------**/
/**-----------------------------------------------------------------------------------------------------------**/
/** This code is hugely based on a code written by Pepijn Verburg for an assignment of his. I am extremely    **/
/** grateful that I was allowed to make use of his previous work. This piece of code is partially adapted by  **/
/** him and partially by me to fit my device better and to overall improve the quality. The result of this is **/
/** a less generic code, yet with some useful adaptions.                                                      **/
/** ADAPTIONS (among others):                                                                                 **/
/** -  No neopixel support anymore                                                                            **/
/** -  Added support for capsense                                                                             **/
/** -  Animations can now be keyframed to play after eachother                                                **/
/** -  Some specific animations added for De Manus                                                            **/
/** -  Apply behaviour now happens with in sensor reading instead of the loop                                 **/
/** -  Improved Easing support                                                                                **/
/** WISHED ADAPTIONS:                                                                                         **/
/** - Better memory usage --> longer animation ques, again adding neopixel support                            **/
/** - a "pause" function in the animation que                                                                 **/
/** - support for more and different sensors                                                                  **/
/** - making it a library in the far future                                                                   **/
/** - Auto Calibration sensors                                                                                **/
/** - A Processing interface                                                                                  **/
/**-----------------------------------------------------------------------------------------------------------**/

/**-----------------------------------------------------------------------------------------------------------**/
/**                                                ABOUT MY PROJECT                                           **/
/**-----------------------------------------------------------------------------------------------------------**/
/**-----------------------------------------------------------------------------------------------------------**/
/** This code is used for my Final Bachelor Project DE MÀNUS at the Eindhoven Univeristy of Technology        **/
/** Industrial Design program. It is adapted to its current form in December 2015.                            **/
/** This project is made for and within the Shape Changing and Reciprocal Interfaces squad                    **/
/**                                                                                                           **/
/**“The hand is a critical interface to the world, allowing the use of tools, the intimate sense of touch,    **/
/**and a vast range of communicative gestures.”                                                               **/
/**              - Golan Levin                                                                                **/
/**                                                                                                           **/
/**How we express ourselves with our hands probably says more about our emotional state                       **/
/**than the use of our words. Yet this expression is not there when interacting with objects.                 **/
/**What if we can add expressions to our interactions?                                                        **/
/**What if our daily objects act on our emotions and not our actions?                                         **/
/**Or to take it further: what if our designs emotionally respond to an expression of our emotions?           **/
/**By  researching circular and linear interactions I try to formulate answers to these questions.            **/
/**-----------------------------------------------------------------------------------------------------------**/

/**-----------------------------------------------------------------------------------------------------------**/
/**                                                 LIBRARIES                                                 **/
/**-----------------------------------------------------------------------------------------------------------**/

// servo library
// SOURCE: native
#include <Servo.h>
#include <Wire.h>

// cap library
//SOURCE: Adafruit, downloaded via to Arduino Library Manager
#include "Adafruit_MPR121.h"

// easing library
// SOURCE: http://portfolio.tobiastoft.dk/Easing-library-for-Arduino
// downloaded using the web.archive.org, as download is not available anymore
// CHEATSHEET: http://easings.net/nl
#include <Easing.h>

/**-----------------------------------------------------------------------------------------------------------**/
/**                                                 VARIABLES                                                 **/
/**-----------------------------------------------------------------------------------------------------------**/

/**
   Global constants
*/
#define MAX_DIGITAL_PINS 4 // amount of digital pins
#define MAX_ANALOG_PINS 0 // amount of analog pins
#define MODULE_AMOUNT 2 // only one
#define MODULE_MOTOR_AMOUNT 1 // only two

/**
   Motor constants
*/
#define START_DEGREES 15 // initialize value of all servo motors
#define MIN_DEGREES 0 // minimum amount of degrees
#define MAX_DEGREES 170 // maximum amount of degrees
#define AVERAGE_MOTOR_SPEED 70 // an abitrary average speed
#define MIN_MOTOR_SPEED 0 // minimum speed of the motor in degrees per second
#define MAX_MOTOR_SPEED 500 // maximum speed of the motor in degrees per second
#define SLOW_MOTOR_SPEED 30 // the minimum motor speed that is registrable by the accelerometer


/**
   Animation constants
*/
#define MAX_ANIMATIONS 8 // amount of animations in the queue per pin
#define MOTOR_ANIMATION_TYPE 1 // type of a motor animation
#define LIGHT_ANIMATION_TYPE 2 // type of a light animation 
#define ANIMATION_START_TIME_INDEX 0 // index in the queue array of the start time
#define ANIMATION_START_STATE_INDEX 1 // index in the queue array of the start value
#define ANIMATION_TARGET_STATE_INDEX 2 // index in the queue array of the target value
#define ANIMATION_DURATION_INDEX 3 // index in the queue array of the duration
#define ANIMATION_TYPE_INDEX 4 // index in the queue array of the type
#define ANIMATION_EASING_INDEX 5 // index in the queue array of the easing
#define EASING_S 1.70158 * 2 // additional parameter needed for some easings
#define EASING_DEFAULT 1 // standard easing
#define EASEINSINE 1 //give number to easing to make refering to easing more easy. 
#define EASEOUTSINE 2
#define EASEINOUTSINE 3
#define EASEINQUAD 4
#define EASEOUTQUAD 5
#define EASEINOUTQUAD 6
#define EASEINCUBIC 7
#define EASEOUTCUBIC 8
#define EASEINOUTCUBIC 9
#define EASEINQUART 10
#define EASEOUTQUART 11
#define EASEINOURQUART 12
#define EASEINQUINT 13
#define EASEOUTQUINT 14
#define EASEINOUTQUINT 15
#define EASEINEXPO 16
#define EASEOUTEXPO 17
#define EASEINOUREXPO 18
#define EASEINCIRC 19
#define EASEOUT CIRC 20
#define EASEINOUTCIRC 21
#define EASEINBACK 22
#define EASEOUTBACK 23
#define EASEINOUTBACK 24
#define EASEINELASTIC 25
#define EASEOUTELASTIC 26
#define EASEINOUTELASTIC 27
#define EASEINBOUNCE 28
#define EASEOUTBOUNCE 29
#define EASEINOUTBOUNCE 30
/**
   Timer constants
*/
#define TIMER_LAST_TIME_INDEX 0 // index where the last time is stored
#define TIMER_DELAY_INDEX 1 // index of the duration we need to wait before passed

#define SENSOR_TIMER_ID 0 // id of the sensor timer
#define SENSOR_TIMER_DELAY 50 // delay of the sensor read
#define ANIMATION_TIMER_ID 1 // id of the animation timer
#define ANIMATION_TIMER_DELAY 2 // delay of the animation read
#define BEHAVIOUR_TIMER_ID 2 // id of the behaviour timer
#define BEHAVIOUR_TIMER_DELAY 100 // delay of the behaviour read

/**
   Array with specific timers
   NOTE: keep the first index update to the amount of timers!
   Now an id of 9 is the maximum
*/

long timerArr[10][2];

/**
   Settings
*/
float motorSpeedMultiplier = 1.0f;
float lightSpeedMultiplier = 1.0f;

/**
   Motor pin allocation
*/
int motorPinArr[MODULE_AMOUNT][MODULE_MOTOR_AMOUNT] = {
  { 2 }, // first motor
  { 3 }, // second motor
};

/**
   Light pin variables
*/

/**
   Servo motor instances
*/
Servo motorArr[MAX_DIGITAL_PINS];
/**
   Animation queue of the motor and the lights
*/
long animationQueueArr[MAX_DIGITAL_PINS][MAX_ANIMATIONS][6];

/**
   Current states of attached devices
*/
int stateArr[MAX_DIGITAL_PINS] = {};

/**
   Capsense variables
*/


Adafruit_MPR121 cap = Adafruit_MPR121();

// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouched = 0;
uint16_t currtouched = 0;

boolean touchStates[1]; //to keep track of the previous touch states
boolean touch;

/**-----------------------------------------------------------------------------------------------------------**/
/**                                                 INITIALIZE                                                **/
/**-----------------------------------------------------------------------------------------------------------**/

/**
   Setup
*/
void setup() {

  // initialize Serial
  Serial.begin(9600);
  // initialize the timers
  initializeTimers();

  // initialize the motors
  initializeMotors();

  // initialize the sensors
  initializeSensors();
  Serial.println("Setup Complete");
}
/**
   Initialize the timers
*/
void initializeTimers() {

  // populate sensor timer
  timerArr[SENSOR_TIMER_ID][TIMER_LAST_TIME_INDEX] = 0;
  timerArr[SENSOR_TIMER_ID][TIMER_DELAY_INDEX] = SENSOR_TIMER_DELAY;

  // populate animation timer
  timerArr[ANIMATION_TIMER_ID][TIMER_LAST_TIME_INDEX] = 0;
  timerArr[ANIMATION_TIMER_ID][TIMER_DELAY_INDEX] = ANIMATION_TIMER_DELAY;

  // populate behaviour timer
  timerArr[BEHAVIOUR_TIMER_ID][TIMER_LAST_TIME_INDEX] = 0;
  timerArr[BEHAVIOUR_TIMER_ID][TIMER_DELAY_INDEX] = BEHAVIOUR_TIMER_DELAY;
}

/**
   Initialize the motors
*/
void initializeMotors() {

  // initialize all the modules
  for (int moduleId = 0; moduleId < MODULE_AMOUNT; moduleId++) {

    // loop all the motors
    for (int motorId = 0; motorId < MODULE_MOTOR_AMOUNT; motorId++) {

      // variables
      int motorPin = getMotorPin(moduleId, motorId);
      Servo motor = getMotor(motorPin);
      int initialDegrees = START_DEGREES;

      // guard: check the motor pin
      if (motorPin < 0) {
        continue;
      }

      // set pin mode
      pinMode(motorPin, OUTPUT);
      // attach the motor pin

      motor.attach(motorPin);
      // calibrate all motors to initial value
      writeMotorLocation(motorPin, initialDegrees);
    }
  }
  Serial.println("Calibrating motor 1..."); //calibrating aligns potmeter with servo.
  calibrate(getMotor(getMotorPin(0, 0)), A0, 90, 180); //calibrate linear motor
  Serial.println("Calibrating motor 2...");
  calibrate(getMotor(getMotorPin(1, 0)), A1, 20, 150); //calibarate rotary motor
}

/**
   calibrate motors
*/

int linMinDegrees;
int linMaxDegrees;
int linMinFeedback;
int linMaxFeedback;
int rotMinDegrees;
int rotMaxDegrees;
int rotMinFeedback;
int rotMaxFeedback;


void calibrate(Servo servo, int analogPin, int minPos, int maxPos)
{
  if (analogPin == A0) { // Move to the minimum position and record the feedback value
    servo.write(minPos);
    linMinDegrees = minPos;
    delay(2000); // make sure it has time to get there and settle
    linMinFeedback = analogRead(analogPin);

    // Move to the maximum position and record the feedback value
    servo.write(maxPos);
    linMaxDegrees = maxPos;
    delay(2000); // make sure it has time to get there and settle
    linMaxFeedback = analogRead(analogPin);
  }
  if (analogPin == A1) { // Move to the minimum position and record the feedback value
    servo.write(minPos);
    rotMinDegrees = minPos;
    delay(2000); // make sure it has time to get there and settle
    rotMinFeedback = analogRead(analogPin);

    // Move to the maximum position and record the feedback value
    servo.write(maxPos);
    rotMaxDegrees = maxPos;
    delay(2000); // make sure it has time to get there and settle
    rotMaxFeedback = analogRead(analogPin);
  }
}

/**
   Setting up all sensors
*/
void initializeSensors() {
  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121 found!");
}


/**-----------------------------------------------------------------------------------------------------------**/
/**                                                 PROCESSES                                                 **/
/**-----------------------------------------------------------------------------------------------------------**/

/**
   Loop
*/
void loop() {
  // handle the animation queue
  handleAnimations();

  // handle the sensors
  handleSensors();
}


/**
   Handle all the requested animations
*/
void handleAnimations() {

  // check the sensor read delay
  if (!hasPassedTimer(ANIMATION_TIMER_ID)) {
    return;
  }

  // update the timer
  updateTimer(ANIMATION_TIMER_ID);

  // loop all the available pins
  for (int pinId = 0; pinId < MAX_DIGITAL_PINS; pinId++) {

    // loop all the animation frames
    for (int frameIndex = 0; frameIndex < MAX_ANIMATIONS; frameIndex ++) {

      // guard: check if an animation is requested
      // we can see this through a defined start time
      if (!hasAnimationFrame(pinId, frameIndex)) {
        continue;
      }

      // variables
      long currentTime = millis();
      long startTime = animationQueueArr[pinId][frameIndex][ANIMATION_START_TIME_INDEX];
      float currentState = getState(pinId);
      float startState = animationQueueArr[pinId][frameIndex][ANIMATION_START_STATE_INDEX];
      float targetState = animationQueueArr[pinId][frameIndex][ANIMATION_TARGET_STATE_INDEX];
      float duration = animationQueueArr[pinId][frameIndex][ANIMATION_DURATION_INDEX];
      int type = animationQueueArr[pinId][frameIndex][ANIMATION_TYPE_INDEX];
      int easing = animationQueueArr[pinId][frameIndex][ANIMATION_EASING_INDEX];
      long passedTime = currentTime - startTime;

      // guard: check if we need to wait for this animation to start
      if (passedTime <= 0) {
        continue;
      }

      // guard: check if we need to remove this from the queue
      if (passedTime >= duration) {

        // disable the animation on this pin
        resetAnimationFrame(pinId, frameIndex);
        continue;
      }

      // calculate the state the device should be in
      int deltaState = targetState - startState;
      int newState = currentState + deltaState;

      // check for a specific state to determine the easing
      if (easing == 1) {
        newState = Easing::easeInSine(passedTime, startState, deltaState, duration);
      } else if (easing == 2) {
        newState = Easing::easeOutBounce(passedTime, startState, deltaState, duration);
      } else if (easing == 3) {
        newState = Easing::easeInOutBounce(passedTime, startState, deltaState, duration);
      } else if (easing == 4) {
        newState = Easing::easeInQuad(passedTime, startState, deltaState, duration);
      } else if (easing == 5) {
        newState = Easing::easeOutQuad(passedTime, startState, deltaState, duration);
      } else if (easing == 6) {
        newState = Easing::easeInOutQuad(passedTime, startState, deltaState, duration);
      } else if (easing == 7) {
        newState = Easing::easeInCubic(passedTime, startState, deltaState, duration);
      } else if (easing == 8) {
        newState = Easing::easeOutCubic(passedTime, startState, deltaState, duration);
      } else if (easing == 9) {
        newState = Easing::easeInOutCubic(passedTime, startState, deltaState, duration);
      } else if (easing == 10) {
        newState = Easing::easeInQuart(passedTime, startState, deltaState, duration);
      } else if (easing == 11) {
        newState = Easing::easeOutQuart(passedTime, startState, deltaState, duration);
      } else if (easing == 12) {
        //        newState = Easing::easeOutInOutQuart(passedTime, startState, deltaState, duration, EASING_S);
      } else if (easing == 13) {
        newState = Easing::easeInQuint(passedTime, startState, deltaState, duration);
      } else if (easing == 14) {
        newState = Easing::easeOutQuint(passedTime, startState, deltaState, duration);
      } else if (easing == 15) {
        newState = Easing::easeInOutQuint(passedTime, startState, deltaState, duration);
      } else if (easing == 16) {
        newState = Easing::easeInExpo(passedTime, startState, deltaState, duration);
      } else if (easing == 17) {
        newState = Easing::easeOutExpo(passedTime, startState, deltaState, duration);
      } else if (easing == 18) {
        newState = Easing::easeInOutExpo(passedTime, startState, deltaState, duration);
      } else if (easing == 19) {
        newState = Easing::easeInCirc(passedTime, startState, deltaState, duration);
      } else if (easing == 20) {
        newState = Easing::easeOutCirc(passedTime, startState, deltaState, duration);
      } else if (easing == 21) {
        newState = Easing::easeInOutCirc(passedTime, startState, deltaState, duration);
      } else if (easing == 22) {
        newState = Easing::easeInBack(passedTime, startState, deltaState, duration, EASING_S);
      } else if (easing == 23) {
        newState = Easing::easeOutBack(passedTime, startState, deltaState, duration, EASING_S);
      } else if (easing == 24) {
        newState = Easing::easeInOutBack(passedTime, startState, deltaState, duration, EASING_S);
      } else if (easing == 25) {
        newState = Easing::easeInElastic(passedTime, startState, deltaState, duration, EASING_S);
      } else if (easing == 26) {
        newState = Easing::easeOutElastic(passedTime, startState, deltaState, duration, EASING_S);
      } else if (easing == 27) {
        newState = Easing::easeInOutElastic(passedTime, startState, deltaState, duration, EASING_S);
      } else if (easing == 28) {
        newState = Easing::easeInBounce(passedTime, startState, deltaState, duration);
      } else if (easing == 29) {
        newState = Easing::easeOutBounce(passedTime, startState, deltaState, duration);
      } else if (easing == 30) {
        newState = Easing::easeInOutBounce(passedTime, startState, deltaState, duration);
      } else {
        newState = Easing::easeInOutSine(passedTime, startState, deltaState, duration);
      }

      newState = validateState(newState, pinId, type);

      // debug turn off when not needed! makes code slow as hell.
      //      Serial.print("I:");
      //      Serial.println(frameIndex);
      //      Serial.print("S:");
      //      Serial.println(startState);
      //      Serial.print("T:");
      //      Serial.println(targetState);
      //      Serial.print("C:");
      //      Serial.println(currentState);
      //      Serial.print("N:");
      //      Serial.println(newState);
      //      Serial.print("Type:");
      //      Serial.println(type);
      //      Serial.println();

      // check for the type, this is still from when light was included.

      if (type == MOTOR_ANIMATION_TYPE) {
        // write the new motor state
        writeMotorLocation(pinId, newState);
      } 
    } // end of frame loop
  } // end of pin loop
}

/**
   Handle all the sensory input
*/
void handleSensors() {
  // check the sensor read delay
  if (!hasPassedTimer(SENSOR_TIMER_ID)) {
    return;
  }

  // update the timer
  updateTimer(SENSOR_TIMER_ID);

  // read the capsense.
  readTouchInputs();
}

/**
   Apply the behaviour to the device. r
*/

void applyBehaviour(int beh) {
  // check the behaviour apply delay
  if (!hasPassedTimer(BEHAVIOUR_TIMER_ID)) {
    return;
  }

  // update the timer
  updateTimer(BEHAVIOUR_TIMER_ID);

  // get the motor pin
  int motorPin = getMotorPin(0, 0);

  // guard: check if we are already animating
  // we will always wait on the animation to end
  if (isAnimating(motorPin)) {
    return;
  }

  if (beh == 1) {
    curious();
  } else if (beh == 2) {
    wijzeoudeman();
  }   else if (beh == 2) {
    wijzeoudeman();
  }  else if (beh == 2) {
    wijzeoudeman();
  } else if (beh == 2) {
    wijzeoudeman();
  }
}


/**-----------------------------------------------------------------------------------------------------------**/
/**                                                  BEHAVIOURS                                               **/
/**-----------------------------------------------------------------------------------------------------------**/


void wijzeoudeman() {
  int spd = 25;
  int lin = getMotorPin(0, 0);
  int rot = getMotorPin(1, 0);
  Serial.println("Wijzeoudeman");
  requestAnimationByTarget(rot, 100, spd, MOTOR_ANIMATION_TYPE, EASEINSINE);
  requestAnimationByTarget(rot, 50, spd, MOTOR_ANIMATION_TYPE, EASEINSINE);
  requestAnimationByTarget(rot, 100, spd, MOTOR_ANIMATION_TYPE, EASEINSINE);
  requestAnimationByTarget(rot, 25, spd, MOTOR_ANIMATION_TYPE, EASEINSINE);
  requestAnimationByTarget(lin, 0, spd, MOTOR_ANIMATION_TYPE, EASEINSINE);
  requestAnimationByTarget(lin, 75, spd, MOTOR_ANIMATION_TYPE, EASEINSINE);
  requestAnimationByTarget(lin, 25, spd, MOTOR_ANIMATION_TYPE, EASEINSINE);
  requestAnimationByTarget(lin, 100, spd, MOTOR_ANIMATION_TYPE, EASEINSINE);
}

void curious() {
  int spd = 100;
  int lin = getMotorPin(0, 0);
  int rot = getMotorPin(1, 0);
  Serial.println("Curious");
  requestAnimationByTarget(lin, 180, spd, MOTOR_ANIMATION_TYPE, EASEOUTSINE);
  requestAnimationByTarget(lin, 120, spd, MOTOR_ANIMATION_TYPE, EASEOUTSINE);
  requestAnimationByTarget(lin, 150, spd, MOTOR_ANIMATION_TYPE, EASEOUTSINE);
  requestAnimationByTarget(lin, 90, spd, MOTOR_ANIMATION_TYPE, EASEOUTSINE);
  requestAnimationByTarget(rot, 180, spd, MOTOR_ANIMATION_TYPE, EASEOUTSINE);
  requestAnimationByTarget(rot, 120, spd, MOTOR_ANIMATION_TYPE, EASEOUTSINE);
  requestAnimationByTarget(rot, 180, spd, MOTOR_ANIMATION_TYPE, EASEOUTSINE);
  requestAnimationByTarget(rot, 90, spd, MOTOR_ANIMATION_TYPE, EASEOUTSINE);
}

/**-----------------------------------------------------------------------------------------------------------**/
/**                                                  OUTPUT                                                   **/
/**-----------------------------------------------------------------------------------------------------------**/

/**
   Request an animation
   Speed is in value per second
*/

void requestAnimationByTarget(int pinId, int targetState, int speed, int type, int easing) {

  // calculate the speed to a duration
  int newAnimationIndex = -1;
  long startTime = millis();
  long startState = getState(pinId);

  // find a new index
  for (int i = 0; i < MAX_ANIMATIONS; i++) {

    // check if this animation index is already taken
    if (hasAnimationFrame(pinId, i)) {

      // check for a new start time that might be later
      long elegibleStartTime = animationQueueArr[pinId][i][ANIMATION_START_TIME_INDEX] + animationQueueArr[pinId][i][ANIMATION_DURATION_INDEX];

      // check if it is later
      if (elegibleStartTime > startTime) {
        startTime = elegibleStartTime;
        startState = animationQueueArr[pinId][i][ANIMATION_TARGET_STATE_INDEX];
      }

      continue;
    }

    // guard: check if we should still set to the new index
    if (newAnimationIndex < 0) {

      // set the new index
      newAnimationIndex = i;
    }
  }

  // guard: check if we found a new animation index
  if (newAnimationIndex < 0) {
    return;
  }

  // calculations based on parameters
  int duration = ((float) abs(targetState - startState)) / ((float) speed) * 1000;

  // validate target state
  targetState = validateState(targetState, pinId, type);

  // check the type to adjust the duration
  if (type == MOTOR_ANIMATION_TYPE) {
    duration = duration / motorSpeedMultiplier;
  } else if (type == LIGHT_ANIMATION_TYPE) {
    duration = duration / lightSpeedMultiplier;
  }

  //Serial.println(newAnimationIndex);
  // add this target to the movement queue
  animationQueueArr[pinId][newAnimationIndex][ANIMATION_START_TIME_INDEX] = startTime;
  animationQueueArr[pinId][newAnimationIndex][ANIMATION_START_STATE_INDEX] = startState;
  animationQueueArr[pinId][newAnimationIndex][ANIMATION_TARGET_STATE_INDEX] = targetState;
  animationQueueArr[pinId][newAnimationIndex][ANIMATION_DURATION_INDEX] = duration;
  animationQueueArr[pinId][newAnimationIndex][ANIMATION_TYPE_INDEX] = type;
  animationQueueArr[pinId][newAnimationIndex][ANIMATION_EASING_INDEX] = easing;
}

/**
   Request a specific movement of a motor in relation to its current location
   NOTE: distance in degrees and speed in degrees per second
*/
void requestAnimationByDelta(int pinId, int deltaState, int speed, int type, int easing) {

  // variables
  int currentState = getState(pinId);
  int targetState = currentState + deltaState;

  // finally request movement by target
  requestAnimationByTarget(pinId, targetState, speed, type, easing);
}

/**
   Request a motor movement by target
*/
void requestMovementByTarget(int moduleId, int motorId, int targetState, int speed, int easing) {
  requestAnimationByTarget(getMotorPin(moduleId, motorId), targetState, speed, MOTOR_ANIMATION_TYPE, easing);
}

/**
   Request a motor movement by target
*/

/**
   Check whether a movement is queued for a specific motor
*/
boolean isAnimating(int pinId) {

  // loop all the possible frames
  for (int i = 0; i < MAX_ANIMATIONS; i++) {

    // guard: check if this frame is animating
    if (hasAnimationFrame(pinId, i)) {
      return true;
    }
  }

  return false;
}

/**
   Check for an animation on a specific frame
*/
boolean hasAnimationFrame(int pinId, int frameIndex) {

  // get the start time, this variables is 0 when nothing is queued
  long startTime = animationQueueArr[pinId][frameIndex][ANIMATION_START_TIME_INDEX];

  // guard: check the start time
  if (startTime <= 0) {
    return false;
  }

  return true;
}

/**
   Reset the animation for a specific pin
*/
void resetAnimation(int pinId) {

  // loop all the frames
  for (int i = 0; i < MAX_ANIMATIONS; i++) {

    // reset each frame
    resetAnimationFrame(pinId, i);
  }
}

/**
   Reset a single animation frame
*/
void resetAnimationFrame(int pinId, int frameIndex) {

  // reset each frame
  animationQueueArr[pinId][frameIndex][ANIMATION_START_TIME_INDEX] = 0;
}

/**
   Set the location of the motor
*/
void writeMotorLocation(int motorPin, int location) {

  // variables
  Servo motor = getMotor(motorPin);

  // set the state
  setState(motorPin, location);
  // send the location to the motor
  motor.write(location);

}

/**
   Validate a state by its type
   We will check the minimum and maximum value
*/
int validateState(int state, int pinId, int type) {

  // check type for minimum, maximum and duration
  if (type == MOTOR_ANIMATION_TYPE) {

    // variables
    int minDegrees = MIN_DEGREES;
    int maxDegrees = MAX_DEGREES;

    // check the min location
    if (state < minDegrees) {
      state = minDegrees;
    }

    // check the max location
    if (state > maxDegrees) {
      state = maxDegrees;
    }

  } else if (type == LIGHT_ANIMATION_TYPE) {

  }

  return state;
}

/**-----------------------------------------------------------------------------------------------------------**/
/**                                              GETTERS & SETTERS                                            **/
/**-----------------------------------------------------------------------------------------------------------**/

/**
   Get a motor at a specific index
*/
Servo getMotor(int motorPin) {
  return motorArr[motorPin];
}

/**
   Get the pin for a specific motor
*/
int getMotorPin(int moduleId, int motorId) {
  return motorPinArr[moduleId][motorId];
}

/**
   Get the current location of a motor
*/
int getMotorLocation(int moduleId, int motorId) {
  return getState(getMotorPin(moduleId, motorId));
}

/**
   Set the state of a device
*/
void setState(int pinId, int state) {
  stateArr[pinId] = state;
}

/**
   Get the state of a device
*/
int getState(int pinId) {
  return stateArr[pinId];
}

/**-----------------------------------------------------------------------------------------------------------**/
/**                                                  TIMERS                                                   **/
/**-----------------------------------------------------------------------------------------------------------**/

/**
   Get a last timer value
*/
long *getTimer(int id) {
  return timerArr[id];
}

/**
   Check whether the timer value has passed
*/
boolean hasPassedTimer(int id) {

  // variables
  long *timerProperties = getTimer(id);

  return millis() >= (timerProperties[TIMER_LAST_TIME_INDEX] + timerProperties[TIMER_DELAY_INDEX]);
}

/**
   Update the timer to the current millis
*/
void updateTimer(int id) {
  setTimer(id, millis() + getTimer(id)[TIMER_DELAY_INDEX]);
}

/**
   Set a timer to a specific value
*/

void setTimer(int id, long time) {
  timerArr[id][TIMER_LAST_TIME_INDEX] = time;
}




/**-----------------------------------------------------------------------------------------------------------**/
/**                                          capsense + caculation                                            **/
/**-----------------------------------------------------------------------------------------------------------**/

int linPot = A1;
int rotPot = A0;
int linVal;
int rotVal;
int linDist;
int rotDist;
long linSpd;
long rotSpd;
long t = 0;

void readTouchInputs() {
  currtouched = cap.touched();

  for (uint8_t i = 0; i < 1; i++) {
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      resetAnimation();
      detachMotors();
      Serial.println("touched");
      linVal = getPos(linPot);
      rotVal = getPos(rotPot);
      t = millis();
    }

    if ((currtouched & _BV(i))) {

    }

    // if it *was* touched and now *isnt*, alert!
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      attachMotors();
      int lin = getMotorPin(0, 0);
      int rot = getMotorPin(1, 0);

      linDist = linVal - getPos(linPot);
      rotDist = rotVal - getPos(rotPot);
      linSpd = linDist / ((millis() - t));
      rotSpd = rotDist / ((millis() - t));

      Serial.print("linDist: ");
      Serial.print(linDist);
      Serial.print("\t");
      Serial.print("rotDist: ");
      Serial.print(rotDist);
      Serial.print("\t");
      Serial.print("linSpd: ");
      Serial.print(linSpd);
      Serial.print("\t");
      Serial.print("rotSpd: ");
      Serial.print(rotSpd);
      Serial.print("\t");
      Serial.print("time: ");
      Serial.println(millis() - t);
      t = millis();
      Serial.println(" released");
    }

    if (!(currtouched & _BV(i))) {
      if (millis() - t > random(7500, 12500)) {
        resetAnimation(getMotorPin(1, 0));
        resetAnimation(getMotorPin(0, 0));
        applyBehaviour(1);
        t = millis();
      }
    }
  }
  // reset our state
  lasttouched = currtouched;
}

/**
   simple command to detach all motors
*/

void detachMotors() {

  // initialize all the modules
  for (int moduleId = 0; moduleId < MODULE_AMOUNT; moduleId++) {

    // loop all the motors
    for (int motorId = 0; motorId < MODULE_MOTOR_AMOUNT; motorId++) {

      // variables
      int motorPin = getMotorPin(moduleId, motorId);
      Servo motor = getMotor(motorPin);

      // guard: check the motor pin
      if (motorPin < 0) {
        continue;
      }

      // attach the motor pin
      motor.detach();
      // calibrate all motors to initial value
    }
  }
}

/**
   attach motors
*/

void attachMotors() {

  // initialize all the modules
  for (int moduleId = 0; moduleId < MODULE_AMOUNT; moduleId++) {

    // loop all the motors
    for (int motorId = 0; motorId < MODULE_MOTOR_AMOUNT; motorId++) {

      // variables
      int motorPin = getMotorPin(moduleId, motorId);
      Servo motor = getMotor(motorPin);
      // guard: check the motor pin
      if (motorPin < 0) {
        continue;
      }

      // set pin mode

      // attach the motor pin
      motor.write(getState(motorPin));
      motor.attach(motorPin);
    }
  }
}

/**
   command to get current position servo. include mapping to align servo values with potmeter. Calibrated in "calibrate"
*/

int getPos(int analogPin)
{
  if (analogPin == A0) {
    return map(analogRead(analogPin), linMinFeedback, linMaxFeedback, linMinDegrees, linMaxDegrees);
  }
  if (analogPin == A1) {
    return map(analogRead(analogPin), rotMinFeedback, rotMaxFeedback, rotMinDegrees, rotMaxDegrees);
  }
}

