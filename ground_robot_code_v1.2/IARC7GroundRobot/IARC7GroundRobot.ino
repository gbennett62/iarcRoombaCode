/*******************************************************************************
 * 
 *  Ground Robot Firmware, developed for IARC Mission 7
 *
 *  Developed under Contract to IARC Design Committee
 *  
 *  Author: Sterling Lewis Peet <sterling.peet@gatech.edu>
 *  Date:   January 3, 2014
 * 
 *  All Rights Reserved, please contact the IARC Competition Committee for
 *  permissions requests.
 *
 ******************************************************************************/

#include <FiniteStateMachine.h>

////////////////////////////////////////////////////////////////////////////////
// Version Information
const byte VERSION_MAJOR = 1;
const byte VERSION_MINOR = 2;

////////////////////////////////////////////////////////////////////////////////
// Hardware Constants

//const byte rxPin = 0;
//const byte txPin = 1;
const byte brcPin = 6;

const byte targetSwitchPin = 7; // The x position is target
const byte runSigPin = 8; // Go Button
const byte waitSigPin = 2; // Stop Button
const byte topTouchPin = 3; // Magnetic Sensors

const byte greenLed = 9;
const byte redLed = 10;

// Design Configuration Constants
const int robotSpeed = 330; // mm/s

// Global Variables

// Robot Behavior State Machine
State Start =             State(chooseType);
State ObstacleWait =      State(obsWaitStart, obsWait, nullFunc);
State ObstacleRun =       State(obsRunStart, obsRun, coiStopMoving);
State ObstacleCollision = State(obsCrashStart, obsCrash, nullFunc);
State TargetWait =        State(trgtWaitStart, trgtWait, trgtWaitExit);
State TargetRun =         State(trgtRunStart, trgtRun, nullFunc);
State TrajectoryNoise =   State(vNoiseStart, vNoise, nullFunc);
State Reverse =           State(vReverseStart, vReverse, nullFunc);
State TargetCollision =   State(trgtCrashStart, trgtCrash, nullFunc);
State TopTouch =          State(touchStart, touch, nullFunc);
//New
State CircleRun =         State(circRunStart, genRun, nullFunc);
State InfRun =            State(infRunStart, genRun, nullFunc);
State ReverseRand =       State(vReverseStart, vReverseRand, nullFunc);

FSM fsm = FSM(Start);     // Initialize state machine, set start state

void setup() {
  Serial.begin(38400);
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  
  // Print out the Sketch and Version Info
  Serial.println("IARC Mission 7 Ground Robot Firmware");
  Serial.print("Release Version: ");
  Serial.print(VERSION_MAJOR);
  Serial.print(".");
  Serial.println(VERSION_MINOR);
  
  initRC();

  // if analog input pin 0 is unconnected, random analog
  // noise will cause the call to randomSeed() to generate
  // different seed numbers each time the sketch runs.
  // randomSeed() will then shuffle the random function.
  randomSeed(analogRead(0));

  // Indicate initialization mode
  digitalWrite(greenLed, HIGH);
  digitalWrite(redLed, HIGH);

  coiInit();
  coiSafeMode();
  delay(200);

  // Stop indication
  digitalWrite(greenLed, LOW);
  digitalWrite(redLed, LOW);
}

void loop() {
  fsm.update();
}



