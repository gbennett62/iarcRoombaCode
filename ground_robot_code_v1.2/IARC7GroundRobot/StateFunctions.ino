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
// Path Differentiator
// 0 has competition conditions
// 1 is in an infinity loop
// 2 is a circle
// 3 will try to escape the a following object (manual closeness sensor)
const int pi = 3.14159265;
int path = 0;
// distance between wheels
int Rbase = .258; // m
// conditions for run 2
int Rcenter = 1; // m
// condition for run 1
int a = 1; // m
unsigned int infCount = 0;
boolean hasRunInitial = false;
unsigned int infInitInterval = 2*pi/3*(a/2+Rbase/2)/robotSpeed;
unsigned int lineInterval = sqrt(3)*a/robotSpeed;
unsigned int circleInterval = 4*pi/3*(a/2+Rbase/2)/robotSpeed;
// Set to true to force only the maximum noise rather than random
const boolean MAX_NOISE_TEST = false;

const byte randomMax = 64;
// Time between trajectory noise injections
unsigned int noiseInterval = 5000;
unsigned long lastNoise = 0;
// Time between auto-reverse
unsigned int reverseInterval = 20000;
unsigned long lastReverse = 0;
// Time needed to affect trajectory
unsigned int noiseLength = 850;
unsigned long beginNoise = 0;
// Time needed to reverse trajectory
unsigned int reverseLength = 2456; // .33/2 m/s * pi * wheelbase / 2
unsigned long beginReverse = 0;
// Time needed to spin 45 degrees
unsigned int topTouchTime = reverseLength/4;
unsigned long beginTopTouch = 0;
boolean isManuevering = false;

////////////////////////////////////////////////////////////////////////////////
// State Machine Functions for Entering a State

void obsWaitStart()
{
  Serial.println("State Change: ObstacleWait");
  coiPassiveMode();
  digitalWrite(redLed, HIGH);
}

void obsRunStart()
{
  Serial.println("State Change: ObstacleRun");
  coiSafeMode();
  digitalWrite(redLed, HIGH);
  digitalWrite(greenLed, LOW);
  // This 9mm/s offset get us close to 5m radius circle trajectory
  //coiDriveDirect(robotSpeed - 9, robotSpeed + 9);
  // Temporary change
  coiDriveDirect(robotSpeed, robotSpeed)
}

void obsCrashStart()
{
  Serial.println("State Change: ObstacleCollision");
  coiStopMoving();
  digitalWrite(redLed, HIGH);
  digitalWrite(greenLed, HIGH);
}

void trgtWaitStart()
{
  Serial.println("State Change: TargetWait");
  digitalWrite(greenLed, HIGH);
  coiPassiveMode();
}

void trgtRunStart()
{
  Serial.println("State Change: TargetRun");
  coiSafeMode();
  coiDriveDirect(robotSpeed, robotSpeed);
  digitalWrite(greenLed, HIGH);
  digitalWrite(redLed, LOW);
  lastNoise = millis();
  lastReverse = millis();
}
// new
void circRunStart()
{
  if (path == 1)
  {
  Rcenter = a/2;
  infCount = infCount + 1;
  }
  Serial.println("State Change: TargetRun");
  coiSafeMode();
  if (infCount % 2 == 0)
  {
  coiDriveDirect(robotSpeed*(Rcenter-.5*Rbase)/(Rcenter+.5*Rbase), robotSpeed);
  }
  else
  {
  coiDriveDirect(robotSpeed*(Rcenter-.5*Rbase)/(Rcenter+.5*Rbase), robotSpeed);
  }
  digitalWrite(greenLed, HIGH);
  digitalWrite(redLed, LOW);
  startOfCycle = millis();
}

void lineRunStart()
{
  Serial.println("State Change: TargetRun");
  coiSafeMode();
  coiDriveDirect(robotSpeed, robotSpeed);
  digitalWrite(greenLed, HIGH);
  digitalWrite(redLed, LOW);
  startOfCycle = millis();
  
}

void vNoiseStart()
{
  Serial.println("State Change: TrajectoryNoise");
  long rand = random(randomMax);
  int offset = (int) rand - randomMax / 2;
  if(MAX_NOISE_TEST)
    offset = 127; // force the maximum noise for testing
  coiDriveDirect(robotSpeed - offset, robotSpeed + offset);
  digitalWrite(redLed, HIGH);
  beginNoise = millis();
}

void vReverseStart()
{
  Serial.println("State Change: Reverse");
  coiDriveDirect(-robotSpeed/2, robotSpeed/2);
  digitalWrite(redLed, HIGH);
  beginReverse = millis();
}

void trgtCrashStart()
{
  Serial.println("State Change: TargetCollision");
  coiStopMoving();
}

void touchStart()
{
  Serial.println("State Change: TopTouch");
  coiDriveDirect(-robotSpeed/2, robotSpeed/2);
  digitalWrite(redLed, HIGH);
  beginTopTouch = millis();
}

////////////////////////////////////////////////////////////////////////////////
// State Machine Functions for Updating a State

void obsWait()
{
  if(isRunSig())
  {
    fsm.transitionTo(ObstacleRun);
  }
}

void obsRun()
{
  if(isWaitSig())
  {
    fsm.transitionTo(ObstacleWait);
  }
  else
  {
    if(coiCheckBump() != 0)
    {
      fsm.transitionTo(ObstacleCollision);
    } 
    else
    {
      delay(15);
    }
  }
}

void obsCrash()
{
  if(isWaitSig())
  {
    fsm.transitionTo(ObstacleWait);
  } 
  else
  {
//    byte bump = coiCheckBump();
//    if(bump == 0)
//    {
//      fsm.transitionTo(ObstacleRun);
//    } 
//    else
//    {
//      delay(15);
//    }
      fsm.transitionTo(Reverse);
  }
}

void trgtWait()
{
  if(isRunSig() && path == 0)
  {
    fsm.transitionTo(TargetRun);
  }
  if else (isRunSig() && path == 1)
  {
    fsm.transitionTo(CircleRun);
  }
}

void trgtRun()
{
  if(isWaitSig())
  {
    fsm.transitionTo(TargetWait);
  } 
  else if (isTopTouch())
  {
    fsm.transitionTo(TopTouch);
  } 
  else if (isTimeUp(&lastReverse, &reverseInterval))
  {
    fsm.transitionTo(Reverse);
  }
  else if (isTimeUp(&lastNoise, &noiseInterval))
  {
    fsm.transitionTo(TrajectoryNoise);
  }
  else
  {
    if(coiCheckBump() != 0 || digitalRead(runSigPin))
    {
      fsm.transitionTo(TargetCollision);
    } 
    else
    {
      // Manuel Control
      // seemed immediatly revert back to orginal path unless the button was held
      // Next experiment
      // Hold in lap but DO NOT COMPRESS BUMBPER
//      if(!isManuevering && digitalRead(runSigPin))
//      {
//        coiDriveDirect(robotSpeed, 0);
//        isManuevering = true;
//      }
//      else if (isManuevering && digitalRead(runSigPin))
//      {
//        coiDriveDirect(robotSpeed, robotSpeed);
//        isManuevering = false;
//      }
      delay(15);
    }
  }
}

void circRun()
{
  if(isWaitSig())
  {
    fsm.transitionTo(TargetWait);
  } 
  else if (path == 1)
  {
    if(~hasRunInitial)
    {
      if(isTimeUp(&startOfCycle, &initInfInterval))
      {
        fsm.transitionTo(LineRun);
      }
    }
    else
    {
      if(isTimeUp(&startOfCycle, &circleInterval))
      {
        fsm.transitionTo(LineRun);
      }
    }
  }
  else
  {
    if(coiCheckBump() != 0 )
    {
      fsm.transitionTo(TargetCollision);
    } 
    else
    {
      delay(15);
    }
  }
}

void lineRun()
{
  if(isWaitSig())
  {
    fsm.transitionTo(TargetWait);
  } 
  else if (path == 1)
  {
    if(~hasRunInitial)
    {
      if(isTimeUp(&startOfCycle, &lineInterval))
      {
        fsm.transitionTo(CircleRun);
      }
    }
  }
  else
  {
    if(coiCheckBump() != 0 )
    {
      fsm.transitionTo(TargetCollision);
    } 
    else
    {
      delay(15);
    }
  }
}

void vNoise()
{
  if(isWaitSig())
  {
    fsm.transitionTo(TargetWait);
  }  
  else if (isTopTouch())
  {
    fsm.transitionTo(TopTouch);
  }  
  else if (isTimeUp(&beginNoise, &noiseLength))
  {
    fsm.transitionTo(TargetRun);
  }
  else
  {
    if(coiCheckBump() != 0)
    {
      fsm.transitionTo(TargetCollision);
    } 
    else
    {
      delay(15);
    }
  }
}

void vReverse()
{
  if(isWaitSig())
  {
    fsm.transitionTo(TargetWait);
  }  
//  else if (isTopTouch())
//  {
//    fsm.transitionTo(TopTouch);
//  } 
  else if (isTimeUp(&beginReverse, &reverseLength))
  {
    //fsm.transitionTo(TargetRun);
    fsm.transitionTo(ObstacleRun);
  }
}

void vReverseRand()
{
  unsigned long randRevLength = random(reverseLength/2,3*reverseLength/2)
  if(isWaitSig())
  {
    fsm.transitionTo(TargetWait);
  }  
  else if (isTopTouch())
  {
    fsm.transitionTo(TopTouch);
  } 
  else if (isTimeUp(&beginReverse, &randRevLength))
  {
    fsm.transitionTo(TargetRun);
  }
}

void trgtCrash()
{
  fsm.transitionTo(ReverseRand);
}

void touch()
{
  if(isWaitSig())
  {
    fsm.transitionTo(TargetWait);
  }  
  else if (isTimeUp(&beginTopTouch, &topTouchTime))
  {
    fsm.transitionTo(TargetRun);
  }
  else
  {
    if(coiCheckBump() != 0)
    {
      fsm.transitionTo(TargetCollision);
    } 
    else
    {
      delay(15);
    }
  }  
}

////////////////////////////////////////////////////////////////////////////////
// State Machine Functions for Exiting a State

void trgtWaitExit()
{
  Serial.println("Resetting Noise and Reverse timers");
  lastNoise = millis();
  lastReverse = millis();
}

void nullFunc(){ /* Do Nothing, but act as a placeholder */
}

