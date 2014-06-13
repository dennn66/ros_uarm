/* 
*  Functions and type-defs for PID control.
*
*  Based on the Beginner PID's series by Brett Beauregard - http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
*  Adapted to use ideal velocity form or position form.
*
*  Originally adapted from Mike Ferguson's ArbotiX code which lives at:
*   
*  http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/
 /* PID setpoint info For a Motor */
  typedef struct {
    int targetVelocity;// ms/sec
    int targetPosition; // estimated position
    int targetTicksPerFrame; // target speed in ticks per frame
    long encoder; // current position
    long prevEnc; // last position
  
    /*
* Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
* see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
*/
    int prevInput; // last input
    //int prevErr; // last error
  
    /*
* Using integrated term (ITerm) instead of integrated error (Ierror),
* to allow tuning changes,
* see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
*/
    long iTerm; //integrated term
    int output; // last motor setting
  }
  SetPointInfo;

//SetPointInfo leftPID, rightPID;
#define PIDS_NUM 5
SetPointInfo      PID[PIDS_NUM];

/* PID Parameters 
* Do not SET these directly here, unless you know what you are doing 
* Use setPIDParameters() instead
*/
int Kp = 0;    
int Kd = 0;
int Ki = 0;      
int Ko = 1; 

unsigned char moving = 0; // is the base in motion?

/*
* Initialize PID variables to zero to prevent startup spikes
* when turning PID on to start moving
* In particular, assign both encoder and prevEnc the current encoder value
* See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
* Note that the assumption here is that PID is only turned on
* when going from stop to moving, that's why we can init everything on zero.
*/
void resetPID(int pidnum){
     PID[pidnum].targetTicksPerFrame = 0;
     PID[pidnum].encoder = getPositionMicroseconds(pidnum);
     PID[pidnum].targetPosition = PID[pidnum].prevEnc = PID[pidnum].encoder;
     PID[pidnum].output = 0;
     PID[pidnum].prevInput = 0;
     PID[pidnum].iTerm = 0;
}

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  int error;
  int output;
  int input;

  input = p->encoder - p->prevEnc;
  error = p->targetTicksPerFrame - input;
  
  p->iTerm += (Ki * error) / Ko;
  if (p->iTerm > (MAX_DELTA-MIN_DELTA)) p->iTerm = MAX_DELTA;
  else if (p->iTerm < (-MAX_DELTA+MIN_DELTA)) p->iTerm = -MAX_DELTA;

  /*
  * Avoid derivative kick and allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  output = (((long)Kp) * error - Kd * (input - p->prevInput))/ Ko + p->iTerm;
  p->prevEnc = p->encoder;

  /*
  * Accumulate Integral error *or* Limit output.
  * 
  * Avoid motor moving back when requesting forward movement, and vice versa (avoid oscillating around 0)
  * Also avoid sending output of 0 (stopping motors)
  *
  * Stop accumulating integral error when output is limited.
  */
  if (p->targetTicksPerFrame > 0){
    output += MIN_DELTA;
    if (output < MIN_DELTA) output = MIN_DELTA;
  } else if (p->targetTicksPerFrame < 0){
    output += -MIN_DELTA;
    if (output > -MIN_DELTA) output = -MIN_DELTA;
  } 
  
  if (output > MAX_DELTA)
    output = MAX_DELTA;
  else if (output < -MAX_DELTA)
    output = -MAX_DELTA;

  p->output = output;
  p->prevInput = input;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  for(int i=0;i<PIDS_NUM;i++) PID[i].encoder = getPositionMicroseconds(i);

  /* Compute PID update for each motor */
  moving = 0;
  for(int i=0;i<PIDS_NUM;i++){
      if(abs(PID[i].targetPosition - PID[i].encoder) > SERVO_PRECISSION ) {
        PID[i].targetTicksPerFrame = (PID[i].targetPosition - PID[i].encoder);
        if(abs(PID[i].targetTicksPerFrame) > abs(PID[i].targetVelocity/PID_RATE)) {
            PID[i].targetTicksPerFrame = (PID[i].targetTicksPerFrame/abs(PID[i].targetTicksPerFrame))*abs(PID[i].targetVelocity/PID_RATE);
        }
        moving = 1;
        doPID(&(PID[i]));
  /* Set the motor position accordingly */

     } else {

    /*
    * Reset PIDs, to prevent startup spikes,
    * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    * Most importantly, keep Encoder and PrevEnc synced; use that as criteria whether we need reset
    */

        if (PID[i].prevEnc != PID[i].encoder) resetPID(i);
      }
   }
}


/* Set PID parameters */
//Assuming pid_rate and Kx parameters all given in s
//Doing some effort to keep things in integer math, through use of Ko
void setPIDParams(int newKp, int newKd, int newKi, int newKo, int pidRate){
    Kp = newKp * pidRate;
    Ki = newKi;
    Kd = newKd * pidRate * pidRate;  
    Ko = newKo * pidRate;
}




