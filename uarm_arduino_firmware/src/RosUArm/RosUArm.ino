#include <ArduinoHardware.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>
//#include <std_msgs/Bool.h>
//#include <uarm_msgs/Joints.h>


#include <Servo.h>
#include <EEPROM.h>

Servo servoR;
Servo servoL;
Servo servoRot;
Servo servoHand;
Servo servoHandRot;
Servo servoCameraRot;
Servo servoCameraTilt;

#include "UF_uArm_c.h"

// Run the PID loop at 30 times per second 
#define PID_RATE           30    // Hz
// Convert the rate into an interval 
const int PID_INTERVAL = 1000 / PID_RATE;
//Track the next time we make a PID calculation
unsigned long nextPID = PID_INTERVAL;


// Run the JointState loop at 10 times per second 
#define JS_RATE           10    // Hz
// Convert the rate into an interval 
const int JS_INTERVAL = 1000 / JS_RATE;
//Track the next time we make a PID calculation
unsigned long nextJS = JS_INTERVAL;


// Run the Manual Input loop at 3 times per second 
#define MI_RATE           3    // Hz
// Convert the rate into an interval 
const int MI_INTERVAL = 1000 / MI_RATE;
//Track the next time we make a PID calculation
unsigned long nextMI = MI_INTERVAL;


/* Initial PID Parameters */
const int INIT_KP = 600;
const int INIT_KD = 300;
const int INIT_KI = 350;
const int INIT_KO = 50;

//Max servo speed MS per sec
#define MAX_SERVO_SPEED 400
//Precission of reaching target position
#define SERVO_PRECISSION 5
#define MAX_DELTA MAX_SERVO_SPEED/PID_RATE
#define MIN_DELTA 1

#include "diff_controller.h"

/*
void jointCallback(const uarm_msgs::Joints &command) {
//  uarm_robot.setPosition(command.stretch, command.height,
//                         command.arm_rot, command.hand_rot);
}

void gripperCallback(const std_msgs::Bool &command) {
  if (command.data) {
//    uarm_robot.gripperCatch();
  } else {
 //   uarm_robot.gripperRelease();
  }
}
*/

ros::NodeHandle nh;
float positions[7] = {0.0};
char* joints[7] = {"base_body_j",
"body_upper_arm_j",
"upper_arm_forearm_j",
"forearm_wrist_j",
"wrist_palm_j",
"palm_left_finger_j",
"palm_right_finger_j"
};

//ros::Subscriber<std_msgs::Bool> gripper_sub("gripper", gripperCallback);
//ros::Subscriber<uarm_msgs::Joints> joint_sub("joint_commands", jointCallback);
                                                   
sensor_msgs::JointState joint_state_msg;
ros::Publisher joint_state_pub("joint_states", &joint_state_msg);


void setup() 
{
  setPIDParams(INIT_KP, INIT_KD, INIT_KI, INIT_KO, PID_RATE);
  for(int i=0;i<PIDS_NUM;i++) {
     resetPID(i);
     PID[i].targetVelocity = 0;
  }

  init_uarm();          // initialize the uArm position

  joint_state_msg.position_length = 7;
  joint_state_msg.name_length = 7;
  joint_state_msg.position = positions;
  joint_state_msg.name = joints;
  nh.initNode();
  nh.advertise(joint_state_pub);
//  nh.subscribe(gripper_sub);
//  nh.subscribe(joint_sub);
  delay(500);
}


void loop()
{
  
 float alpha;
 float betta;
 float gamma;


 float alpha_offset;
 float betta_offset;

//  delay(100);

  if (millis() > nextPID) {
    nextPID = millis() + PID_INTERVAL;
    updatePID();
    servoL.writeMicroseconds(PID[0].encoder+PID[0].output);
    servoR.writeMicroseconds(PID[1].encoder+PID[1].output);
    servoRot.writeMicroseconds(PID[2].encoder+PID[2].output);
    servoHandRot.writeMicroseconds(PID[3].encoder+PID[3].output);
    servoHand.writeMicroseconds(PID[4].encoder+PID[4].output);
//    servoCameraRot.writeMicroseconds(map(servoRot.readMicroseconds(), D150A_SERVO_MIN_PUL,  D150A_SERVO_MAX_PUL, D009A_SERVO_MAX_PUL,  D009A_SERVO_MIN_PUL));
//    servoCameraTilt.writeMicroseconds(map(analogRead(3), 0, 1023, D009A_SERVO_MIN_PUL,  D009A_SERVO_MAX_PUL));
 }
  if (millis() > nextJS) {
    nextJS = millis() + JS_INTERVAL;
    //alpha_offset = (((float)(analogRead(2)))/((float)(1023)))*2*PI-PI; 
    //betta_offset = (((float)(analogRead(3)))/((float)(1023)))*2*PI-PI; 
    #define BODYROT  (((float)(servoRot.readMicroseconds()-SERVOROT_MIN))/((float)(SERVOROT_MAX-SERVOROT_MIN)))*PI
    alpha = (((float)(servoR.readMicroseconds()-D150A_SERVO_MIN_PUL))/((float)(D150A_SERVO_MAX_PUL-D150A_SERVO_MIN_PUL)))*PI;
    betta = (((float)(servoL.readMicroseconds()-D150A_SERVO_MIN_PUL))/((float)(D150A_SERVO_MAX_PUL-D150A_SERVO_MIN_PUL)))*PI;
    #define HANDROT  (((float)(servoHandRot.readMicroseconds()-D009A_SERVO_MIN_PUL))/((float)(D009A_SERVO_MAX_PUL-D009A_SERVO_MIN_PUL)))*PI
    gamma =  (((float)(servoHand.readMicroseconds()-SERVOHAND_CLOSE_MAX))/((float)(SERVOHAND_CLOSE_MAX-SERVOHAND_OPEN_MIN)))*0.43;

    joint_state_msg.header.stamp = nh.now();
    joint_state_msg.position[JOINT_BASE_BODY]          = BODYROT-PI/2;
    joint_state_msg.position[JOINT_BODY_UPPER_ARM]     = PI-betta; //"body_upper_arm_j"
    joint_state_msg.position[JOINT_UPPER_ARM_FOREARM]  = alpha+betta;
    joint_state_msg.position[JOINT_FOREARM_WRIST]      = PI-alpha; 
    joint_state_msg.position[JOINT_WRIST_PALM]         = HANDROT-PI/2;
    joint_state_msg.position[JOINT_PALM_RIGHT_FINGER]  = -gamma;
    joint_state_msg.position[JOINT_PALM_LEFT_FINGER]   = gamma;

    joint_state_pub.publish(&joint_state_msg);
      nh.spinOnce();
 }
  if (millis() > nextMI) {
   //servoCameraRot.attach(SERVO_CAMERA_ROT, D009A_SERVO_MIN_PUL, D009A_SERVO_MAX_PUL);
   nextMI = millis() + MI_INTERVAL;
// SERVO_ROT   SERVO_L  SERVO_HAND_ROT  SERVO_HAND 
//  if(moving == 0){
//    servoL.writeMicroseconds(map(analogRead(0), 0, 1023, SERVOL_MIN,  SERVOL_MAX));
//    servoR.writeMicroseconds(map(analogRead(1), 0, 1023, SERVOR_MIN,  SERVOR_MAX));
//    servoRot.writeMicroseconds(map(analogRead(2), 0, 1023, SERVOROT_MIN,  SERVOROT_MAX));
//    servoHandRot.writeMicroseconds(map(analogRead(3), 0, 1023, D009A_SERVO_MIN_PUL,  D009A_SERVO_MAX_PUL));
 //   servoHand.writeMicroseconds(map(analogRead(2), 0, 1023, SERVOHAND_OPEN_MIN,  SERVOHAND_CLOSE_MAX));

       //  alert(1, 200, 0);
    PID[1].targetPosition   = map(analogRead(0), 0, 1023, SERVOR_MIN,  SERVOR_MAX); //map(testSequence[testState][0], 0, 180, D150A_SERVO_MIN_PUL,  D150A_SERVO_MAX_PUL); 
    PID[0].targetPosition   = map(analogRead(1), 0, 1023, SERVOL_MIN,  SERVOL_MAX); //map(testSequence[testState][1], 0, 180, D150A_SERVO_MIN_PUL,  D150A_SERVO_MAX_PUL); 
    PID[2].targetPosition   = map(analogRead(2), 0, 1023, SERVOROT_MIN,  SERVOROT_MAX); //map(testSequence[testState][2], 0, 180, D150A_SERVO_MIN_PUL,  D150A_SERVO_MAX_PUL);
    PID[3].targetPosition   = map(analogRead(3), 0, 1023, D009A_SERVO_MIN_PUL,  D009A_SERVO_MAX_PUL);//map(testSequence[testState][3], 0, 180, D009A_SERVO_MIN_PUL,  D009A_SERVO_MAX_PUL);
    PID[4].targetPosition   = servoHand.readMicroseconds();//map(testSequence[testState][4], 0, 180, D009A_SERVO_MIN_PUL,  D009A_SERVO_MAX_PUL);
    PID[0].targetVelocity = MAX_SERVO_SPEED;
    PID[1].targetVelocity = MAX_SERVO_SPEED;
    PID[2].targetVelocity = MAX_SERVO_SPEED/3;
    PID[3].targetVelocity = MAX_SERVO_SPEED;
    PID[4].targetVelocity = MAX_SERVO_SPEED;
 //         testState++;
 //         testState = testState==5?0:testState;
 //  } 

//	servoCameraRot.writeMicroseconds(map(servoRot.readMicroseconds(), D150A_SERVO_MIN_PUL,  D150A_SERVO_MAX_PUL, D009A_SERVO_MAX_PUL,  D009A_SERVO_MIN_PUL));
//	servoCameraTilt.writeMicroseconds(map(analogRead(3), 0, 1023, D009A_SERVO_MIN_PUL,  D009A_SERVO_MAX_PUL));
     // 	servoCameraRot.detach();
 }
  /* delay release valve, this function must be in the main loop */
 // uarm.gripperDetach();  
};
