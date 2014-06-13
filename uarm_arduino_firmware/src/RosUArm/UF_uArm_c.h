/******************************************************************************
* File Name          : UF_uArm.h
* Author             : Evan
* Updated            : Evan
* Version            : V0.1 (BATE)
* Created Date       : 2 MAY, 2014
* Modified Date      : 29 MAY, 2014
* Description        : 
* License            : 
* Copyright(C) 2014 UFactory Team. All right reserved.
*******************************************************************************/

#ifndef UF_uArm_c_h
#define UF_uArm_c_h
#define PIEZOBUZZER

/****************  Macro definitions  ****************/
#define ARM_A                   148    // upper arm
#define ARM_B                   160    // lower arm
#define ARM_2AB                 47360  // 2*A*B
#define ARM_A2                  21904  // A^2
#define ARM_B2                  25600  // B^2
#define ARM_A2B2                47504  // A^2 + B^2
#define ARM_STRETCH_MIN         0
#define ARM_STRETCH_MAX         210
#define ARM_HEIGHT_MIN          -180
#define ARM_HEIGHT_MAX          150
#define ARM_ROTATION_MIN        -90
#define ARM_ROTATION_MAX        90
#define HAND_ROTATION_MIN       -90
#define HAND_ROTATION_MAX       90
#define HAND_ANGLE_OPEN         25
#define HAND_ANGLE_CLOSE        70
#define FIXED_OFFSET_L          18
#define FIXED_OFFSET_R          36
#define D150A_SERVO_MIN_PUL     535
#define D150A_SERVO_MAX_PUL     2415
#define D009A_SERVO_MIN_PUL     700
#define D009A_SERVO_MAX_PUL     2650
#define SAMPLING_DEADZONE       2
#define INIT_POS_L              37
#define INIT_POS_R              25
#define BTN_TIMEOUT_MS          3000
#define CATCH					0x01
#define RELEASE					0x02
#define CALIBRATION_FLAG		0xEE

#define SERVOL_MIN 951 
#define SERVOL_MAX 2415 
#define SERVOR_MIN 700 
#define SERVOR_MAX 1800 
#define SERVOROT_MIN 600 
#define SERVOROT_MAX 2320 
#define SERVOHAND_OPEN_MIN 890 
#define SERVOHAND_CLOSE_MAX 1304 


#define PID_SERVO_L             0    //
#define PID_SERVO_R             1    //
#define PID_SERVO_ROT           2    //
#define PID_SERVO_HAND_ROT      3    //
#define PID_SERVO_HAND          4     //
#define PID_SERVO_CAMERA_ROT    5    //
#define PID_SERVO_CAMERA_TILT   6     //


/*****************  Port definitions  *****************/
#define BTN_D4                  4     //
#define BTN_D7                  7     //
#define BUZZER                  3     //
#define LIMIT_SW                2     // Limit Switch
#define PUMP_EN                 6     //
#define VALVE_EN                5     //
#define SERVO_HAND              9     //
#define SERVO_HAND_ROT          10    //
#define SERVO_ROT               11    //
#define SERVO_R                 12    //
#define SERVO_L                 13    //
#define SERVO_CAMERA_ROT        8    //
#define SERVO_CAMERA_TILT       7     //

#define JOINT_BASE_BODY             0    //
#define JOINT_BODY_UPPER_ARM        1    //
#define JOINT_UPPER_ARM_FOREARM     2    //
#define JOINT_FOREARM_WRIST         3    //
#define JOINT_WRIST_PALM            4     //
#define JOINT_PALM_RIGHT_FINGER     5     //
#define JOINT_PALM_LEFT_FINGER      6     //

int getPositionMicroseconds(int _pidNum){
  int positionMS;
	switch(_pidNum)
	{
		case PID_SERVO_L:
			positionMS = servoL.readMicroseconds();
			break;
		case PID_SERVO_R:
			positionMS = servoR.readMicroseconds();
			break;
		case PID_SERVO_ROT:
			positionMS = servoRot.readMicroseconds();
			break;
		case PID_SERVO_HAND_ROT:
			positionMS = servoHandRot.readMicroseconds();
			break;
		case PID_SERVO_HAND:
			positionMS = servoHand.readMicroseconds();
			break;
		case PID_SERVO_CAMERA_ROT:
			positionMS = servoHandRot.readMicroseconds();
			break;
		case PID_SERVO_CAMERA_TILT:
			positionMS = servoHand.readMicroseconds();
			break;
		default: return 0; 
			break;
	}
	return positionMS;
}    // 

void alert(int _times, int _runTime, int _stopTime)
{
	for(int _ct=0; _ct < _times; _ct++)
	{
#ifdef PIEZOBUZZER
        delay(_stopTime);
        analogWrite(BUZZER, 20);      // Almost any value can be used except 0 and 255
        delay(_runTime);
        analogWrite(BUZZER, 0);       // 0 turns it off
#else
        delay(_stopTime);
        digitalWrite(BUZZER, HIGH);
        delay(_runTime);
        digitalWrite(BUZZER, LOW);
#endif
	}
}
void init_uarm(){
    pinMode(BUZZER,   OUTPUT); digitalWrite(BUZZER,   LOW);
      alert(1, 200, 0);

  	// attaches the servo on pin to the servo object
	servoL.attach(SERVO_L, D150A_SERVO_MIN_PUL, D150A_SERVO_MAX_PUL);
	servoR.attach(SERVO_R, D150A_SERVO_MIN_PUL, D150A_SERVO_MAX_PUL);
	servoRot.attach(SERVO_ROT, SERVOROT_MIN, SERVOROT_MAX);
	servoHand.attach(SERVO_HAND, D009A_SERVO_MIN_PUL, D009A_SERVO_MAX_PUL);
	servoHandRot.attach(SERVO_HAND_ROT, D009A_SERVO_MIN_PUL, D009A_SERVO_MAX_PUL);
	servoCameraRot.attach(SERVO_CAMERA_ROT, D009A_SERVO_MIN_PUL, D009A_SERVO_MAX_PUL);
	servoCameraTilt.attach(SERVO_CAMERA_TILT, D009A_SERVO_MIN_PUL, D009A_SERVO_MAX_PUL);

	servoR.writeMicroseconds(SERVOR_MIN);
	servoL.writeMicroseconds(SERVOL_MAX);
	servoHandRot.write(90);
	servoHand.write(HAND_ANGLE_CLOSE);
	servoCameraRot.write(90);
	servoCameraTilt.write(90);
	servoRot.write(90);

      delay(3000);
      alert(1, 200, 0);

	servoR.detach();
	servoL.detach();
	servoHand.detach();
	servoRot.detach();
        servoCameraRot.detach();
	servoCameraTilt.detach();
	servoHandRot.detach();
}

#endif

