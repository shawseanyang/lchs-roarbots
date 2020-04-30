#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"
//
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"
//
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"
//
/*
* ------------------------------------------------------------------------
* Project: xray-bougie-v7.2
* Author: Shaw-Sean Yang
* Date: 1/23/2020
* Desc: Sets up ports for motors and sensors for the main.cpp file
* ------------------------------------------------------------------------
*/

// Set up brain
vex::brain Brain;

/*
* !! OUTDATED !!
* PORT TO MOTOR SET UP REFERENCE
* 
* Port 1: rampLiftMotor [Motor] (Moves ramp forward and back, placing stack or stacking)
* Port 2: rightIntakeMotor [Motor] (Intake spinning motor on the right arm)
* Port 3: leftIntakeMotor [Motor] (Intake spinning motor on the left arm)
* Port 4: armPivotMotor [Motor] (Controls pitch of the two arms)
* Port 5: [VEX Bluetooth Transmitter]
*
* Port 6: baseTopRightMotor [Motor] (Top right driving wheel on base)
* Port 7: baseTopLeftMotor [Motor] (Top left driving wheel on base)
* Port 8: 
* Port 9:
* Port 10: 
*
* Port 11: baseBottomRightMotor [Motor] (Bottom right driving wheel on base)
* Port 12: baseBottomLeftMotor [Motor] (Bottom left driving wheel on base)
* Port 13:
* Port 14: 
* Port 15: 
*
* Port 16:
* Port 17:
* Port 18:
* Port 19: 
* Port 20: 
*
*/

// Set up motors
vex::motor baseTopLeftMotor (vex::PORT9, vex::gearSetting::ratio6_1);
vex::motor baseTopRightMotor (vex::PORT8, vex::gearSetting::ratio6_1);
vex::motor baseBottomLeftMotor (vex::PORT2, vex::gearSetting::ratio6_1);
vex::motor baseBottomRightMotor (vex::PORT1, vex::gearSetting::ratio6_1);

vex::motor leftIntakeMotor (vex::PORT4, vex::gearSetting::ratio6_1);
vex::motor rightIntakeMotor (vex::PORT14, vex::gearSetting::ratio6_1);

vex::motor rampLiftMotor (vex::PORT10, vex::gearSetting::ratio36_1);
vex::motor armPivotMotor (vex::PORT7, vex::gearSetting::ratio36_1);

// Set up Controller
vex::controller Controller = vex::controller();
