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
* Project: omni_hdrive_v5.2
* Author: Shaw-Sean Yang
* Date: 11/11/2019
* Desc: Sets up ports for motors and sensors for the main.cpp file
* ------------------------------------------------------------------------
*/

// Set up brain
vex::brain Brain;

/*
* PORT TO MOTOR SET UP REFERENCE
* 
* Port 1: baseTopLeftMotor [Motor] (Top left driving wheel on base)
* Port 2: baseTopRightMotor [Motor] (Top right driving wheel on base)
* Port 3: baseBottomLeftMotor [Motor] (Bottom left driving wheel on base)
* Port 4: baseBottomRightMotor [Motor] (Bottom right driving wheel on base)
* Port 5:
*
* Port 6: 
* Port 7: 
* Port 8: 
* Port 9:
* Port 10: 
*
* Port 11: leftIntakeMotor [Motor] (Intake spinning motor on the left arm)
* Port 12: 
* Port 13:
* Port 14: 
* Port 15: rampLiftMotor [Motor] (Moves ramp forward and back, placing stack or stacking)
*
* Port 16:
* Port 17:
* Port 18:
* Port 19: rightIntakeMotor [Motor] (Intake spinning motor on the right arm)
* Port 20: armPivotMotor [Motor] (Controls pitch of the two arms)
*
*/

// Set up motors
vex::motor baseTopLeftMotor (vex::PORT1, vex::gearSetting::ratio6_1);
vex::motor baseTopRightMotor (vex::PORT2, vex::gearSetting::ratio6_1);
vex::motor baseBottomLeftMotor (vex::PORT3, vex::gearSetting::ratio6_1);
vex::motor baseBottomRightMotor (vex::PORT4, vex::gearSetting::ratio6_1);

vex::motor leftIntakeMotor (vex::PORT11, vex::gearSetting::ratio6_1);
vex::motor rightIntakeMotor (vex::PORT19, vex::gearSetting::ratio6_1);

vex::motor rampLiftMotor (vex::PORT15, vex::gearSetting::ratio36_1);
vex::motor armPivotMotor (vex::PORT20, vex::gearSetting::ratio36_1);

// Set up Controller
vex::controller Controller = vex::controller();
