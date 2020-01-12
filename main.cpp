/*
* ------------------------------------------------------------------------
* Project: omni_hdrive_v6.1
* Author: Shaw-Sean Yang
* Date: 11/11/2019
* Desc: Allows the driver to manually control the xdrive robot using the controller, or preprogram the robot to travel specific distances.
* Version: Modified control mapping for Aiden, added controlled tower placement / back up function, added "hold" type stop on intake motors
* ------------------------------------------------------------------------
*/


#include "robot-config.h"
#include <sstream>

vex::competition Competition;

/*
* ScreenButton class for ScreenButton objects. One instance is created for every clickable menu button displayed on the screen.
*/

/*class ScreenButton {
    private:
        // initialize instance variables with default values
        double xPos = 0;
        double yPos = 0;
        double width = 10;
        double height = 10;
        std::string hexColor = "#FFFFFF";
        std::string displayText = "Button";
        int returnValue = 0;
    
        static ScreenButton* listOfButtons[4];
        static int numberOfButtons;
    
    public:
    
        /* ------------------------------------------------------------------------
        * Function: ScreenButton Constructor
        * Desc: Create a new screen button with parameters
        * Param: 
        *   - x position
        *   - y position
        *   - width
        *   - height
        *   - color (hex code)
        *   - text to display
        *   - value to return when pressed
        * Output: prints text to the robot Brain screen
        *\
        ScreenButton(double myXPos, double myYPos, double myWidth, double myHeight, std::string myHexColor, std::string myDisplayText, int myReturnValue) {
            xPos = myXPos;
            yPos = myYPos;
            width = myWidth;
            height = myHeight;
            hexColor = myHexColor;
            displayText = myDisplayText;
            returnValue = myReturnValue;
            
            // create button
            Brain.Screen.drawRectangle(xPos,yPos,width,height,hexColor.c_str());
            Brain.Screen.printAt(xPos,yPos,displayText.c_str());
            
            // add button to array of all screen buttons
            listOfButtons[numberOfButtons] = this;
            numberOfButtons++;
        }
    
        /* ------------------------------------------------------------------------
        * Function: whichButtonPressed
        * Desc: returns the value of the button which the cursor is currently on
        * Param: 
        *   - cursor X position
        *   - cursor Y position
        * Output: returns the returnValue of the ScreenButton object that is pressed
        *\
        static int whichButtonPressed(double cursorX, double cursorY) {
            
            // check the coordinates of each button to see which one matches the cursor position
            
            bool isThisOne = false;
            for( int i = 0; i < 4; i++) {
                if (cursorX >= listOfButtons[i]->getXPos()) {
                    isThisOne = true;
                } else { isThisOne = false;}
                
                if (cursorX <= listOfButtons[i]->getXPos() + listOfButtons[i]->getWidth()) {
                    isThisOne = true;
                } else { isThisOne = false;}
                
                if (cursorY >= listOfButtons[i]->getYPos()) {
                    isThisOne = true;
                } else { isThisOne = false;}
                
                if (cursorY <= listOfButtons[i]->getYPos() + listOfButtons[i]->getHeight()) {
                    isThisOne = true;
                } else { isThisOne = false;}
                
                if (isThisOne) {
                    return listOfButtons[i]->getReturnValue();
                }
            }
            // return 0 by default
            return 0;
        }
        
        /*
        * Object Instance Variable GET functions
        *\
        double getXPos() {
            return xPos;
        }
        double getYPos() {
            return yPos;
        }
        double getWidth() {
            return width;
        }  
        double getHeight() {
            return height;
        }  
        double getReturnValue() {
            return returnValue;
        }  
}; */


/*
* Robot class for Robot objects. One instance is created in the int main() function. The robot object's robotMain function will be called. 
* This allows variables to be shared among robot functions to avoid passing variables multiple times between functions.
*/

class Robot {
    private:
        /*
        * ------------------------------------------------------------------------
        * SHARED VARIABLES
        *
        * movementThreshold: minimum threshold controller value before base wheels begin moving
        * armPivotCurrentAngle = continuously updated value that holds current arm pivot angle (degrees)
        * armPivotUpperAngle = predefined upper limit of the arm pivot (degrees)
        * armPivotLowerAngle = predefined lower limit of the arm pivot (degrees)
        * rampLiftCurrentAngle = continuously updated value that holds current ramp lift angle (degrees)
        * rampLiftUpperAngle = predefined upper limit of the ramp lift (degrees)
        * rampLiftLowerAngle = predefined lower limit of the ramp lift (degrees)
        * wheelCircumference: circumference of the wheel used for encoder rotation to distance calculations (meters)
        * linearDistanceSoFar: continuously updating autonomous linear distance travelled since last movement command (meters)
        * previouslinearRotation: last updated rotation value used to calculate change distance so far (degrees)
        * kpLinear: PID poportional constant (Kp)
        * rotationalDistanceSoFar: continuously updating autonomous rotational angle travelled since last movement command (degrees)
        * previousRotationalRotation: last updated rotation value used to calculate change in rotational angle so far (degrees)
        * kpRotational: PID poportional constant (Kp)
        * linearPrecisionThreshold: predefined maximum difference between linear target distance and distance so far for the movement command to complete (meters)
        * rotationalPrecisionThreshold: predefined maximum difference between rotional target angle and angle so far for the movement command to complete (degrees)
        */
    
        //bool autonomousSelected = false;
        //int autonomousSelection = 0;

        double movementThreshold = 5;
        
        double armPivotCurrentAngle = 0;
        double armPivotUpperAngle = 10000; // must be >= 0
        double armPivotLowerAngle = 0; // must be <= 0
        
        double rampLiftCurrentAngle = 0; 
        double rampLiftUpperAngle = 890; // must be >= 0
        double rampLiftLowerAngle = 0; // must be <= 0
        double rampLiftSpeed = 0;
    
        double wheelCircumference = 0.319185814; // meters
        double encoderTicksPerRotation = 1100;
        double errorBottomLeft;
        double errorTopRight;
        double errorBottomRight;
    
        double linearDistanceSoFar = 0;
        double kpLinear = 10;
    
        double rotationalAngleSoFar = 0;
        double absoluteTargetAngle = 0;
        double kpRotational = 0.1;
    
        double linearPrecisionThreshold = 0.01;
        double rotationalPrecisionThreshold = 10;

        double intakeCircumference = 0.25215679274; // meters
    
        vex::rotationUnits degreesUnit = vex::rotationUnits::deg;
        vex::velocityUnits percentVelocityUnit = vex::velocityUnits::pct;
        vex::percentUnits percentUnit = vex::percentUnits::pct;
        vex::directionType forwardDirection = vex::directionType::fwd;
        vex::directionType reverseDirection = vex::directionType::rev;
        
        /* ------------------------------------------------------------------------
        * Function: runPrint
        * Param: text to print, number of iterations to print
        * Output: prints text to the robot Brain screen
        */
        void runPrint(std::string text, int iterations) {
            const char* char_name = text.c_str();
            for(int i = 0; i < iterations;i++) {
                Brain.Screen.print(char_name);
                Brain.Screen.newLine();
            };
        };
        void runPrint(std::string text) {
            const char* char_name = text.c_str();
            Brain.Screen.print(char_name);
            Brain.Screen.newLine();
        };
    
        /* ------------------------------------------------------------------------
        * Function: baseMove
        * Desc: forwards, backwards, and rotate movement for driver control
        * Param: 
        *   - linearAxis between [-100, 100] to control forward / backward movement
        *   - rotationalAxis between [-100, 100] to control left / right rotation
        * Output: moves robot base
        */
        void baseMove(double linearAxis, double rotationalAxis) {
            
            // if 0, stop motors
            if (linearAxis == 0 && rotationalAxis == 0) {
                
                baseTopLeftMotor.stop(vex::brakeType::brake);
                baseBottomLeftMotor.stop(vex::brakeType::brake);
                baseTopRightMotor.stop(vex::brakeType::brake);
                baseBottomRightMotor.stop(vex::brakeType::brake);
                
            } else {
                
                baseTopLeftMotor.spin(forwardDirection, linearAxis + rotationalAxis, percentVelocityUnit);
                baseBottomLeftMotor.spin(forwardDirection, linearAxis + rotationalAxis, percentVelocityUnit);
                
                // right motors are reverse to map properly to motor orientation on physical robot
                baseTopRightMotor.spin(reverseDirection, linearAxis - rotationalAxis, percentVelocityUnit);
                baseBottomRightMotor.spin(reverseDirection, linearAxis - rotationalAxis, percentVelocityUnit);
                
            }
        };
        
        /* ------------------------------------------------------------------------
        * Function: linearMove
        * Desc: FOR forward / backwards AUTONOMOUS MOVEMENT.
        * Param: 
        *   - targetDistance for robot to travel by the end of the function in meters. Negative for backwards, Positive for forwards.
        *   - percentSpeed to be applied to the motors [0.0 - 1.0]
        * Output: uses baseMove to move robot base
        */
        void linearMove(double targetDistance, double percentSpeed) {
            
            baseTopLeftMotor.resetRotation();
            baseBottomLeftMotor.resetRotation();
            baseTopRightMotor.resetRotation();
            baseBottomRightMotor.resetRotation();
            
            // set up distances
            linearDistanceSoFar = (baseTopLeftMotor.rotation(degreesUnit)/encoderTicksPerRotation) * wheelCircumference;
            double absoluteTargetDistance = targetDistance - linearDistanceSoFar;
            
            if (targetDistance >= 0) {
                errorBottomLeft = -0.2;
                errorTopRight, errorBottomRight = 0.2;
                
            } else {
                errorBottomLeft = -0.2;
                errorTopRight, errorBottomRight = 0.2;
            }
            
            /*
            // TEMP
            std::ostringstream sstream;
            sstream << targetDistance << " " << absoluteTargetDistance << " " << linearDistanceSoFar;
            std::string printStatement = sstream.str();
            runPrint(printStatement);
            */
            
            // run until robot travels the given distance
            while (fabs(absoluteTargetDistance - linearDistanceSoFar) >= linearPrecisionThreshold) {
                
                /*
                baseTopLeftMotor.spin(forwardDirection, (absoluteTargetDistance - linearDistanceSoFar) * kpLinear, percentVelocityUnit);
                baseBottomLeftMotor.spin(forwardDirection, (absoluteTargetDistance - linearDistanceSoFar) * kpLinear, percentVelocityUnit);

                baseTopRightMotor.spin(reverseDirection, (absoluteTargetDistance - linearDistanceSoFar) * kpLinear, percentVelocityUnit);
                baseBottomRightMotor.spin(reverseDirection, (absoluteTargetDistance - linearDistanceSoFar) * kpLinear, percentVelocityUnit);
                */
                
                // if the difference in target distance and distance so far is positive, go forward; else, go backwards
                if (absoluteTargetDistance - linearDistanceSoFar >= 0) {
                    
                    baseTopLeftMotor.spin(forwardDirection, percentSpeed * 100  , percentVelocityUnit);
                    baseBottomLeftMotor.spin(forwardDirection, percentSpeed * 100 - errorBottomLeft * kpLinear, percentVelocityUnit);

                    baseTopRightMotor.spin(reverseDirection, percentSpeed * 100 + errorTopRight * kpLinear, percentVelocityUnit);
                    baseBottomRightMotor.spin(reverseDirection, percentSpeed * 100 + errorBottomRight * kpLinear, percentVelocityUnit);
                    
                } else {
                    
                    baseTopLeftMotor.spin(reverseDirection, percentSpeed * 100  , percentVelocityUnit);
                    baseBottomLeftMotor.spin(reverseDirection, percentSpeed * 100 - errorBottomLeft * kpLinear, percentVelocityUnit);

                    baseTopRightMotor.spin(forwardDirection, percentSpeed * 100 + errorTopRight * kpLinear, percentVelocityUnit);
                    baseBottomRightMotor.spin(forwardDirection, percentSpeed * 100 + errorBottomRight * kpLinear, percentVelocityUnit);
                }
                
                // to correct for differing speeds on each wheel, calculate the error of each encoder relative to the top left wheel and adjust speeds accordingly
                linearDistanceSoFar = (baseTopLeftMotor.rotation(degreesUnit)/encoderTicksPerRotation) * wheelCircumference;
                errorBottomLeft = linearDistanceSoFar - (baseBottomLeftMotor.rotation(degreesUnit)/encoderTicksPerRotation) * wheelCircumference;
                errorTopRight = linearDistanceSoFar + (baseTopRightMotor.rotation(degreesUnit)/encoderTicksPerRotation) * wheelCircumference;
                errorBottomRight = linearDistanceSoFar + (baseBottomRightMotor.rotation(degreesUnit)/encoderTicksPerRotation) * wheelCircumference;
            }
            
            baseTopLeftMotor.stop(vex::brakeType::brake);
            baseTopRightMotor.stop(vex::brakeType::brake);
            baseBottomLeftMotor.stop(vex::brakeType::brake);
            baseBottomRightMotor.stop(vex::brakeType::brake);
            
        }
        
        /* ------------------------------------------------------------------------
        * Function: rotationalMove
        * Desc: FOR rotating in place / turning a radius AUTONOMOUS MOVEMENT. Uses PID
        * Param: 
        *   - radius for robot follow in meters. // YET TO BE IMPLEMENTED
        *       * 0 to turn in place. Greater than 0 to turn a radius
        *   - targetAngle for robot to travel by the end of the function in degrees. 0 - infinity
        *   - percentSpeed to be applied to the motors ( negative for counter clockwise, positive for clockwise )
        * Output: uses baseMove to move robot base
        */
        void rotationalMove(/*double radius,*/ double targetAngle, double percentSpeed) {
            
            baseTopLeftMotor.resetRotation();
            baseBottomLeftMotor.resetRotation();
            baseTopRightMotor.resetRotation();
            baseBottomRightMotor.resetRotation();
            
            double startingAngle = baseTopLeftMotor.rotation(degreesUnit);
            rotationalAngleSoFar = startingAngle;
            
            // convert targetAngle in degrees to targetAngle in encoder ticks. Add startingAngle to obtain absolute targetAngle in encoder values
            // 8.4667 encoder ticks in one rotation in place (number obtained experimentally)
            absoluteTargetAngle = targetAngle * 8.85 + startingAngle; 
            
            // initiate error values greater than 0 to correct for initial drift
            if (targetAngle >= 0) {
                errorBottomLeft, errorTopRight, errorBottomRight = 0.2;
            } else {
                errorBottomLeft, errorTopRight, errorBottomRight = -0.2;
            }

            // rotate until robot pivots to the given angle
            while (fabs(absoluteTargetAngle - rotationalAngleSoFar) >= rotationalPrecisionThreshold) {
                
                /*
                // TEMP
                std::ostringstream sstream;
                sstream << fabs(absoluteTargetAngle - rotationalAngleSoFar);
                std::string printStatement = sstream.str();
                runPrint(printStatement);
                */
                
                // if target is positive, spin counter clockwise. if negative, spin clockwise
                if (targetAngle >= 0) {
                    
                    baseTopLeftMotor.spin(forwardDirection, percentSpeed * 100, percentVelocityUnit);
                    baseBottomLeftMotor.spin(forwardDirection, percentSpeed * 100 + errorBottomLeft * kpRotational, percentVelocityUnit);

                    baseTopRightMotor.spin(forwardDirection, percentSpeed * 100 + errorTopRight * kpRotational, percentVelocityUnit);
                    baseBottomRightMotor.spin(forwardDirection, percentSpeed * 100 + errorBottomRight * kpRotational, percentVelocityUnit);
                    
                } else {
                    
                    baseTopLeftMotor.spin(reverseDirection, percentSpeed * 100, percentVelocityUnit);
                    baseBottomLeftMotor.spin(reverseDirection, percentSpeed * 100 - errorBottomLeft * kpRotational, percentVelocityUnit);

                    baseTopRightMotor.spin(reverseDirection, percentSpeed * 100 - errorTopRight * kpRotational, percentVelocityUnit);
                    baseBottomRightMotor.spin(reverseDirection, percentSpeed * 100 - errorBottomRight * kpRotational, percentVelocityUnit);
                    
                }
                
                // to correct for differing speeds on each wheel, calculate the error of each encoder relative to the top left wheel and adjust speeds accordingly
                rotationalAngleSoFar = baseTopLeftMotor.rotation(degreesUnit);
                errorBottomLeft = rotationalAngleSoFar - baseBottomLeftMotor.rotation(degreesUnit);
                errorTopRight = rotationalAngleSoFar - baseTopRightMotor.rotation(degreesUnit);
                errorBottomRight = rotationalAngleSoFar - baseBottomRightMotor.rotation(degreesUnit);
                
                /*
                // TEMP
                std::ostringstream sstream;
                sstream << absoluteTargetAngle << " " << rotationalAngleSoFar;
                std::string printStatement = sstream.str();
                runPrint(printStatement);
                */
            }
            
            baseTopLeftMotor.stop(vex::brakeType::brake);
            baseTopRightMotor.stop(vex::brakeType::brake);
            baseBottomLeftMotor.stop(vex::brakeType::brake);
            baseBottomRightMotor.stop(vex::brakeType::brake);
        }
    
        /* ------------------------------------------------------------------------
        * Function: armPivot
        * Desc: arm pivot function for driver control
        * Param: 
        *   -true for up pivot, false for down pivot
        *   -speed to apply to motors in percent of motor speed ranging 0-1
        * Output: pivots robot arm up and down
        */
        void armPivot(bool upOrDown, double percentSpeed) {
            // update armPivot angle
            armPivotCurrentAngle = armPivotMotor.rotation(degreesUnit);
            
            // hold arm steady if function argument speed is 0 
            if (percentSpeed == 0) {
                armPivotMotor.stop(vex::brakeType::hold);
            }
            // else if armPivot position is at the maximum or minimums, only allow movement in the opposite direction
            else if (armPivotCurrentAngle >= armPivotUpperAngle) {
                if (upOrDown) {
                    //runPrint("a");
                    armPivotMotor.stop(vex::brakeType::hold);
                } else {
                    //runPrint("b");
                    armPivotMotor.spin(reverseDirection, percentSpeed * 100, percentVelocityUnit);
                }
            }
            else if (armPivotCurrentAngle <= armPivotLowerAngle) {
                
                if (upOrDown) {
                    //runPrint("c");
                    armPivotMotor.spin(forwardDirection, percentSpeed * 100, percentVelocityUnit);
                } else {
                    //runPrint("d");
                    armPivotMotor.stop(vex::brakeType::hold);
                }
            }
            else {
                if (upOrDown) {
                    //runPrint("e");
                    armPivotMotor.spin(forwardDirection, percentSpeed * 100, percentVelocityUnit);
                } else {
                    //runPrint("f");
                    armPivotMotor.spin(reverseDirection, percentSpeed * 100, percentVelocityUnit);
                }
            }
        };
    
        /* ------------------------------------------------------------------------
        * Function: rampLiftUntil (version of rampLift function for AUTONOMOUS)
        * Desc: ramp lifting function to place stack down for AUTONOMOUS
        * Param:
        *   -true for forward / placing mode, false for retracted / stacking mode
        *   -speed to apply to motors in percent of motor speed ranging 0-1
        * Output: activates or deactivates ramp lift
        */
        void armPivotUntil(bool upOrDown, double percentSpeed) {
            // update ramp lift angle
            armPivotCurrentAngle = armPivotMotor.rotation(degreesUnit);
            
            // hold arm steady if function argument speed is 0. Else, move ramp lift forward or back until it reaches its maximum or minimum
            if (percentSpeed == 0) {
                armPivotMotor.stop(vex::brakeType::hold);
            } else {
                if (upOrDown) {
                    while(armPivotCurrentAngle < armPivotUpperAngle) {
                        armPivotCurrentAngle = armPivotMotor.rotation(degreesUnit);
                        armPivotMotor.spin(forwardDirection, percentSpeed * 100, percentVelocityUnit);
                    }
                    armPivotMotor.stop(vex::brakeType::hold);
                } else {
                    while(armPivotCurrentAngle > armPivotLowerAngle) {
                        armPivotCurrentAngle = armPivotMotor.rotation(degreesUnit);
                        armPivotMotor.spin(reverseDirection, percentSpeed * 100, percentVelocityUnit);
                    }
                    armPivotMotor.stop(vex::brakeType::hold);
                }
            }
        };
    
        /* ------------------------------------------------------------------------
        * Function: intakeSpin
        * Desc: intake spinning function
        * Param: 
        *   -true for spin inwards (intake mode), false for spin outwards (output mode)
        *   -speed to apply to motors in percent of motor speed ranging 0-1
        * Output: starts and stops intake
        */
        void intakeSpin(bool inOrOut, double percentSpeed) {
            //if speed is 0, stop and hold motors
            if (percentSpeed == 0) {
                leftIntakeMotor.stop(vex::brakeType::hold);
                rightIntakeMotor.stop(vex::brakeType::hold);
            }
            // if in
            if (inOrOut) {
                leftIntakeMotor.spin(reverseDirection, percentSpeed * 100, percentVelocityUnit);
                rightIntakeMotor.spin(forwardDirection, percentSpeed * 100, percentVelocityUnit);
            } 
            // else out
            else {
                leftIntakeMotor.spin(forwardDirection, percentSpeed * 100, percentVelocityUnit);
                rightIntakeMotor.spin(reverseDirection, percentSpeed * 100, percentVelocityUnit);
            };
        };
    
        /* ------------------------------------------------------------------------
        * Function: rampLift
        * Desc: ramp lifting function to place stack down
        * Param:
        *   -true for forward / placing mode, false for retracted / stacking mode
        *   -speed to apply to motors in percent of motor speed ranging 0-1
        * Output: activates or deactivates ramp lift
        */
        void rampLift(bool forwardOrBack, double percentSpeed) {
            // update ramp lift angle
            rampLiftCurrentAngle = rampLiftMotor.rotation(degreesUnit);
            
            //calculate ramp lift speed
            rampLiftSpeed = percentSpeed * 100; //(1000/(percentSpeed * 100 + 1)) + 5;
            
            // hold arm steady if function argument speed is 0 
            if (percentSpeed == 0) {
                rampLiftMotor.stop(vex::brakeType::hold);
            }
            // else if ramp lift position is at the maximum or minimums, only allow movement in the opposite direction
            else if (rampLiftCurrentAngle >= rampLiftUpperAngle) {
                if (!forwardOrBack) {
                    // lift up
                    rampLiftMotor.spin(reverseDirection, rampLiftSpeed, percentVelocityUnit);
                } else {
                    rampLiftMotor.stop(vex::brakeType::hold);
                }
            }
            else if (rampLiftCurrentAngle <= rampLiftLowerAngle) {
                if (forwardOrBack) {
                    // move back
                    rampLiftMotor.spin(forwardDirection, percentSpeed * 100, percentVelocityUnit);
                } else {
                    rampLiftMotor.stop(vex::brakeType::hold);
                }
            }
            else {
                if (forwardOrBack) {
                    //lift up
                    rampLiftMotor.spin(forwardDirection, rampLiftSpeed, percentVelocityUnit);
                } else {
                    //move back
                    rampLiftMotor.spin(reverseDirection, percentSpeed * 100, percentVelocityUnit);
                }
            }
        };
    
        /* ------------------------------------------------------------------------
        * Function: rampLiftByVelocity
        * Desc: ramp lifting function to place stack down with motor power throttled to velocity
        * Param:
        *   -true for forward / placing mode, false for retracted / stacking mode
        *   -speed to apply to motors in percent of motor speed ranging 0-1
        * Output: activates or deactivates ramp lift
        */
        void rampLiftByVelocity (bool forwardOrBack) {
            // update ramp lift angle
            rampLiftCurrentAngle = rampLiftMotor.rotation(degreesUnit);

            // if ramp lift position is at the maximum or minimums, only allow movement in the opposite direction
            if (rampLiftCurrentAngle >= rampLiftUpperAngle) {
                if (forwardOrBack) {
                    // lift up
                    rampLiftMotor.setVelocity(-50, percentVelocityUnit);
                } else {
                    rampLiftMotor.stop(vex::brakeType::hold);
                }
            }
            else if (rampLiftCurrentAngle <= rampLiftLowerAngle) {
                if (!forwardOrBack) {
                    // move back
                    rampLiftMotor.setVelocity(10, percentVelocityUnit);
                } else {
                    rampLiftMotor.stop(vex::brakeType::hold);
                }
            }
            else {
                if (forwardOrBack) {
                    //lift up
                    rampLiftMotor.setVelocity(-50, percentVelocityUnit);
                } else {
                    //move back
                    rampLiftMotor.setVelocity(10, percentVelocityUnit);
                }
            }
        }
    
        /* ------------------------------------------------------------------------
        * Function: rampLiftUntil (version of rampLift function for AUTONOMOUS)
        * Desc: ramp lifting function to place stack down for AUTONOMOUS
        * Param:
        *   -true for forward / placing mode, false for retracted / stacking mode
        *   -speed to apply to motors in percent of motor speed ranging 0-1
        * Output: activates or deactivates ramp lift
        */
        void rampLiftUntil(bool placeOrRetract, double percentSpeed) {
            // update ramp lift angle
            rampLiftCurrentAngle = rampLiftMotor.rotation(degreesUnit);
            
            // hold arm steady if function argument speed is 0. Else, move ramp lift forward or back until it reaches its maximum or minimum
            if (percentSpeed == 0) {
                rampLiftMotor.stop(vex::brakeType::hold);
            } else {
                if (placeOrRetract) {
                    while(rampLiftCurrentAngle <= rampLiftUpperAngle) {
                        rampLiftCurrentAngle = rampLiftMotor.rotation(degreesUnit);
                        rampLiftMotor.spin(forwardDirection, percentSpeed * 100, percentVelocityUnit);
                    }
                    rampLiftMotor.stop(vex::brakeType::hold);
                } else {
                    while(rampLiftCurrentAngle >= rampLiftLowerAngle) {
                        rampLiftCurrentAngle = rampLiftMotor.rotation(degreesUnit);
                        rampLiftMotor.spin(reverseDirection, percentSpeed * 100, percentVelocityUnit);
                    }
                    rampLiftMotor.stop(vex::brakeType::hold);
                }
            }
        };

        /* ------------------------------------------------------------------------
        * Function: controlledToweringMovement
        * Desc: Moves robot and sets intake out speed to match the movement speed, 
        *   that way the stacked tower does not get dragged along with the robot
        * Param:
        *   - speed to back up at in percent, ranging from -1 to 1 (positive for forward, negative for backward)
        * Output: performs a controlled movement while coordinating intake speed, used after a tower has been stacked in the goal
        */
        void controlledToweringMovement(double percentSpeed) {
            if (percentSpeed == 0) { // stop and do nothing if percentSpeed is 0
                baseMove(0,0);
                intakeSpin(true,0);
            } 

            // move robot and calculate and coordinate intake to move at the opposite speed so that the tower isn't dragged along
            // intake speed is based on ratio of intake circumference to wheel circumference, and by poportionality, the speed of each

            else if (percentSpeed < 0) { // forward movement
                baseMove(percentSpeed * 100, 0); 
                intakeSpin(true, percentSpeed * 2 * (intakeCircumference/wheelCircumference));
            }
            else { // backwards movement
              baseMove(percentSpeed * 100, 0); 
              intakeSpin(true, percentSpeed * 2 * (intakeCircumference/wheelCircumference));
            }
        }
    
        /* ------------------------------------------------------------------------
        * Function: autonomousSelector
        * Desc: Allows user to SELECT which AUTONOMOUS ROUTINE to use during PRE-AUTONOMOUS
        * Param:
        * Output: Selects which autonomous routine to use
        *\
        void autonomousSelector() {
            // Create screen buttons
            ScreenButton* blueFrontButton = new ScreenButton(100,100,100,50,"#FFFFFF","Blue Front", 1);
            ScreenButton* blueBackButton = new ScreenButton(100,100,100,50,"#FFFFFF","Blue Back", 1);
            ScreenButton* redFrontButton = new ScreenButton(100,100,100,50,"#FFFFFF","Red Front", 1);
            ScreenButton* redBackButton = new ScreenButton(100,100,100,50,"#FFFFFF","Red Back", 1);
            
            // Check for cursor inputs
            autonomousSelected = false;
            while (!autonomousSelected) {
                cursorPressedCheck();
            }
            
        } 
        void cursorPressedCheck() {
            int selection = ScreenButton::whichButtonPressed(Brain.Screen.xPosition(), Brain.Screen.yPosition());
            if (selection != 0) {
                autonomousSelected = true;
                autonomousSelection = selection;
            }
        }*/
    
        
    public:
        // Constructor, aka Pre-Autonomous
        Robot() {
            //autonomousSelector();
            
            // set up ramp lift angles according to ramp lift starting position
            rampLiftCurrentAngle = rampLiftMotor.rotation(degreesUnit);
            rampLiftUpperAngle = rampLiftCurrentAngle + rampLiftUpperAngle;
            rampLiftLowerAngle = rampLiftCurrentAngle + rampLiftLowerAngle;
            
            // set up arm pivot angles according to arm pivot starting position
            armPivotCurrentAngle = armPivotMotor.rotation(degreesUnit);
            armPivotUpperAngle = armPivotCurrentAngle + armPivotUpperAngle;
            armPivotLowerAngle = armPivotCurrentAngle + armPivotLowerAngle;

            rampLiftMotor.setMaxTorque(100,percentUnit);
            
            baseTopLeftMotor.setStopping(vex::brakeType::brake);
            baseTopRightMotor.setStopping(vex::brakeType::brake);
            baseBottomLeftMotor.setStopping(vex::brakeType::brake);
            baseBottomRightMotor.setStopping(vex::brakeType::brake);
        };
    
        /* ------------------------------------------------------------------------
        * Function: autonomousMain
        * Desc: Runs autonomous procedure
        * Param: none
        * Output: Moves robot according to preprogrammed autonomous procedure
        */
        void autonomousMain( int routineNumber ) {
            runPrint("Started autonomousMain", 1);
            
            /*
            * use linearMove(distance in meters, percent power from 0-1) for forward/backwards movement
            * use rotationalMove(rotation in degrees, percent power from 0-1) for rotating or pivoting in place
            */
            
            /*
            * Flip Out / Initation
            */
            intakeSpin(false, 1); // spin out intake
            vex::task::sleep(1000); // wait 1s
            intakeSpin(true, 0); // stop intake
            rampLift(true, 1); // ramp forward
            vex::task::sleep(700);
            rampLiftUntil(false,1); // ramp back
            //rampLiftUntil(true, 1); // ramp forward
            //rampLiftUntil(false, 1); // ramp back
            intakeSpin(true,1); // spin in intake
            
            /*
            * Routine Specific Movements
            */
            
            switch(routineNumber) {
                case 1: {
                        // Red Front
                        linearMove(1.2, 0.6); // fwd 1.2m
                        vex::task::sleep(200);
                        rotationalMove(90, 0.2); // cw 90 deg
                        vex::task::sleep(200);
                        linearMove(0.6, 0.6); // fwd 0.6m
                        vex::task::sleep(200);
                        rotationalMove(90, 0.2); // cw 90 deg
                        vex::task::sleep(200);
                        linearMove(0.9, 0.6); // fwd 1m
                        vex::task::sleep(200);
                        rotationalMove(-45, 0.2); // cw 40 deg
                        vex::task::sleep(200);
                        linearMove(0.6, 0.5); //fwd 0.5m
                        vex::task::sleep(200);
                        break;
                    
                    
                        /*linearMove(0.9, 0.7); // fwd 0.9m
                        linearMove(0.1,0.1); // fwd 0.1m
                        linearMove(-0.3,0.5); // back 0.4
                        rotationalMove(-45, 0.2); // ccw 45 deg
                        linearMove(-0.6, 0.5); // back 0.4m
                        rotationalMove(45, 0.2); // cw 45 deg
                        linearMove(0.7, 0.7); // fwd 0.8m
                        linearMove(0.1, 0.1); // fwd 0.1m
                        linearMove(-0.6,0.5); // back 0.5m
                        rotationalMove(135,0.2); // cw 135 deg
                        linearMove(0.5, 0.5); //fwd 0.4m
                        break;*/
                    };
                case 2: { // DO NOT USE
                        // Red Back
                        linearMove(1.1, 0.5); // fwd 1.1m
                        rotationalMove(-90, 0.2); // ccw 90 deg
                        linearMove(0.6, 0.5); // fwd 0.5m
                        rotationalMove(-90, 0.2); // ccw 90 deg
                        linearMove(0.6, 0.5); // fwd 0.5m
                        rotationalMove(-60, 0.2); // ccw 70 deg
                        linearMove(0.88, 0.5); //fwd 0.88m
                        break;
                    };
                case 3: {
                        // Blue Front
                        linearMove(1.2, 0.6); // fwd 1.2m
                        vex::task::sleep(200);
                        rotationalMove(-90, 0.2); // cw 90 deg
                        vex::task::sleep(200);
                        linearMove(0.6, 0.6); // fwd 0.6m
                        vex::task::sleep(200);
                        rotationalMove(-90, 0.2); // cw 90 deg
                        vex::task::sleep(200);
                        linearMove(0.9, 0.6); // fwd 1m
                        vex::task::sleep(200);
                        rotationalMove(45, 0.2); // cw 40 deg
                        vex::task::sleep(200);
                        linearMove(0.6, 0.5); //fwd 0.5m
                        vex::task::sleep(200);
                        break;
                    };
                case 4: { // DO NOT USE
                        // Blue Back
                        linearMove(1.1, 0.5); // fwd 1.1m
                        rotationalMove(-90, 0.2); // ccw 90 deg
                        linearMove(0.6, 0.5); // fwd 0.5m
                        rotationalMove(-90, 0.2); // ccw 90 deg
                        linearMove(0.6, 0.5); // fwd 0.5m
                        rotationalMove(-60, 0.2); // ccw 70 deg
                        linearMove(0.88, 0.5); //fwd 0.88m
                        break;
                    };
            };

            /*
            * Release a single cube into the goal and then back up
            */
            intakeSpin(false,0.5); // intake spin out
            vex::task::sleep(850);
            intakeSpin(true,0); // stop intake
            vex::task::sleep(200);
            linearMove(-0.5,0.5); // back 0.5m
            vex::task::sleep(1000); // wait until auton period ends

            /*
            * Place Stack
            *//*
            intakeSpin(false, 0.2); // lower cubes slightly
            vex::task::sleep(200);
            intakeSpin(true,0); // stop intake
            rampLiftUntil(true,1); // ramp forward
            linearMove(0.05,0.1); // bump it forward a bit
            vex::task::sleep(200);
            rampLiftUntil(false,1);
            controlledToweringMovement(-0.2);
            vex::task::sleep(5000); // wait 5 seconds
            */
        };
    
        /* ------------------------------------------------------------------------
        * DRIVER CONTROLS
        *
        * Stick 3:
        *   - Linear Forward / Linear Backwards Movement
        *
        * Stick 1:
        *   - Rotate Left / Rotate Right Movement
        *
        * Button X / B:
        *   - Controlled Towering Movement Forwards / Backwards
        *
        * Top Buttons R2 / R1:
        *   - Intake Spin Out / In
        *   - If neither are pressed, stop motors with hold brake type
        *
        * Top Buttons L1 / L2:
        *   - Ramp Lift Forward / Backwards
        *   - If neither are pressed, stop motors with hold brake type
        *
        *
        *
        * Button "UP" / "DOWN":
        *   - Ramp Lift Forward (place stack) / Ramp Lift Back (deactivate and angle back)
        *
        * ------------------------------------------------------------------------
        */
        /*
        * Function: driverMain
        * Desc: Sets up driver controls and checks continuously during driver period to translate to controller inputs
        * Param: none
        * Output: Allows driver to control robot
        */
        void driverMain( void ) {
            //runPrint("Started driverMain", 1);
            
            // CONTROLLER VALUES
            double stick3;
            double stick1;

            bool buttonR2;
            bool buttonR1;
            
            bool buttonL2;
            bool buttonL1;

            bool buttonX;
            bool buttonB;
            bool buttonA;

            bool buttonUp;
            bool buttonDown;

            // continuously check for inputs and translate to robot movement
            while(true) {
                /*
                * UPDATE CONTROLLER VALUES
                */
                stick3 = Controller.Axis3.position(percentUnit);
                stick1 = Controller.Axis1.position(percentUnit);
                
                buttonR2 = Controller.ButtonR2.pressing();
                buttonR1 = Controller.ButtonR1.pressing();

                buttonL2 = Controller.ButtonL2.pressing();
                buttonL1 = Controller.ButtonL1.pressing();
                
                buttonX = Controller.ButtonX.pressing();
                buttonB = Controller.ButtonB.pressing();
                buttonA = Controller.ButtonA.pressing();
                
                buttonUp = Controller.ButtonUp.pressing();
                buttonDown = Controller.ButtonDown.pressing();
                
                /*
                // TEMP
                std::ostringstream sstream;
                sstream << baseTopLeftMotor.rotation(degreesUnit);
                std::string printStatement = sstream.str();
                runPrint(printStatement);
                */
                
                /*
                * TRANSLATE STICK 3 for FORWARD / BACKWARDS and STICK 4 for ROTATE Motion
                */
                // move base only when sticks are not in neutral (when they become outside of movement threshold range)
                if ( (stick3 > movementThreshold || stick3 < -movementThreshold) || (stick1 > movementThreshold || stick1 < -movementThreshold)) {
                    
                    // calculate the cubed value of the linear stick, dampening sensitivity near the center and increasing sensitivity at the extremes
                    stick3 = stick3 * fabs(stick3) / 100;
                    
                    baseMove(stick3 * 0.6, stick1 * 0.2);
                } 

                // Do not allow Controlled Towering Movement if the sticks are being used for normal movement

                    /*
                    * TRANSLATE BUTTONS X / B for CONTROLLED TOWERING MOVEMENT FORWARD / BACKWARDS motion
                    * Makes intakes move at the according to robot movement in order to move without dragging the stacked tower
                    */
                    else if (buttonX) {
                        controlledToweringMovement(-0.35);
                    }
                    else if (buttonB) {
                        controlledToweringMovement(0.35);
                    }
                    else { // if neither are pressed, stop intake and stop movements
                        baseMove(0,0);
                    }
                
                // Do not allow intake controls if either X and B are being used
                /*
                * TRANSLATE BUTTONS R1 / R2 for INTAKE SPIN IN / INTAKE SPIN OUT motion
                */
                if (!buttonX && !buttonB) {
                    if (buttonR1) {
                        intakeSpin(true, 1); // in spin
                    }
                    else if (buttonR2) {
                        intakeSpin(false, 0.2); // in spin slow
                    }
                    else if (buttonA) {
                        intakeSpin(false,1); // out spin fast
                    }
                    else {
                        intakeSpin(true,0); // stop
                    }
                }
                
                
                /*
                * TRANSLATE BUTTONS UP / DOWN for ARM UP / ARM DOWN motion
                */
                if (buttonUp) {
                    armPivot(true, 1);
                }
                else if (buttonDown) {
                    armPivot(false, 1);
                }
                // if both buttons aren't pressed
                else{
                    if(fabs(armPivotMotor.velocity(percentVelocityUnit)) >= 2) {
                        armPivot(true, 0);
                    }
                }
                
                
                
                /*
                * TRANSLATE BUTTON L1 / L2 for RAMP LIFT FORWARD / BACK motion
                */
                if (buttonL1) {
                    rampLift(true, 0.7);
                }
                else if(buttonL2) {
                    rampLift(false, 1);
                }
                else {
                    if(fabs(rampLiftMotor.velocity(percentVelocityUnit)) >= 2) {
                        rampLift(true, 0);
                    }
                }
                
                //runPrint("Controller Updater Looped");
                // TEMP
                std::ostringstream sstream;
                sstream << rampLiftMotor.rotation(degreesUnit);;
                std::string printStatement = sstream.str();
                runPrint(printStatement);
            }
        }
};

/*
* Required main() method. Point of origin for code execution.
*/ 

Robot* robot = new Robot();

void competitionAutonomous( void ) {
    robot->autonomousMain(1);
}

void competitionDriver( void ) {
    robot->driverMain();
    delete robot;
}

int main() {
    
    Competition.autonomous(competitionAutonomous);
    Competition.drivercontrol(competitionDriver);
    
    //competitionAutonomous();
    //competitionDriver();
        
    while (1) {
        vex::task::sleep(100);
    }
}