/*
* ------------------------------------------------------------------------
* Project: xray-bougie-v7.4
* Author: Shaw-Sean Yang
* Date: 1/23/2020
* Desc: Allows the driver to manually control the xdrive robot using the controller, or preprogram the robot to travel specific distances.
* Version: Ports remapped to X-Ray Bot IV: Bougie
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
        double armPivotUpperAngle = 600; // must be >= 0
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
    
        double armPivotIncrementalPercents [3] = {0, 0.8, 1}; // incremental percents used for incrementArmPivot function
        int currentArmIncrement = 0;
        double armPivotThreshold = 10;
    
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
            
        };
        
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
            absoluteTargetAngle = targetAngle * 7.8 + startingAngle; 
            
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
                if (absoluteTargetAngle - rotationalAngleSoFar > rotationalPrecisionThreshold) {
                    
                    baseTopLeftMotor.spin(forwardDirection, percentSpeed * 100, percentVelocityUnit);
                    baseBottomLeftMotor.spin(forwardDirection, percentSpeed * 100 + errorBottomLeft * kpRotational, percentVelocityUnit);

                    baseTopRightMotor.spin(forwardDirection, percentSpeed * 100 + errorTopRight * kpRotational, percentVelocityUnit);
                    baseBottomRightMotor.spin(forwardDirection, percentSpeed * 100 + errorBottomRight * kpRotational, percentVelocityUnit);
                    
                } else if (absoluteTargetAngle - rotationalAngleSoFar < rotationalPrecisionThreshold) {
                    
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
                
                
                // TEMP
                std::ostringstream sstream;
                sstream << fabs(absoluteTargetAngle - rotationalAngleSoFar);
                std::string printStatement = sstream.str();
                runPrint(printStatement);
                
            }
            
            baseTopLeftMotor.stop(vex::brakeType::brake);
            baseTopRightMotor.stop(vex::brakeType::brake);
            baseBottomLeftMotor.stop(vex::brakeType::brake);
            baseBottomRightMotor.stop(vex::brakeType::brake);
        };
    
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
                    runPrint("a");
                    armPivotMotor.stop(vex::brakeType::hold);
                } else {
                    runPrint("b");
                    armPivotMotor.spin(reverseDirection, percentSpeed * 100, percentVelocityUnit);
                }
            }
            else if (armPivotCurrentAngle <= armPivotLowerAngle) {
                
                if (upOrDown) {
                    runPrint("c");
                    armPivotMotor.spin(forwardDirection, percentSpeed * 100, percentVelocityUnit);
                } else {
                    runPrint("d");
                    armPivotMotor.stop(vex::brakeType::hold);
                }
            }
            else {
                if (upOrDown) {
                    runPrint("e");
                    armPivotMotor.spin(forwardDirection, percentSpeed * 100, percentVelocityUnit);
                } else {
                    runPrint("f");
                    armPivotMotor.spin(reverseDirection, percentSpeed * 100, percentVelocityUnit);
                }
            }
        };
    
        /* ------------------------------------------------------------------------
        * Function: armPivotUntilPercent (version of rampLift function with while loop)
        * Desc: pivot arm until given percentage of the arm movement range
        * Param:
        *   -percent between [0, 1] of the arm movement range
        *   -speed to apply to motors in percent of motor speed ranging [0, 1]
        * Output: moves arm up and down
        */
        void armPivotUntilPercent(double untilPercentage, double percentSpeed) {
            // calculate and update current and target angles
            double currentAngle = armPivotMotor.rotation(degreesUnit);
            double targetAngle = untilPercentage * (armPivotUpperAngle - armPivotLowerAngle);
            
            // Hold arm still if argument speed is 0 or if target angle is the same as the current angle
            if (percentSpeed == 0 || currentAngle == targetAngle) {
                armPivotMotor.stop(vex::brakeType::hold);
                
            } else {
                bool upOrDown = (currentAngle <= targetAngle) ? true : false; // if current angle less than target, set boolean to "go up"
                    
                if (upOrDown) {
                    while(currentAngle < targetAngle) {
                        currentAngle = armPivotMotor.rotation(degreesUnit);
                        armPivotMotor.spin(forwardDirection, percentSpeed * 100, percentVelocityUnit);
                    }                    
                    armPivotMotor.stop(vex::brakeType::hold);
                } else {
                    while(currentAngle > targetAngle) {
                        currentAngle = armPivotMotor.rotation(degreesUnit);
                        armPivotMotor.spin(reverseDirection, percentSpeed * 100, percentVelocityUnit);
                    }
                    armPivotMotor.stop(vex::brakeType::hold);
                }
            }
        };

        /* ------------------------------------------------------------------------
        * Function: armPivotToPercent (version of rampLift function with ifs)
        * Desc: pivot arm until given percentage of the arm movement range
        * Param:
        *   -percent between [0, 1] of the arm movement range
        *   -speed to apply to motors in percent of motor speed ranging [0, 1]
        * Output: moves arm up and down
        */
        void armPivotToPercent(double toPercentage, double percentSpeed) {
            // calculate and update current and target angles
            double targetAngle = toPercentage * (armPivotUpperAngle - armPivotLowerAngle);

            armPivotCurrentAngle = armPivotMotor.rotation(degreesUnit);
            
            // Hold arm still if argument speed is 0 or if target angle is near the current angle
            if (percentSpeed == 0 || fabs(armPivotCurrentAngle - targetAngle) <= armPivotThreshold) {
                armPivotMotor.stop(vex::brakeType::hold);
                
            } else {
                // if target is above, set to Up, if target is below, set to Below
                bool upOrDown;
                if (armPivotCurrentAngle <= targetAngle) {
                  upOrDown = true;
                } else {
                  upOrDown = false;
                }
                
                if (armPivotCurrentAngle >= targetAngle) {
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
            }
        };
    
        /* ------------------------------------------------------------------------
        * Function: incrementArmPivot
        * Desc: pivot arm until given pre-defined increment
        * Param:
        *   - true for up an increment, false for down an increment
        *   - increment integer [0, 2]. Use the absolute value of the desired increment EX: incrementArmPivot( false , 1 ); for down one increment
        * Output: moves arm up and down
        */
        void incrementArmPivot(bool upOrDown, int increment) {
            
            if (upOrDown && currentArmIncrement < 2) { // if up and current increment less than 2
                
                currentArmIncrement = currentArmIncrement + increment;
                
                // move arm at full speed until predefined target angle based on increment
                armPivotUntilPercent(armPivotIncrementalPercents[currentArmIncrement], 1);
                
            } else if (!upOrDown && currentArmIncrement > 0) { // if down and current increment less than 0
                
                currentArmIncrement = currentArmIncrement - increment;
                
                // move arm at full speed until predefined target angle based on increment
                armPivotUntilPercent(armPivotIncrementalPercents[currentArmIncrement], 1);
                
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
                leftIntakeMotor.spin(forwardDirection, percentSpeed * 100, percentVelocityUnit);
                rightIntakeMotor.spin(reverseDirection, percentSpeed * 100, percentVelocityUnit);
            } 
            // else out
            else {
                leftIntakeMotor.spin(reverseDirection, percentSpeed * 100, percentVelocityUnit);
                rightIntakeMotor.spin(forwardDirection, percentSpeed * 100, percentVelocityUnit);
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
        };
    
        /* ------------------------------------------------------------------------
        * Function: rampLiftUntilExtrema (version of rampLift function for AUTONOMOUS)
        * Desc: ramp lifting function to place stack down for AUTONOMOUS
        * Param:
        *   -true for forward / placing mode, false for retracted / stacking mode
        *   -speed to apply to motors in percent of motor speed ranging 0-1
        * Output: activates or deactivates ramp lift
        */
        void rampLiftUntilExtrema(bool placeOrRetract, double percentSpeed) {
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
            
            //rotationalMove(-90,0.2);

            /*
            * Routine Specific Movements
            */
            
            switch(routineNumber) {
                case 1: {
                        // RED FRONT
                        
                        // Flip Out / Initation
                        armPivotUntilPercent(0.4, 1);
                        armPivotUntilPercent(0, 1);
                        intakeSpin(true,1); // spin in intake
                        
                        // Pick up 4 cubes
                        linearMove(1.2, 0.2);
                        vex::task::sleep(200);
                        rotationalMove(180, 0.2);
                        vex::task::sleep(200);
                        linearMove(0.8, 0.7);
                        vex::task::sleep(200);
                        rotationalMove(-45, 0.2); // cw 45 deg
                        vex::task::sleep(200);
                        linearMove(0.425, 0.4); //fwd 0.45m
                        vex::task::sleep(200);
                        
                        // Place Stack
                        intakeSpin(false, 0.4); // stop intakes
                        vex::task::sleep(1000);
                        intakeSpin(true, 0);
                        rampLiftUntilExtrema(true, 0.7); // ramp forward
                        intakeSpin(false, 0.1); // slow outtake
                        linearMove(-0.6, 0.3);
                        vex::task::sleep(5000); // wait 5 seconds
                        
                        break;
                    };
                case 2: {
                        // RED BACK


                        // Flip out ramp
                        armPivotUntilPercent(0.4, 1);
                        armPivotUntilPercent(0, 1);

                        /*intakeSpin(false, 1); // spin out intake
                        vex::task::sleep(500); // wait 0.5s*/
                        intakeSpin(true, 1); // spin in intake

                        // Put starter block in tower
                        linearMove(0.1, 0.5);
                        vex::task::sleep(300);
                        intakeSpin(true,0);
                        armPivotUntilPercent(armPivotIncrementalPercents[2], 1);
                        rotationalMove(45, 0.4);
                        vex::task::sleep(200);
                        linearMove(0.25, 0.7);
                        intakeSpin(false, 1);
                        vex::task::sleep(1000);
                        intakeSpin(true,0);
                        linearMove(-0.25, 0.7);
                        armPivotUntilPercent(armPivotIncrementalPercents[0], 1);
                        
                        // Move to goal
                        intakeSpin(true, 1);
                        rotationalMove(-60, 0.4);
                        vex::task::sleep(200);
                        linearMove(0.3, 0.7);
                        vex::task::sleep(200);
                        rotationalMove(-50, 0.4);
                        vex::task::sleep(200);
                        linearMove(0.6, 0.7);
                        vex::task::sleep(200);
                        rotationalMove(-45, 0.4);
                        vex::task::sleep(200);
                        linearMove(0.4, 1);
                        vex::task::sleep(200);

                        // spit out cubes
                        intakeSpin(false, 1);
                        vex::task::sleep(1500);
                        linearMove(-0.3, 1);

                        break;
                    };
                case 3: {
                        // BLUE FRONT
                        
                        // Flip Out / Initation
                        armPivotUntilPercent(0.4, 1);
                        armPivotUntilPercent(0, 1);
                        intakeSpin(true,1); // spin in intake
                        
                        // Pick up 4 cubes
                        linearMove(1.2, 0.2);
                        vex::task::sleep(200);
                        rotationalMove(-180, 0.2);
                        vex::task::sleep(200);
                        linearMove(0.8, 0.7);
                        vex::task::sleep(200);
                        rotationalMove(45, 0.2); // cw 45 deg
                        vex::task::sleep(200);
                        linearMove(0.425, 0.4); //fwd 0.45m
                        vex::task::sleep(200);
                        
                        // Place Stack
                        intakeSpin(false, 0.4); // stop intakes
                        vex::task::sleep(1000);
                        intakeSpin(true, 0);
                        rampLiftUntilExtrema(true, 0.7); // ramp forward
                        intakeSpin(false, 0.1); // slow outtake
                        linearMove(-0.6, 0.3);
                        vex::task::sleep(5000); // wait 5 seconds
                        
                        break;

                    };
                case 4: {
                        // BLUE BACK


                        // Flip out ramp
                        armPivotUntilPercent(0.4, 1);
                        armPivotUntilPercent(0, 1);
                        
                        // Spin in intakes
                        intakeSpin(true, 1);

                        // Put starter block in tower
                        linearMove(0.1, 0.5);
                        vex::task::sleep(300);
                        intakeSpin(true,0);
                        armPivotUntilPercent(armPivotIncrementalPercents[2], 1);
                        rotationalMove(-45, 0.2);
                        vex::task::sleep(200);
                        linearMove(0.25, 0.7);
                        intakeSpin(false, 1);
                        vex::task::sleep(1000);
                        intakeSpin(true,0);
                        linearMove(-0.2, 0.6);
                        armPivotUntilPercent(armPivotIncrementalPercents[0], 1);
                        vex::task::sleep(200);

                        // Move to goal
                        intakeSpin(true, 1);
                        rotationalMove(65, 0.2);
                        vex::task::sleep(200);
                        linearMove(0.3, 0.7);
                        vex::task::sleep(200);
                        rotationalMove(50, 0.2);
                        vex::task::sleep(200);
                        linearMove(0.7, 0.6);
                        vex::task::sleep(250);
                        rotationalMove(70, 0.2);
                        vex::task::sleep(200);
                        linearMove(0.4, 0.7);

                        // spit out cubes
                        intakeSpin(false, 1);
                        vex::task::sleep(1500);
                        linearMove(-0.2, 1);

                        break;
                    };
                case 5: {
                  // ONE MIN AUTONOMOUS SKILLS

                  // BLUE BACK


                  // Flip out ramp
                  armPivotUntilPercent(0.4, 1);
                  armPivotUntilPercent(0, 1);
                  
                  // Spin in intakes
                  intakeSpin(true, 1);

                  // Put starter block in tower
                  linearMove(0.1, 0.5);
                  vex::task::sleep(300);
                  intakeSpin(true,0);
                  armPivotUntilPercent(armPivotIncrementalPercents[2], 1);
                  rotationalMove(-45, 0.2);
                  vex::task::sleep(200);
                  linearMove(0.25, 0.7);
                  intakeSpin(false, 1);
                  vex::task::sleep(1000);
                  intakeSpin(true,0);
                  linearMove(-0.2, 0.6);
                  armPivotUntilPercent(armPivotIncrementalPercents[0], 1);
                  vex::task::sleep(200);
                  intakeSpin(true,1);
                  linearMove(0.2, 0.4);
                  vex::task::sleep(200);
                  linearMove(-0.2, 0.4);

                  // Move to goal
                  rotationalMove(65, 0.2);
                  vex::task::sleep(200);
                  linearMove(0.3, 0.7);
                  vex::task::sleep(200);
                  rotationalMove(50, 0.2);
                  vex::task::sleep(200);
                  linearMove(0.7, 0.6);
                  vex::task::sleep(250);
                  rotationalMove(70, 0.2);
                  vex::task::sleep(200);
                  linearMove(0.4, 0.7);

                  // spit out cubes
                  intakeSpin(false, 1);
                  vex::task::sleep(1500);
                  linearMove(-0.2, 1);

                  break;
                }
            };

            
            
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

            bool buttonUp = false;
            bool buttonRight = false;
            bool buttonDown = false;

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
                buttonRight = Controller.ButtonRight.pressing();
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
                    
                    baseMove(stick3 * 0.6, stick1 * 0.6);
                } 

                // Do not allow Controlled Towering Movement if the sticks are being used for normal movement

                    /*
                    * TRANSLATE BUTTONS X / B for SLOW FORWARD / SLOW BACKWARDS movement
                    */
                    else if (buttonX) {
                        baseMove(10, 0);
                    }
                    else if (buttonB) {
                        baseMove(-10, 0);
                    }
                    else { // if neither are pressed, stop movement
                        baseMove(0,0);
                    }
                
                // Do not allow intake controls if either X and B are being used
                /*
                * TRANSLATE BUTTONS R1 / R2 for INTAKE SPIN IN / INTAKE SPIN OUT motion
                */
                if (!buttonX && !buttonB) {
                    if (buttonR2) {
                        intakeSpin(true, 0.9); // in spin fast
                    }
                    else if (buttonR1) {
                        intakeSpin(false, 0.2); // out spin slow
                    }
                    else if (buttonA) {
                        intakeSpin(false,1); // out spin fast
                    }
                    else {
                        intakeSpin(true,0); // stop
                    }
                }
                
                
                /*
                * TRANSLATE BUTTONS UP / RIGHT / DOWN for ARM UP / ARM DOWN incremental motion
                */
                if (buttonUp) {
                    armPivotToPercent(armPivotIncrementalPercents[2], 1);
                }
                else if (buttonRight) {
                    armPivotToPercent(armPivotIncrementalPercents[1], 1);
                }
                else if (buttonDown) {
                    armPivot(false, 1);
                }
                else {
                  armPivot(true,0);
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
            }
        }
};

/*
* Required main() method. Point of origin for code execution.
*/ 

Robot* robot = new Robot();

void competitionAutonomous( void ) {
    robot->autonomousMain(5); 
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