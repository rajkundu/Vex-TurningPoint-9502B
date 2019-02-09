//Header guard
#pragma once

//----------------------------------------------------------------------------//
//                                Miscellaneous                               //
//----------------------------------------------------------------------------//

//--------- Functions --------//

/**
 * Scales input according to deadband and maximum
 * @param input value to be scaled
 *  - range [-127, 127]
 * @param threshold threshold at/below which to return 0
 *  - range [-127, 127]
 * @return scaled value or 0
 *  - range [-127, 127]
 */
double scaleDeadband(double input, double threshold);

/**
 * Scales input according to exponential curve
 * @param input base
 * @param power power to which input will be raised
 * @param maxValue the maximum value to be reached
 * 
 * @return f(x) according to curve defined by parameters
 */
double expCurve(double input, double power, double maxValue);

//----------------------------------------------------------------------------//
//                                 Drivetrain                                 //
//----------------------------------------------------------------------------//

//---------- Globals ---------//

extern ChassisControllerPID drivetrain;
extern bool slewEnabled;
//Slew rate in [-1, 1] units per second
extern double slewRate_y;
extern double slewRate_r;
extern double y_last;
extern double r_last;

//--------- Functions --------//

/**
 * sets drivetrain velocity
 * @param y desired forward-backward component
 *  - range [-200, 200]
 *  - units RPM
 * @param r desired rotational component
 *  - range [-200, 200]
 *  - units RPM
 * @param preserveProportion whether or not to preserve proportion between y and
 *  r
 *  - default true
 */
void driveRPM(double y, double r, bool preserveProportion = true);

/**
 * sets drivetrain speed using voltage control
 * @param y desired forward-backward component
 *  - range [-127, 127]
 * @param r desired rotational component
 *  - range [-127, 127]
 * @param preserveProportion whether or not to preserve proportion between y and
 *  r
 *  - default true
 */
void driveVoltage(double y, double r, bool preserveProportion = true);

//----------------------------------------------------------------------------//
//                                  Cap Lift                                  //
//----------------------------------------------------------------------------//

//---------- Motors ----------//

extern Motor capLiftMotor;

//---------- Globals ---------//

enum class CAPLIFT_INTERFERENCE_BOUNDS
{
    LOWER = 15,
    UPPER = 40
};
const int CAPLIFT_VOLTAGE_HOLD = 1500;

//--------- Functions --------//

/**
 * gets angle of cap lift relative to home/starting position
 * @return angle of capLiftMotor / gear ratio = angle of cap lift
 */
double getCapLiftPos();

/**
 * returns whether the cap lift is in range of interfering with puncher
 * @return whether cap lift is interfering range
 */
bool capLiftInterfering();

//----------------------------------------------------------------------------//
//                                  Puncher                                   //
//----------------------------------------------------------------------------//

//---------- Motors ----------//

extern Motor puncher;
extern Motor angleAdjuster;

//---------- Globals ---------//

extern int numLaunches;
enum class PuncherAngles
{
    NEAR_HIGH_FLAG = 50,
    NEAR_LOW_FLAG = 77,
    FAR_HIGH_FLAG = 57,
    FAR_LOW_FLAG = 75
};
extern SettledUtil puncherSettledUtil;
extern bool puncherReady;
extern bool puncherReady_last;

//--------- Functions --------//

/**
 * resets puncher home position
 */
void resetPuncher();

/**
 * checks whether puncher has reached target position
 * @return status of puncher SettledUtil
 */
void updatePuncherReady();

/**
 * launches ball by rotating puncher motor 360 degrees
 * @param blocking whether or not to wait for launch to complete
 */
void launch(bool blocking = false);

/**
 * moves puncher motor to end target position
 * @param endPos end target position relative to home position
 *  - units degrees
 * @param blocking whether or not to wait for puncher to reach end target position
 */
void movePuncherTo(int endPos, bool blocking = false);

/**
 * waits for puncher to reach its last-set target position
 */
void waitForPuncherReady();

/**
 * sets angle of puncher angle adjuster arm
 * @param angle the angle at which to set the puncher
 * @param speed the speed at which to run the angle adjuster motor
 *  - units RPM
 *  - default 50 RPM
 * @param blocking whether or not to wait for angle adjuster to reach angle
 *  - default false
 */
void setPuncherAngle(PuncherAngles angle, int speed = 50, bool blocking = false);

/**
 * runs double shot macro
 * @param firstAngle the first angle at which to set the puncher angle
 *  adjuster arm
 *  - units degrees
 * @param secondAngle the second angle at which to set the puncher angle
 *  adjuster arm
 *  - units degrees
 */
void doubleShot(PuncherAngles firstAngle, PuncherAngles secondAngle);

//----------------------------------------------------------------------------//
//                                   Intake                                   //
//----------------------------------------------------------------------------//

//---------- Motors ----------//

extern Motor intake;

//--------- Functions --------//

/**
 * sets speed of ball intake
 */
void setIntake(int speed);
