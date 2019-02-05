//Header guard
#pragma once

//----------------------------------------------------------------------------//
//                                Miscellaneous                               //
//----------------------------------------------------------------------------//

//--------- Functions --------//

/**
 * Scales input according to deadband and maximum
 * @param input value to be scaled
 * @param threshold threshold at/below which to return 0
 * @return scaled value or 0
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
    NEAR_MID_FLAG = 77,
    FAR_HIGH_FLAG = 57,
    FAR_MID_FLAG = 75
};

//--------- Functions --------//

/**
 * resets puncher home position
 */
void resetPuncher();

/**
 * launches ball by rotating puncher motor 360 degrees
 * @param blocking whether or not to wait for launch to complete
 */
void launch(bool blocking = false);

/**
 * primes puncher (moves slide track back so that ball can be loaded)
 * @param priming position relative to home position
 *  - units degrees
 *  - default 90 degrees
 * @param blocking whether or not to wait for priming to complete
 */
void primePuncher(int primePos = 90, bool blocking = false);

/**
 * waits for puncher to reach certain position
 * @param endPos the position until which to wait
 *  - units degrees
 *  - default next multiple of 360 deg (next launch end position)
 */
void waitForPuncher(int endPos = numLaunches * 360 + 360);

/**
 * waits for puncher to complete launch
 */
void waitForLaunch();

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
 * @param firstPuncherAngle the first angle at which to set the puncher angle
 *  adjuster arm
 *  - units degrees
 * @param secondPuncherAngle the second angle at which to set the puncher angle
 *  adjuster arm
 *  - units degrees
 */
void doubleShot(PuncherAngles firstPuncherAngle, PuncherAngles secondPuncherAngle);

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
