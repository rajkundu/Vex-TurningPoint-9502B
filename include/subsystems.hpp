//Guard
#pragma once

//-------------------------------- Drivetrain --------------------------------//

//---------- Motors ----------//

extern Motor frontLeft;
extern Motor frontRight;
extern Motor backLeft;
extern Motor backRight;

//--------- Functions --------//

/**
 * sets drivetrain velocity
 * @param y desired forward-backward component
 *  - range [-200, 200]
 *  - units RPM
 * @param r desired rotational component
 *  - range [-200, 200]
 *  - units RPM
 * @param scalingEnabled whether or not to apply proportional scaling to y and r
 *  - default true
 */
void driveRPM(int y, int r, bool scalingEnabled = true);

/**
 * sets drivetrain speed using voltage control
 * @param y desired forward-backward component
 *  - range [-127, 127]
 * @param r desired rotational component
 *  - range [-127, 127]
 * @param scalingEnabled whether or not to apply proportional scaling to y and r
 *  - default true
 */
void driveVoltage(float y, float r, bool scalingEnabled = true);

//----------------------------------------------------------------------------//

//---------------------------------- Puncher ---------------------------------//

//---------- Motors ----------//

extern Motor puncher;
extern Motor angleAdjuster;
extern Motor intake;

//-------- Global Vars -------//

extern int numLaunches;
enum class PuncherAngles
{
    NEAR_HIGH_FLAG = 45,
    NEAR_MID_FLAG = 75,
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

/**
 * sets speed of ball intake
 */
void setIntake(int speed);

//----------------------------------------------------------------------------//
