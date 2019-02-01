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
 *  - range [-12000, 12000]
 * @param r desired rotational component
 *  - range [-12000, 12000]
 * @param scalingEnabled whether or not to apply proportional scaling to y and r
 *  - default true
 */
void driveVoltage(int y, int r, bool scalingEnabled = true);

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
    NEAR_LOW_FLAG = 75,
    FAR_HIGH_FLAG = 57,
    FAR_LOW_FLAG = 75
};

//--------- Functions --------//

/**
 * resets puncher home position
 */
void resetPuncher();

/**
 * launches ball by rotating puncher motor 360 degrees
 */
void launch();

/**
 * sets angle of puncher angle adjuster arm
 */
void setPuncherAngle(PuncherAngles angle, int speed = 50);

/**
 * sets speed of ball intake
 */
void setIntake(int speed);

//----------------------------------------------------------------------------//
