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
void driveVoltage(int y, int r, bool scalingEnabled = true);

//----------------------------------------------------------------------------//