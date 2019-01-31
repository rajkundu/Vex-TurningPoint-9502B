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
 *  range [-200, 200]
 *  units RPM
 * @param r desired rotational component
 *  range [-200, 200]
 *  units RPM
 */
void driveRPM(int y, int r, bool scalingEnabled = true);

/**
 * sets drivetrain speed using voltage control
 * @param y desired forward-backward component
 *  range [-127, 127]
 *  units N/A
 * @param r desired rotational component
 *  range [-127, 127]
 *  units N/A
 */
void driveVoltage(int y, int r, bool scalingEnabled = true);

//----------------------------------------------------------------------------//