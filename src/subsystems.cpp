#include "main.h"
#include "subsystems.hpp"

//-------------------------------- Drivetrain --------------------------------//

//---------- Motors ----------//

Motor frontLeft(1, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
Motor frontRight(2, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
Motor backLeft(3, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
Motor backRight(4, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);

//--------- Functions --------//

void driveRPM(int y, int r, bool scalingEnabled = true)
{
    //If the values are greater than 200, scale them proportionally so that they
    //are within the motor bounds of 200
    if(scalingEnabled)
    {
        if(abs(y) + abs(r) > 200)
        {
            y = 200 * y / ((float)(abs(y) + abs(r)));
            r = 200 * r / ((float)(abs(y) + abs(r)));
        }
    }

    frontLeft.move_velocity(y + r);
    frontRight.move_velocity(-y + r);
    backLeft.move_velocity(y + r);
    backRight.move_velocity(-y + r);
}

//----------------------------------------------------------------------------//