#include "main.h"
#include "subsystems.hpp"

//-------------------------------- Drivetrain --------------------------------//

//---------- Motors ----------//

Motor frontLeft(1, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
Motor frontRight(2, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
Motor backLeft(3, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);
Motor backRight(4, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);

//--------- Functions --------//

void driveRPM(int y, int r, bool scalingEnabled)
{
    //If the values are greater than 200, scale them proportionally so that they
    //are within the motor velocity bounds of [-200, 200]
    if(scalingEnabled)
    {
        if(abs(y) + abs(r) > 200)
        {
            y = round(200 * y / ((float)(abs(y) + abs(r))));
            r = round(200 * r / ((float)(abs(y) + abs(r))));
        }
    }

    frontLeft.move_velocity(y + r);
    frontRight.move_velocity(-y + r);
    backLeft.move_velocity(y + r);
    backRight.move_velocity(-y + r);
}

void driveVoltage(int y, int r, bool scalingEnabled)
{
    //If the input values are greater than 127, scale them proportionally so
    //that they are within the motor voltage bounds of [-12000, 12000]
    if(scalingEnabled)
    {
        if(abs(y) + abs(r) > 127)
        {
            //12000 because in mV, not RPM
            y = round(12000 * y / ((float)(abs(y) + abs(r))));
            r = round(12000 * r / ((float)(abs(y) + abs(r))));
        }
    }

    frontLeft.move_voltage(y + r);
    frontRight.move_voltage(-y + r);
    backLeft.move_voltage(y + r);
    backRight.move_voltage(-y + r);
}

//----------------------------------------------------------------------------//
