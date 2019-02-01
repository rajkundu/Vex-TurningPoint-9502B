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
            y = round(200 * y / static_cast<float>(abs(y) + abs(r)));
            r = round(200 * r / static_cast<float>(abs(y) + abs(r)));
        }
    }

    frontLeft.move_velocity(y + r);
    frontRight.move_velocity(-y + r);
    backLeft.move_velocity(y + r);
    backRight.move_velocity(-y + r);
}

void driveVoltage(int y, int r, bool scalingEnabled)
{
    //If the input values are greater than 12000, scale them proportionally so
    //that they are within the motor voltage bounds of [-12000, 12000]
    if(scalingEnabled)
    {
        if(abs(y) + abs(r) > 12000)
        {
            y = round(12000 * y / static_cast<float>(abs(y) + abs(r)));
            r = round(12000 * r / static_cast<float>(abs(y) + abs(r)));
        }
    }

    frontLeft.move_voltage(y + r);
    frontRight.move_voltage(-y + r);
    backLeft.move_voltage(y + r);
    backRight.move_voltage(-y + r);
}

//----------------------------------------------------------------------------//

//---------------------------------- Puncher ---------------------------------//

//---------- Motors ----------//

Motor puncher(5, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_DEGREES);
Motor angleAdjuster(6, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_DEGREES);
Motor intake(7, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);

//-------- Global Vars -------//

int numLaunches = 0;

//--------- Functions --------//

void resetPuncher()
{
    puncher.move_velocity(0);

    //Wait until puncher motor is stopped
    while(!puncher.is_stopped())
    {
        delay(20);
    }

    //Reset position variables
    puncher.tare_position();
    numLaunches = 0;
}

void launch()
{
    numLaunches++;
    puncher.move_absolute(numLaunches * 360, 100);
}

void setPuncherAngle(PuncherAngles angle, int speed)
{
    angleAdjuster.move_absolute(static_cast<double>(angle), speed);
}

void setIntake(int speed)
{
    intake.move_velocity(speed);
}

//----------------------------------------------------------------------------//
