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
    //If the input values' total exceeds 200, scale them while maintaining their
    //proportion to each other
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

void driveVoltage(float y, float r, bool scalingEnabled)
{
    //Scale up from 127 to 12000
    y *= 12000.0 / 127.0;
    r *= 12000.0 / 127.0;

    //If the input values' total exceeds 12000, scale them while maintaining
    //their proportion to each other
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
    return;
}

void launch(bool blocking)
{
    //Set puncher motor target to next launch's end position
    puncher.move_absolute(numLaunches * 360 + 360, 100);

    if(blocking)
    {
        waitForPuncher();
        numLaunches++;
    }
    return;
}

void cockPuncher(bool blocking)
{
    //Set puncher motor target to current launch end position + 90 deg
    puncher.move_absolute(numLaunches * 360 + 90, 100);
    return;
}

void waitForPuncher(int endPos)
{
    //Wait for angle adjuster to be done
    auto settledUtil = okapi::SettledUtilFactory::create();
    while(!settledUtil.isSettled(endPos - puncher.get_position()))
    {
        delay(20);
    }
    return;
}

void setPuncherAngle(PuncherAngles angle, int speed, bool blocking)
{
    angleAdjuster.move_absolute(static_cast<double>(angle), speed);
    
    if(blocking)
    {
        //Wait for angle adjuster to be done
        auto settledUtil = okapi::SettledUtilFactory::create();
        while(!settledUtil.isSettled(angleAdjuster.get_target_position() - angleAdjuster.get_position()))
        {
            delay(20);
        }
    }
    return;
}

void setIntake(int speed)
{
    intake.move_velocity(speed);
    return;
}

//----------------------------------------------------------------------------//
