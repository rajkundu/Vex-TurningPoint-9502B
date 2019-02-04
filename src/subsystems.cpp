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

void primePuncher(int primePos, bool blocking)
{
    //Set puncher motor target to current launch end position + 90 deg
    puncher.move_absolute(numLaunches * 360 + primePos, 100);

    if(blocking)
    {
        waitForPuncher(primePos);
    }
    return;
}

void waitForPuncher(int endPos)
{
    //Wait for puncher rotation to be done
    auto settledUtil = okapi::SettledUtilFactory::create();
    //Wait until puncher reaches (endPos) degrees past home position
    while(!settledUtil.isSettled(numLaunches * 360 + endPos - puncher.get_position()))
    {
        delay(20);
    }
    return;
}

void waitForLaunch()
{
    waitForPuncher(360);
    numLaunches++;
}

void setPuncherAngle(PuncherAngles angle, int speed, bool blocking)
{
    angleAdjuster.move_absolute(static_cast<double>(angle), speed);
    
    if(blocking)
    {
        //Wait for angle adjuster to be done
        auto settledUtil = okapi::SettledUtilFactory::create(3, 5, 60_ms);
        while(!settledUtil.isSettled(static_cast<float>(angle) - angleAdjuster.get_position()))
        {
            delay(20);
        }
    }
    return;
}

void doubleShot(PuncherAngles firstPuncherAngle, PuncherAngles secondPuncherAngle)
{
    //Set puncher to high flag
    setPuncherAngle(firstPuncherAngle, 50, true);

    //Launch and wait for completion
    launch();
    waitForLaunch();

    //Once first launch is complete, load second ball, initiate launch,
    //and wait for angle to be set to low flag
    setIntake(200);
    launch();
    setPuncherAngle(secondPuncherAngle, 50, true);

    //Wait for launch to complete and stop intake
    waitForLaunch();
    setIntake(0);
}

void setIntake(int speed)
{
    intake.move_velocity(speed);
    return;
}

//----------------------------------------------------------------------------//
