#include "main.h"
#include "subsystems.hpp"

//----------------------------------------------------------------------------//
//                                Miscellaneous                               //
//----------------------------------------------------------------------------//

//--------- Functions --------//

double scaleDeadband(double input, double threshold)
{
    if(std::abs(input) > threshold)
    {
        return std::copysign((std::abs(input) - std::abs(threshold)) * (127.0 / (127.0 - std::abs(threshold))), input);
    }
    else
    {
        return 0;
    }
}

double expCurve(double input, double power, double maxValue)
{
    return maxValue * std::copysign(std::pow(std::abs(input) / std::abs(maxValue), power), input);
}

//----------------------------------------------------------------------------//
//                                 Drivetrain                                 //
//----------------------------------------------------------------------------//

//---------- Globals ---------//

ChassisControllerPID drivetrain = ChassisControllerFactory::create(
    //Left motors
    {1, 2},
    //Right motors
    {-3, -4},
    //Distance PID constants
    IterativePosPIDController::Gains{0.5, 0, 0},
    //Angle PID constants (keeps robot straight)
    IterativePosPIDController::Gains{0.1, 0.05, 0},
    //Turn PID constants
    IterativePosPIDController::Gains{0.2, 0, 0},
    //Gearset
    AbstractMotor::gearset::green,
    //Wheel diameter, wheelbase width
    {4.1_in, 12.5_in}
);

//--------- Functions --------//

void driveRPM(double y, double r, bool preserveProportion)
{
    //Scale down from 200 to 1
    y /= 200.0;
    r /= 200.0;

    //If the input values' total exceeds 1, scale them to maintain their
    //proportion to each other
    if(preserveProportion)
    {
        if(abs(y) + abs(r) > 1)
        {
            y = y / static_cast<float>(abs(y) + abs(r));
            r = r / static_cast<float>(abs(y) + abs(r));
        }
    }

    drivetrain.driveVector(y, r);
}

void driveVoltage(double y, double r, bool preserveProportion)
{
    //Scale down from 127 to 1
    y /= 127.0;
    r /= 127.0;

    //If the input values' total exceeds 1, scale them to maintain their
    //proportion to each other
    if(preserveProportion)
    {
        if(abs(y) + abs(r) > 1)
        {
            y = y / static_cast<float>(abs(y) + abs(r));
            r = r / static_cast<float>(abs(y) + abs(r));
        }
    }

    drivetrain.arcade(y, r);
}

//----------------------------------------------------------------------------//
//                                   Puncher                                  //
//----------------------------------------------------------------------------//

//---------- Motors ----------//

Motor puncher(5, true, AbstractMotor::gearset::red);
Motor angleAdjuster(6, false, AbstractMotor::gearset::red);

//---------- Globals ---------//

int numLaunches = 0;

//--------- Functions --------//

void resetPuncher()
{
    puncher.moveVelocity(0);

    //Wait until puncher motor is stopped
    while(!puncher.isStopped())
    {
        pros::delay(10);
    }

    //Reset position variables
    puncher.tarePosition();
    numLaunches = 0;
    return;
}

void launch(bool blocking)
{
    //Set puncher motor target to next launch's end position
    puncher.moveAbsolute(numLaunches * 360 + 360, 100);

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
    puncher.moveAbsolute(numLaunches * 360 + primePos, 100);

    if(blocking)
    {
        waitForPuncher(primePos);
    }
    return;
}

void waitForPuncher(int endPos)
{
    //Wait for puncher rotation to be done
    auto settledUtil = SettledUtilFactory::create();
    //Wait until puncher reaches (endPos) degrees past home position
    while(!settledUtil.isSettled(numLaunches * 360 + endPos - puncher.getPosition()))
    {
        pros::delay(10);
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
    angleAdjuster.moveAbsolute(static_cast<double>(angle), speed);
    
    if(blocking)
    {
        //Wait for angle adjuster to be done
        auto settledUtil = SettledUtilFactory::create(3, 5, 60_ms);
        while(!settledUtil.isSettled(static_cast<float>(angle) - angleAdjuster.getPosition()))
        {
            pros::delay(10);
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

//----------------------------------------------------------------------------//
//                                  Intake                                   //
//----------------------------------------------------------------------------//

//---------- Motors ----------//

Motor intake(7, false, AbstractMotor::gearset::green);

//--------- Functions --------//

/**
 * sets speed of ball intake
 */
void setIntake(int speed)
{
    intake.moveVelocity(speed);
    return;
}
