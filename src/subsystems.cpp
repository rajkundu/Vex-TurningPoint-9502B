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
    {10, 3},
    //Right motors
    {-1, -2},
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

Motor puncher(9, true, AbstractMotor::gearset::red);
Motor angleAdjuster(7, false, AbstractMotor::gearset::red);

//---------- Globals ---------//

int numLaunches = 0;
const int puncherPositionValues[] = {50, 77, 57, 75};
SettledUtil puncherSettledUtil = SettledUtilFactory::create();
bool puncherReady = true;
bool puncherReady_last = true;

//--------- Functions --------//

void resetPuncher()
{
    /*puncher.moveVelocity(0);

    //Wait until puncher motor is stopped
    while(!puncher.isStopped())
    {
        pros::delay(10);
    }

    //Reset position variables
    puncher.tarePosition();
    numLaunches = 0;
    */

    //Wait for puncher motor to initialize
    while(!puncherReady)
    {
        updatePuncherReady();
        pros::delay(10);
    }
    return;
}

void updatePuncherReady()
{
    puncherReady = puncherSettledUtil.isSettled(puncher.getTargetPosition() - puncher.getPosition());

    //Update numLaunches if target position is not 0 (to ensure that this isn't
    //just the puncher motor starting at and immediately reaching 0)
    if(puncherReady && !puncherReady_last && (puncher.getTargetPosition() != 0))
    {
        numLaunches++;
    }

    puncherReady_last = puncherReady;
}

void waitForPuncherReady()
{
    //Wait until puncher reaches target
    while(!puncherReady)
    {
        updatePuncherReady();
        pros::delay(10);
    }

    return;
}

void launch(bool blocking)
{
    //Set puncher motor target to next launch's end position
    puncher.moveAbsolute(numLaunches * 360 + 360, 100);

    if(blocking)
    {
        puncherReady = false;
        waitForPuncherReady();
    }
    return;
}

void primePuncher(int primePos, bool blocking)
{
    //Set puncher motor target to current launch end position + 90 deg
    puncher.moveAbsolute(numLaunches * 360 + primePos, 100);

    if(blocking)
    {
        waitForPuncherReady();
    }
    return;
}

void setPuncherAngle(PuncherAngles angle, int speed, bool blocking)
{
    angleAdjuster.moveAbsolute(static_cast<int>(angle), speed);
    
    if(blocking)
    {
        //Wait for angle adjuster to be done
        auto angleSettledUtil = SettledUtilFactory::create(3, 5, 60_ms);
        while(!angleSettledUtil.isSettled(static_cast<int>(angle) - angleAdjuster.getPosition()))
        {
            pros::delay(10);
        }
    }
    return;
}

void doubleShot(PuncherAngles firstAngle, PuncherAngles secondAngle)
{
    //Set puncher to high flag
    setPuncherAngle(firstAngle, 50, true);

    //Launch and wait for completion
    launch(true);

    //Once first launch is complete, load second ball, initiate launch,
    //and wait for angle to be set to low flag
    setIntake(200);
    launch();
    setPuncherAngle(secondAngle, 50, true);

    //Wait for launch to complete and stop intake
    waitForPuncherReady();
    setIntake(0);
}

//----------------------------------------------------------------------------//
//                                  Cap Lift                                  //
//----------------------------------------------------------------------------//

//---------- Motors ----------//

Motor capLiftMotor(20, false, AbstractMotor::gearset::red);

//--------- Functions --------//

double getCapLiftPos()
{
    return capLiftMotor.getPosition() * 3.0 / 5.0;
}

//----------------------------------------------------------------------------//
//                                  Intake                                   //
//----------------------------------------------------------------------------//

//---------- Motors ----------//

Motor intake(8, false, AbstractMotor::gearset::green);

//--------- Functions --------//

/**
 * sets speed of ball intake
 */
void setIntake(int speed)
{
    intake.moveVelocity(speed);
    return;
}
