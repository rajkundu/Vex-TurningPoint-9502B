#include "main.h"
#include "subsystems.hpp"

//----------------------------------------------------------------------------//
//                                Miscellaneous                               //
//----------------------------------------------------------------------------//

//---------- Globals ---------//

Controller masterController(ControllerId::master);

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
bool slewEnabled = true;
//Slew rates in units [-127, 127] per loop iteration
double slewRate_y = 7;
double slewRate_r = 7;
//Variables for tracking change in y and r
double d_y = 0;
double d_r = 0;
double y_last = 0;
double r_last = 0;

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
    //Slew control
    if(slewEnabled)
    {
        d_y = y - y_last;
        d_r = r - r_last;

        //If y changed by more than the slew rate...
        if(std::abs(d_y) > slewRate_y)
        {
            y = (d_y < 0) ? y_last - slewRate_y : y_last + slewRate_y;
        }
        //If r changed by more than the slew rate...
        if(std::abs(d_r) > slewRate_r)
        {
            r = (d_r < 0) ? r_last - slewRate_r : r_last + slewRate_r;
        }

        y_last = y;
        r_last = r;
    }

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
//                     Puncher & Cap Lift Synchronization                     //
//----------------------------------------------------------------------------//

namespace PuncherAngles
{
    PuncherAngle NEAR_HIGH_FLAG(50, 20, 50);
    PuncherAngle NEAR_LOW_FLAG(74, 33, 50);
    PuncherAngle FAR_HIGH_FLAG(57, 18, 48);
    PuncherAngle FAR_LOW_FLAG(71, 33, 50);
    PuncherAngle * CURRENT;
}
PuncherAngle::PuncherAngle(double angleValue, double lowerBound, double upperBound)
{
    this->angleValue = angleValue;
    this->lowerCapLiftInterferenceBound = lowerBound;
    this->upperCapLiftInterferenceBound = upperBound;
}
double PuncherAngle::getAngleValue()
{
    return this->angleValue;
}
double PuncherAngle::getLowerInterferenceBound()
{
    return this->lowerCapLiftInterferenceBound;
}
double PuncherAngle::getUpperInterferenceBound()
{
    return this->upperCapLiftInterferenceBound;
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

bool capLiftInterfering()
{
    double capLiftPos = getCapLiftPos();
    return capLiftPos > PuncherAngles::CURRENT->getLowerInterferenceBound() || PuncherAngles::CURRENT->getUpperInterferenceBound();
}

void unobstructCapLift()
{
    //If lower than middle of interference range...
    if(getCapLiftPos() < (PuncherAngles::CURRENT->getUpperInterferenceBound() + PuncherAngles::CURRENT->getLowerInterferenceBound()) / 2.0)
    {
        //Move cap lift down out of the way
        capLiftMotor.moveVoltage(-6000);
        while(getCapLiftPos() > PuncherAngles::CURRENT->getLowerInterferenceBound())
        {
            pros::delay(REFRESH_MS);
        }
    }
    //Else, if higher than or in the middle of interference range...
    else
    {
        //Move cap lift up out of the way
        capLiftMotor.moveVoltage(8000);
        while(getCapLiftPos() < PuncherAngles::CURRENT->getUpperInterferenceBound())
        {
            pros::delay(REFRESH_MS);
        }
    }
    //Hold cap lift in place
    capLiftMotor.moveVoltage(CAPLIFT_VOLTAGE_HOLD);
}

//----------------------------------------------------------------------------//
//                                   Puncher                                  //
//----------------------------------------------------------------------------//

//---------- Motors ----------//

Motor puncher(9, true, AbstractMotor::gearset::red);
Motor angleAdjuster(7, false, AbstractMotor::gearset::red);

//---------- Globals ---------//

int numLaunches = 0;
bool puncherReady = false;

//--------- Functions --------//

void resetPuncher()
{
    /*puncher.moveVelocity(0);

    //Wait until puncher motor is stopped
    while(!puncher.isStopped())
    {
        pros::delay(REFRESH_MS);
    }

    //Reset position variables
    puncher.tarePosition();
    numLaunches = 0;
    */

   return;
}

void waitForPuncherReady()
{
    SettledUtil puncherSettledUtil = SettledUtilFactory::create(3, 5, 30_ms);

    while(!puncherSettledUtil.isSettled(puncher.getTargetPosition() - puncher.getPosition()))
    {
        pros::delay(REFRESH_MS);
    }
    return;
}

void launch(bool blocking)
{
    puncherReady = false;

    //Move cap lift if interfering
    if(capLiftInterfering())
    {
        unobstructCapLift();
    }

    movePuncherTo(360, blocking);
    
    //Increment numLaunches after firing
    numLaunches++;

    puncherReady = true;

    return;
}

void movePuncherTo(int endPos, bool blocking)
{
    //Set puncher motor target to current launch home + endPos
    puncher.moveAbsolute(numLaunches * 360 + endPos, 100);

    if(blocking)
    {
        waitForPuncherReady();
    }
    return;
}

void setPuncherAngle(PuncherAngle &pAngle, int speed, bool blocking)
{
    PuncherAngles::CURRENT = &pAngle;

    pros::lcd::clear_line(3);
	pros::lcd::print(3, "PAngle: %f, LIB: %f, UIB: %f", PuncherAngles::CURRENT->getAngleValue(), PuncherAngles::CURRENT->getLowerInterferenceBound(), PuncherAngles::CURRENT->getUpperInterferenceBound());

    angleAdjuster.moveAbsolute(pAngle.getAngleValue(), speed);
    
    if(blocking)
    {
        //Wait for angle adjuster to be done
        auto angleSettledUtil = SettledUtilFactory::create(3, 5, 30_ms);
        while(!angleSettledUtil.isSettled(pAngle.getAngleValue() - angleAdjuster.getPosition()))
        {
            pros::delay(REFRESH_MS);
        }
    }
    return;
}

void doubleShot(PuncherAngle &firstAngle, PuncherAngle &secondAngle)
{
    //Set puncher to high flag
    setPuncherAngle(firstAngle, 50, true);

    //Launch and wait for completion
    launch(true);

    //Once first launch is complete, load second ball, initiate launch,
    //and wait for angle to be set to low flag
    setIntake(200);
    launch(false);
    setPuncherAngle(secondAngle, 50, true);

    //Wait for launch to complete and stop intake
    waitForPuncherReady();
    setIntake(0);
}

void puncherHandler(void * param)
{
    //Button booleans
	bool launchButtonPressed = false;
	bool launchButtonLastPressed = false;
	bool togglePuncherAnglePressed = false;
	bool togglePuncherAngleLastPressed = false;

	//0 = Low Flag, 1 = High Flag
	bool puncherAngleLowHigh = 0;

    while(true)
    {
        //--------------------------------------------------------------------//
		//                               Puncher                              //
		//--------------------------------------------------------------------//

		//Puncher launch = Button Y
		launchButtonPressed = masterController.getDigital(ControllerDigital::Y);
		//If new press...
		if(launchButtonPressed && !launchButtonLastPressed)
		{
			if(puncherReady)
			{
				//Initiate launch
				launch(true);
			}
		}
		launchButtonLastPressed = launchButtonPressed;

		//Angle adjuster toggle = Button X
		togglePuncherAnglePressed = masterController.getDigital(ControllerDigital::X);
		//If new press...
		if(togglePuncherAnglePressed && !togglePuncherAngleLastPressed)
		{
			puncherAngleLowHigh = !puncherAngleLowHigh;
			
			//Set new puncher angle
			if(puncherAngleLowHigh == 0)
			{
				setPuncherAngle(PuncherAngles::NEAR_HIGH_FLAG);
			}
			else
			{
				setPuncherAngle(PuncherAngles::NEAR_LOW_FLAG);
			}
		}
		togglePuncherAngleLastPressed = togglePuncherAnglePressed;
		
		//Double shot macro
		if(masterController.getDigital(ControllerDigital::B))
		{
			doubleShot(PuncherAngles::NEAR_HIGH_FLAG, PuncherAngles::NEAR_LOW_FLAG);
		}

        pros::delay(REFRESH_MS);
    }
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
