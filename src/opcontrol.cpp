#include "main.h"
#include "subsystems.hpp"

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
	//Instantiate controller object
	Controller masterController(ControllerId::master);

	//Set drivetrain brake mode
	drivetrain.setBrakeMode(AbstractMotor::brakeMode::coast);

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
		//                             Drivetrain                             //
		//--------------------------------------------------------------------//

		//Drive arcade control with y and r inputs
		int y = expCurve(scaleDeadband(127 * masterController.getAnalog(ControllerAnalog::leftY), 8), 1.0, 127);
		int r_linear = expCurve(scaleDeadband(127 * masterController.getAnalog(ControllerAnalog::leftX), 8), 1.0, 127);
		int r_sensitive = expCurve(scaleDeadband(127 * masterController.getAnalog(ControllerAnalog::rightX), 8), 1.3, 127);

		driveVoltage(y, r_linear + r_sensitive, false);

		//--------------------------------------------------------------------//
		//                               Puncher                              //
		//--------------------------------------------------------------------//

		//Update puncher status
		updatePuncherReady();

		//Puncher launch = Button Y
		launchButtonPressed = masterController.getDigital(ControllerDigital::Y);
		//If new press...
		if(launchButtonPressed && !launchButtonLastPressed)
		{
			if(puncherReady)
			{
				//Initiate launch
				launch(false);
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

		//Print angle adjuster to LLEMU
		pros::lcd::clear_line(1);
		pros::lcd::print(1, "PAngle: %f", angleAdjuster.getPosition());
		
		//Double shot macro
		if(masterController.getDigital(ControllerDigital::B))
		{
			//doubleShot(PuncherAngles::NEAR_HIGH_FLAG, PuncherAngles::NEAR_MID_FLAG);
		}

		//--------------------------------------------------------------------//
		//                              Cap Lift                              //
		//--------------------------------------------------------------------//

		if(masterController.getDigital(ControllerDigital::R2))
		{
			//Speed up as cap lift gets higher
			capLiftMotor.moveVoltage(12000 - (std::min(160.0, getCapLiftPos()) - getCapLiftPos()) * 45);
		}
		else if(masterController.getDigital(ControllerDigital::L2))
		{
			//Slow down as cap lift gets lower
			capLiftMotor.moveVoltage(-12000 + (std::min(160.0, getCapLiftPos()) - getCapLiftPos()) * 55);
		}
		else
		{
			//If cap is in carry position range, set brake mode to hold
			if(getCapLiftPos() < 135 && getCapLiftPos() > 30)
			{
				if(capLiftMotor.getBrakeMode() != AbstractMotor::brakeMode::hold)
				{
					capLiftMotor.setBrakeMode(AbstractMotor::brakeMode::hold);
				}
				capLiftMotor.moveVelocity(0);
			}
			//Else, set brake mode to coast and use physical hardstops
			else
			{
				if(capLiftMotor.getBrakeMode() != AbstractMotor::brakeMode::coast)
				{
					capLiftMotor.setBrakeMode(AbstractMotor::brakeMode::coast);
				}
				capLiftMotor.moveVelocity(0);
			}
		}

		pros::lcd::clear_line(2);
		pros::lcd::print(2, "CLPos#: %f", getCapLiftPos());

		//--------------------------------------------------------------------//
		//                               Intake                               //
		//--------------------------------------------------------------------//

		if(masterController.getDigital(ControllerDigital::R1))
		{
			//Intake balls
			setIntake(200);
		}
		else if(masterController.getDigital(ControllerDigital::L1))
		{
			//Flip caps/out-take balls
			setIntake(-200);
		}
		else
		{
			setIntake(0);
		}

		pros::delay(10);
	}
}
