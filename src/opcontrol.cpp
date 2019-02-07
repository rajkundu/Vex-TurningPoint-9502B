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

	//Launch button booleans
	bool launchButtonPressed = false;
	bool launchButtonLastPressed = false;

	while(true)
	{
		//--------------------------------------------------------------------//
		//                             Drivetrain                             //
		//--------------------------------------------------------------------//

		//Drive arcade control with y and r inputs
		driveVoltage(
			expCurve(scaleDeadband(127 * masterController.getAnalog(ControllerAnalog::leftY), 8), 1.5, 127),
			expCurve(scaleDeadband(127 * masterController.getAnalog(ControllerAnalog::rightX), 8), 1.5, 127)
		);

		//--------------------------------------------------------------------//
		//                               Puncher                              //
		//--------------------------------------------------------------------//

		//Update puncher status
		updatePuncherReady();

		//Puncher launch = Button A
		launchButtonPressed = masterController.getDigital(ControllerDigital::A);
		if(launchButtonPressed && !launchButtonLastPressed)
		{
			if(puncherReady)
			{
				//Initiate launch
				launch(false);
			}
		}
		launchButtonLastPressed = launchButtonPressed;

		pros::lcd::clear_line(2);
		pros::lcd::print(2, "BPressed: %i", launchButtonPressed);
		
		pros::lcd::clear_line(3);
		pros::lcd::print(3, "PTarget: %f", puncher.getTargetPosition());
		pros::lcd::clear_line(4);
		pros::lcd::print(4, "PReady: %i", puncherReady);
		pros::lcd::clear_line(5);
		pros::lcd::print(5, "#L: %i", numLaunches);
		

		//Print angle adjuster to LLEMU
		pros::lcd::clear_line(1);
		pros::lcd::print(1, "PAngle: %f", angleAdjuster.get_position());
		
		//Double shot macro
		if(masterController.getDigital(ControllerDigital::B))
		{
			doubleShot(PuncherAngles::NEAR_HIGH_FLAG, PuncherAngles::NEAR_MID_FLAG);
		}

		//--------------------------------------------------------------------//
		//                               Intake                               //
		//--------------------------------------------------------------------//

		if(masterController.getDigital(ControllerDigital::L1))
		{
			//Intake balls
			setIntake(200);
		}
		else if(masterController.getDigital(ControllerDigital::L2))
		{
			//Flip caps/out-take balls
			setIntake(-150);
		}
		else
		{
			setIntake(0);
		}

		pros::delay(20);
	}
}
