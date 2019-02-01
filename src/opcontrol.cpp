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
	Controller masterController(E_CONTROLLER_MASTER);

	while(true)
	{
		//Drive arcade control with y and r inputs
		driveVoltage(
			masterController.get_analog(E_CONTROLLER_ANALOG_LEFT_Y),
			masterController.get_analog(E_CONTROLLER_ANALOG_RIGHT_X)
		);

		//Puncher launch = Button A
		if(masterController.get_digital_new_press(E_CONTROLLER_DIGITAL_A))
		{
			//Launch and wait for completion
			launch(false);
		}

		lcd::clear_line(1);
		lcd::print(1, "PAngle: %f", angleAdjuster.get_position());
		
		//Double shot macro
		if(masterController.get_digital_new_press(E_CONTROLLER_DIGITAL_B))
		{
			doubleShot(PuncherAngles::NEAR_HIGH_FLAG, PuncherAngles::NEAR_MID_FLAG);
		}

		if(masterController.get_digital(E_CONTROLLER_DIGITAL_L1))
		{
			//Intake balls
			setIntake(200);
		}
		else if(masterController.get_digital(E_CONTROLLER_DIGITAL_L2))
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
