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

	while (true)
	{
		//Drive arcade control with y and r inputs
		driveVoltage(
			masterController.get_analog(E_CONTROLLER_ANALOG_LEFT_Y),
			masterController.get_analog(E_CONTROLLER_ANALOG_RIGHT_X)
		);
		
		pros::delay(20);
	}
}
