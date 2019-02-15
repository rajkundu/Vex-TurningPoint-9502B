//Header guard
#pragma once

//----------------------------------------------------------------------------//
//                                Miscellaneous                               //
//----------------------------------------------------------------------------//

//---------- Globals ---------//

extern Controller masterController;

//--------- Functions --------//

/**
 * Scales input according to deadband and maximum
 * @param input value to be scaled
 *  - range [-127, 127]
 * @param threshold threshold at/below which to return 0
 *  - range [-127, 127]
 * @return scaled value or 0
 *  - range [-127, 127]
 */
double scaleDeadband(double input, double threshold);

/**
 * Scales input according to exponential curve
 * @param input base
 * @param power power to which input will be raised
 * @param maxValue the maximum value to be reached
 * 
 * @return f(x) according to curve defined by parameters
 */
double expCurve(double input, double power, double maxValue);

//----------------------------------------------------------------------------//
//                                 Drivetrain                                 //
//----------------------------------------------------------------------------//

//---------- Globals ---------//

extern ChassisControllerPID drivetrain;
extern bool slewEnabled;
//Slew rate in [-1, 1] units per second
extern double slewRate_y;
extern double slewRate_r;
extern double y_last;
extern double r_last;

//--------- Functions --------//

/**
 * sets drivetrain velocity
 * @param y desired forward-backward component
 *  - range [-200, 200]
 *  - units RPM
 * @param r desired rotational component
 *  - range [-200, 200]
 *  - units RPM
 * @param preserveProportion whether or not to preserve proportion between y and
 *  r
 *  - default true
 */
void driveRPM(double y, double r, bool preserveProportion = true);

/**
 * sets drivetrain speed using voltage control
 * @param y desired forward-backward component
 *  - range [-127, 127]
 * @param r desired rotational component
 *  - range [-127, 127]
 * @param preserveProportion whether or not to preserve proportion between y and
 *  r
 *  - default true
 */
void driveVoltage(double y, double r, bool preserveProportion = true);

//----------------------------------------------------------------------------//
//                     Puncher & Cap Lift Synchronization                     //
//----------------------------------------------------------------------------//

class PuncherAngle
{
    protected:
        double angleValue;
        double lowerCapLiftInterferenceBound;
        double upperCapLiftInterferenceBound;
    public:
        //Constructors
        PuncherAngle(double angleValue, double lowerBound, double upperBound);

        //Getters
        double getAngleValue();
        double getLowerInterferenceBound();
        double getUpperInterferenceBound();
};

//----------------------------------------------------------------------------//
//                                  Cap Lift                                  //
//----------------------------------------------------------------------------//

//---------- Motors ----------//

extern Motor capLiftMotor;

//---------- Globals ---------//

const int CAPLIFT_VOLTAGE_HOLD = 1500;

//--------- Functions --------//

/**
 * gets angle of cap lift relative to home/starting position
 * @return angle of capLiftMotor / gear ratio = angle of cap lift
 */
double getCapLiftPos();

/**
 * returns whether the cap lift is in range of interfering with puncher
 * @return whether cap lift is interfering range
 */
bool capLiftInterfering();

//----------------------------------------------------------------------------//
//                                  Puncher                                   //
//----------------------------------------------------------------------------//

//---------- Motors ----------//

extern Motor puncher;
extern Motor angleAdjuster;

//---------- Globals ---------//

extern int numLaunches;
extern bool puncherReady;
namespace PuncherAngles
{
    extern PuncherAngle NEAR_HIGH_FLAG;
    extern PuncherAngle NEAR_LOW_FLAG;
    extern PuncherAngle FAR_HIGH_FLAG;
    extern PuncherAngle FAR_LOW_FLAG;
    extern PuncherAngle * current;
}

//--------- Functions --------//

/**
 * resets puncher home position
 */
void resetPuncher();

/**
 * launches ball by rotating puncher motor 360 degrees
 * @param blocking whether or not to wait for launch to complete
 */
void launch(bool blocking = false);

/**
 * moves puncher motor to end target position
 * @param endPos end target position relative to home position
 *  - units degrees
 * @param blocking whether or not to wait for puncher to reach end target position
 */
void movePuncherTo(int endPos, bool blocking = false);

/**
 * waits for puncher to reach its last-set target position
 */
void waitForPuncherReady();

/**
 * sets angle of puncher angle adjuster arm
 * @param angle the angle at which to set the puncher
 * @param speed the speed at which to run the angle adjuster motor
 *  - units RPM
 *  - default 50 RPM
 * @param blocking whether or not to wait for angle adjuster to reach angle
 *  - default false
 */
void setPuncherAngle(PuncherAngle &angle, int speed = 50, bool blocking = false);

/**
 * runs double shot macro
 * @param firstAngle the first angle at which to set the puncher angle
 *  adjuster arm
 *  - units degrees
 * @param secondAngle the second angle at which to set the puncher angle
 *  adjuster arm
 *  - units degrees
 */
void doubleShot(PuncherAngle &firstAngle, PuncherAngle &secondAngle);

/**
 * handles puncher during opcontrol
 * @param param unused null parameter
 */
void puncherHandler(void * param);

//----------------------------------------------------------------------------//
//                                   Intake                                   //
//----------------------------------------------------------------------------//

//---------- Motors ----------//

extern Motor intake;

//--------- Functions --------//

/**
 * sets speed of ball intake
 */
void setIntake(int speed);
