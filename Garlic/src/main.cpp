
#include "main.h"

// Import libraries
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"
#include "subsystems/rollerIntake.hpp"
#include "subsystems/indexer.hpp"
#include "subsystems/conveyorlift.hpp"
#include "subsystems/conveyor.hpp"
#include "subsystems/mergedIMU.hpp"

// Two major definitions for each bot
#define QUAL_AUTO
// #define MATCH_AUTO

// #define ARCADE
#define TANK

// Use states to control current limits
enum class RobotState
{
	Casual,
	SideStakes
};

enum class RiseControl
{
	Front,
	Back,
	Both
};
// Initialize ports and key variables

bool riser_status = false;

constexpr int8_t FRONT_LEFT_PORT = 16;
constexpr int8_t MIDDLE_LEFT_PORT = 17;
constexpr int8_t BACK_LEFT_PORT = 18;
constexpr int8_t FRONT_RIGHT_PORT = 1;
constexpr int8_t MIDDLE_RIGHT_PORT = 2;
constexpr int8_t BACK_RIGHT_PORT = 3;

constexpr int8_t INTAKE_PORT = 11;
constexpr int8_t CONVEYOR_PORT = 12;

constexpr int8_t CONVEYOR_LIFT_LEFT_FRONT_PORT = 13;
constexpr int8_t CONVEYOR_LIFT_RIGHT_FRONT_PORT = 4;
constexpr int8_t CONVEYOR_LIFT_LEFT_BACK_PORT = 14;
constexpr int8_t CONVEYOR_LIFT_RIGHT_BACK_PORT = 5;

constexpr char HORIZONTAL_POD_PORT_1 = 'F';
constexpr char HORIZONTAL_POD_PORT_2 = 'E';
constexpr char VERTICAL_POD_PORT_1 = 'H';
constexpr char VERTICAL_POD_PORT_2 = 'G';
constexpr int8_t GYRO_PORT_TOP = 8;
constexpr int8_t GYRO_PORT_BOTTOM = 10;

constexpr double TRACK_WIDTH = 12;
constexpr double WHEEL_DIAMETER = 3;
constexpr double DRIVE_RPM = 600;
constexpr double CHASE_POWER = 2;

constexpr double ODOM_WHEEL_DIAMETER = 0.087890625;
constexpr double HORIZONTAL_WHEEL_DISTANCE = 0.5;
constexpr double VERTICAL_WHEEL_DISTANCE = -0.5;

constexpr char STOPPER_SOLENOID_FRONT = 'D';
constexpr char STOPPER_SOLENOID_BACK = 'C';
constexpr char HOLDER_SOLENOID = 'B';

pros::Controller driver(pros::controller_id_e_t::E_CONTROLLER_MASTER);

RobotState robotState = RobotState::Casual;


RiseControl riserControl = RiseControl::Front;


// MOTORS and PNEUMATICS
pros::adi::DigitalOut stopper_solenoid_front(STOPPER_SOLENOID_FRONT);
pros::adi::DigitalOut stopper_solenoid_back(STOPPER_SOLENOID_BACK);
pros::adi::DigitalOut holder_solenoid(HOLDER_SOLENOID);

pros::MotorGroup leftSide({FRONT_LEFT_PORT, MIDDLE_LEFT_PORT, -BACK_LEFT_PORT});
pros::MotorGroup rightSide({-FRONT_RIGHT_PORT, -MIDDLE_RIGHT_PORT, BACK_RIGHT_PORT});
pros::MotorGroup riGroup({INTAKE_PORT});
pros::MotorGroup conveyorLiftGroupFront({-CONVEYOR_LIFT_LEFT_FRONT_PORT, CONVEYOR_LIFT_RIGHT_FRONT_PORT});
pros::MotorGroup conveyorLiftGroupBack({CONVEYOR_LIFT_LEFT_BACK_PORT, -CONVEYOR_LIFT_RIGHT_BACK_PORT});
pros::MotorGroup conveyorGroup({CONVEYOR_PORT});

// SENSORS
pros::adi::Encoder horizontalPod(VERTICAL_POD_PORT_1, VERTICAL_POD_PORT_2, false);
pros::adi::Encoder verticalPod(HORIZONTAL_POD_PORT_1, HORIZONTAL_POD_PORT_2, true);
pros::IMU gyro_top(GYRO_PORT_TOP);
pros::IMU gyro_bottom(GYRO_PORT_BOTTOM);
MergedIMU gyro(&gyro_top, &gyro_bottom, true);

// LEMLIB STRUCTURES

lemlib::TrackingWheel horizontalWheel(&horizontalPod, ODOM_WHEEL_DIAMETER, HORIZONTAL_WHEEL_DISTANCE);
lemlib::TrackingWheel verticalWheel(&verticalPod, ODOM_WHEEL_DIAMETER, VERTICAL_WHEEL_DISTANCE);

lemlib::Drivetrain LLDrivetrain(
	&leftSide,
	&rightSide,
	TRACK_WIDTH,
	WHEEL_DIAMETER,
	DRIVE_RPM,
	CHASE_POWER);

lemlib::ControllerSettings linearController(
	10,	 // proportional gain (kP)
	0,	 // integral gain (kI)
	120, // derivative gain (kD)
	3,	 // anti windup
	1,	 // small error range, in inches
	100, // small error range timeout, in milliseconds
	3,	 // large error range, in inches
	500, // large error range timeout, in milliseconds
	10	 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularController(
	2,	 // proportional gain (kP)
	0,	 // integral gain (kI)
	10,	 // derivative gain (kD)
	3,	 // anti windup
	1,	 // small error range, in degrees
	100, // small error range timeout, in milliseconds
	3,	 // large error range, in degrees
	500, // large error range timeout, in milliseconds
	0	 // maximum acceleration (slew)
);

lemlib::OdomSensors sensors(
	&verticalWheel,
	nullptr,
	&horizontalWheel,
	nullptr,
	&gyro);

lemlib::Chassis chassis(LLDrivetrain, linearController, angularController, sensors);

RollerIntake ri(riGroup);
Indexer ind(holder_solenoid);
ConveyorLift conveyorlift(conveyorLiftGroupFront, conveyorLiftGroupBack, stopper_solenoid_front, stopper_solenoid_back);
Conveyor conveyor(conveyorGroup);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

const char *toString(RobotState state)
{
	switch (state)
	{
	case RobotState::Casual:
		return "Casual";
	case RobotState::SideStakes:
		return "Side Stakes";
	default:
		return "Unknown State";
	}
}

const char *toString(RiseControl control)
{
	switch (control)
	{
	case RiseControl::Front:
		return "Front";
	case RiseControl::Back:
		return "Back";
	case RiseControl::Both:
		return "Both";
	default:
		return "Unknown Control";
	}
}

// Set current limits based on robot state
void setcurrentstate(RobotState state)
{

	if (state == RobotState::Casual)
	{
		leftSide.set_current_limit_all(2500);
		rightSide.set_current_limit_all(2500);
		riGroup.set_current_limit_all(2500);
		conveyorGroup.set_current_limit_all(2500);
		conveyorLiftGroupFront.set_current_limit_all(0);
		conveyorLiftGroupBack.set_current_limit_all(0);
	}


	if (state == RobotState::SideStakes)
	{
		leftSide.set_current_limit_all(1500);
		rightSide.set_current_limit_all(1500);
		riGroup.set_current_limit_all(0);
		conveyorGroup.set_current_limit_all(1000);
		conveyorLiftGroupFront.set_current_limit_all(2500);
		conveyorLiftGroupBack.set_current_limit_all(2500);
	}
}

void initialize()
{
	pros::lcd::initialize();
	chassis.calibrate();

	leftSide.set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
	rightSide.set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
	conveyorLiftGroupFront.set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
	conveyorLiftGroupBack.set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);

	pros::Task screenTask([&]()
						  {
    chassis.setPose({0, 0, 0});
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Garlic");
#if defined(QUAL_AUTO)
                pros::lcd::print(4, "QUAL");
#elif defined(MATCH_AUTO)
                pros::lcd::print(4, "MATCH");
#endif
            pros::lcd::print(5, "Robot State: %s", toString(robotState));      
			pros::lcd::print(6, "Current riser: %s", toString(riserControl));
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        } });
}

void autoIntakeManager()
{

}

// Link to curve
// https://www.desmos.com/calculator/umicbymbnl
double deadband = 0;
double minOutput = 0;
float expCurve(float input, float curveGain)
{
	// return 0 if input is within deadzone
	if (fabs(input) <= deadband)
		return 0;
	// g is the output of g(x) as defined in the Desmos graph
	const float g = fabs(input) - deadband;
	// g127 is the output of g(127) as defined in the Desmos graph
	const float g127 = 127 - deadband;
	// i is the output of i(x) as defined in the Desmos graph
	const float i = pow(curveGain, g - 127) * g * lemlib::sgn(input);
	// i127 is the output of i(127) as defined in the Desmos graph
	const float i127 = pow(curveGain, g127 - 127) * g127;
	return (127.0 - minOutput) / (127) * i * 127 / i127 + minOutput * lemlib::sgn(input);
}

// Poll controller for input
void pollController()
{
	if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
	{
		if (robotState != RobotState::Casual)
		{
			robotState = RobotState::Casual;
			setcurrentstate(robotState);
		}
		ri.spin(ri.STANDARD_MV);
	}
	else if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
	{
		if (robotState != RobotState::Casual)
		{
			robotState = RobotState::Casual;
			setcurrentstate(robotState);
		}
		ri.spin(-ri.STANDARD_MV);
	}
	else
	{
		if(!pros::competition::is_autonomous()){
			ri.spin(0);
		}
	}

	if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
	{
		conveyorlift.openStopperFront();
	}

	if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
	{
		conveyorlift.openStopperBack();
	}

	if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
	{
		if (robotState != RobotState::SideStakes)
		{
			robotState = RobotState::SideStakes;
			setcurrentstate(robotState);
		}
		if(riserControl == RiseControl::Front){
			conveyorLiftGroupFront.move_velocity(30);
		}
		else if (riserControl == RiseControl::Back){
			conveyorLiftGroupBack.move_velocity(30);
		}
		else{
			conveyorLiftGroupFront.move_velocity(30);
			conveyorLiftGroupBack.move_velocity(30);
		}
	}
	else if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
	{
		if (robotState != RobotState::SideStakes)
		{
			robotState = RobotState::SideStakes;
			setcurrentstate(robotState);
		}
		if(riserControl == RiseControl::Front){
			conveyorLiftGroupFront.move_velocity(-30);
		}
		else if (riserControl == RiseControl::Back){
			conveyorLiftGroupBack.move_velocity(-30);
		}
		else{
			conveyorLiftGroupFront.move_velocity(-30);
			conveyorLiftGroupBack.move_velocity(-30);
		}
	}
	else
	{
		if(!pros::competition::is_autonomous()){
			conveyorLiftGroupBack.move_voltage(0);
			conveyorLiftGroupFront.move_voltage(0);
		}
	}

	if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
	{
		if (robotState != RobotState::Casual)
		{
			robotState = RobotState::Casual;
			setcurrentstate(robotState);
		}
		conveyorGroup.move_velocity(400);
	}
	else if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
	{
		if (robotState != RobotState::Casual)
		{
			robotState = RobotState::Casual;
			setcurrentstate(robotState);
		}
		conveyorGroup.move_velocity(-400);
	}
	else
	{
		if(!pros::competition::is_autonomous()){
			conveyorGroup.move_voltage(0);
		}
	}

	if(driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
		int nextValue = (static_cast<int>(riserControl) + 1) % 3;
		riserControl = static_cast<RiseControl>(nextValue);
	}

	if (!driver.get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&
		!driver.get_digital(pros::E_CONTROLLER_DIGITAL_R2) &&
		!driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
		!driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2) &&
		!driver.get_digital(pros::E_CONTROLLER_DIGITAL_UP) &&
		!driver.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) &&
		!driver.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) &&
		!driver.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) &&
		!driver.get_digital(pros::E_CONTROLLER_DIGITAL_A))
	{
		if(!pros::competition::is_autonomous()){
			if (robotState != RobotState::Casual)
			{
				robotState = RobotState::Casual;
				setcurrentstate(robotState);
			}
		}	
	}
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

ASSET(J_M_1_txt);

void qualJ()
{

}

// The match function has the main calls you would do for an autonomous routine besides the non-drivebase motor calls
void matchJ()
{
	
}

void autonomous_skills(){
	chassis.setPose({-59, -59, 225});
	pros::delay(100);
	chassis.moveToPoint(-60.5, -60.5, 500, {false});
	chassis.moveToPoint(-59, -59, 500);
	chassis.turnToHeading(45, 500);
	chassis.moveToPoint(-47, -47, 500);
	chassis.turnToHeading(270, 500);
	chassis.moveToPoint(-35, -47, 500);

	chassis.turnToHeading(25, 500);
	chassis.moveToPoint(-28, -31, 500);
	chassis.turnToHeading(115, 500);
	chassis.moveToPoint(-7, -42, 500);
	chassis.turnToHeading(60, 500);
	chassis.moveToPoint(-56.5, -58.5, 1000, {false});

	chassis.follow(J_M_1_txt, 30, 3500, {false});
}
/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{
	pros::Task intakeTask(autoIntakeManager);
#if defined(QUAL_AUTO)
	qualJ();
#elif defined(MATCH_AUTO)
	matchJ();
#endif
}

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
	while (true)
	{
	pollController();
	int l = driver.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
#if defined(ARCADE)
		int r = driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		// r = expCurve(r, 1.015);
		chassis.arcade(l, r);
#elif defined(TANK)
		int r = driver.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		chassis.tank(l, r);
#endif

		pros::delay(10);
	}
}
