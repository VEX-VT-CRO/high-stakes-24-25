#include "main.h"

// Import libraries
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"
#include "subsystems/rollerIntake.hpp"
#include "subsystems/indexer.hpp"
#include "subsystems/sideStakes.hpp"
#include "subsystems/conveyor.hpp"

// Two major definitions for each bot
#define QUAL_AUTO
// #define MATCH_AUTO

// #define ARCADE
#define TANK

// Use states to control current limits
enum class RobotState
{
	Driving,
	Intaking,
	SideStakes
};

// Initialize ports and key variables

bool auto_climb_state = false;

constexpr int8_t FRONT_LEFT_PORT = 1;
constexpr int8_t MIDDLE_FRONT_LEFT_PORT = 2;
constexpr int8_t MIDDLE_BACK_LEFT_PORT = 15;
constexpr int8_t BACK_LEFT_PORT = 13;
constexpr int8_t FRONT_RIGHT_PORT = 10;
constexpr int8_t MIDDLE_FRONT_RIGHT_PORT = 9;
constexpr int8_t MIDDLE_BACK_RIGHT_PORT = 16;
constexpr int8_t BACK_RIGHT_PORT = 18;

constexpr int8_t INTAKE_PORT = 6;
constexpr int8_t CONVEYOR_PORT = 20;

constexpr int8_t SIDESTAKES_PORT_1 = 11;
constexpr int8_t SIDESTAKES_PORT_2 = 12;

constexpr int8_t GYRO_PORT = 5;
constexpr int8_t OPTICAL_PORT = 3;
constexpr int8_t ROTATION_PORT = 4;

constexpr double TRACK_WIDTH = 12;
constexpr double WHEEL_DIAMETER = 3;
constexpr double DRIVE_RPM = 600;
constexpr double CHASE_POWER = 2;

constexpr double ODOM_WHEEL_DIAMETER = 2;
constexpr double HORIZONTAL_WHEEL_DISTANCE = 1.5625;
constexpr double VERTICAL_WHEEL_DISTANCE = -4.0625;

constexpr char LEFT_SOLENOID = 'A';
constexpr char RIGHT_SOLENOID = 'B';
constexpr char HORIZONTAL_POD_PORT_1 = 'C';
constexpr char HORIZONTAL_POD_PORT_2 = 'D';
constexpr char VERTICAL_POD_PORT_1 = 'E';
constexpr char VERTICAL_POD_PORT_2 = 'F';

pros::Controller driver(pros::controller_id_e_t::E_CONTROLLER_MASTER);

RobotState robotState = RobotState::Driving;

// MOTORS and PNEUMATICS
pros::adi::DigitalOut right_solenoid(RIGHT_SOLENOID);
pros::adi::DigitalOut left_solenoid(LEFT_SOLENOID);

pros::MotorGroup leftSide({FRONT_LEFT_PORT, MIDDLE_FRONT_LEFT_PORT, MIDDLE_BACK_LEFT_PORT, -BACK_LEFT_PORT});
pros::MotorGroup rightSide({-FRONT_RIGHT_PORT, -MIDDLE_FRONT_RIGHT_PORT, -MIDDLE_BACK_RIGHT_PORT, BACK_RIGHT_PORT});
pros::MotorGroup riGroup({INTAKE_PORT});
pros::MotorGroup conveyorGroup({CONVEYOR_PORT});
pros::MotorGroup sideStakesGroup({SIDESTAKES_PORT_1, -SIDESTAKES_PORT_2});

// SENSORS
pros::adi::Encoder horizontalPod(HORIZONTAL_POD_PORT_1, HORIZONTAL_POD_PORT_2);
pros::adi::Encoder verticalPod(VERTICAL_POD_PORT_1, VERTICAL_POD_PORT_2);

pros::IMU gyro(GYRO_PORT);
pros::Rotation sideStakesRotation(ROTATION_PORT);
pros::Optical ringColor(OPTICAL_PORT);

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
Indexer ind(left_solenoid, right_solenoid);
SideStakes sideStakes(sideStakesGroup, sideStakesRotation);
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
	case RobotState::Driving:
		return "Driving";
	case RobotState::Intaking:
		return "Intaking";
	case RobotState::SideStakes:
		return "SideStakes";
	default:
		return "Unknown State";
	}
}

// Set current limits based on robot state
void setcurrentstate(RobotState state)
{

	if (state == RobotState::Driving)
	{
		leftSide.set_current_limit_all(2500);
		rightSide.set_current_limit_all(2500);
		riGroup.set_current_limit_all(0);
		conveyorGroup.set_current_limit_all(0);
		sideStakesGroup.set_current_limit_all(0);

	}

	if (state == RobotState::Intaking)
	{
		leftSide.set_current_limit_all(1875);
		rightSide.set_current_limit_all(1875);
		riGroup.set_current_limit_all(2500);
		conveyorGroup.set_current_limit_all(2500);
		sideStakesGroup.set_current_limit_all(0);
	}

	if (state == RobotState::SideStakes)
	{
		leftSide.set_current_limit_all(1875);
		rightSide.set_current_limit_all(1875);
		riGroup.set_current_limit_all(0);
		conveyorGroup.set_current_limit_all(0);
		sideStakesGroup.set_current_limit_all(2500);
	}
}

void initialize()
{
	pros::lcd::initialize();
	chassis.calibrate();

	leftSide.set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
	rightSide.set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);

	pros::Task screenTask([&]()
						  {
    chassis.setPose({0, 0, 0});
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Salt");
#if defined(QUAL_AUTO)
                pros::lcd::print(4, "QUAL");
#elif defined(MATCH_AUTO)
                pros::lcd::print(4, "MATCH");
#endif
            pros::lcd::print(5, "Robot State: %s", toString(robotState));        
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        } });
}

bool allianceColor()
{
	pros::c::optical_rgb_s_t rgb_value;
	rgb_value = ringColor.get_rgb();
	if (rgb_value.red > rgb_value.blue)
		return true;
	return false;
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
	if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
	{
		if (robotState != RobotState::Intaking)
		{
			robotState = RobotState::Intaking;
			setcurrentstate(robotState);
		}
		ri.spin(12000);
	}
	else if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
	{
		if (robotState != RobotState::Intaking)
		{
			robotState = RobotState::Intaking;
			setcurrentstate(robotState);
		}
		ri.spin(-12000);
	}
	else
	{
		ri.spin(0);
	}

	if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
	{
		ind.clamp();
	}

	if (!driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
		!driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2) &&
		!driver.get_digital(pros::E_CONTROLLER_DIGITAL_A) &&
		!driver.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
	{
		if (robotState != RobotState::Driving)
		{
			robotState = RobotState::Driving;
			setcurrentstate(robotState);
		}
	}
}

ASSET(J_M_1_txt);

void qual_auto()
{

}

void match_auto()
{

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
	pros::Task colorTask(allianceColor);
	#if defined(QUAL_AUTO)
		qual_auto();
	#elif defined(MATCH_AUTO)
		match_auto();
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
		if (!auto_climb_state)
			pollController();
		if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
		{
			auto_climb_state = false;
		}
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