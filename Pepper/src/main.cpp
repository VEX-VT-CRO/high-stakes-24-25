#include "main.h"

// Import libraries
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"
#include "subsystems/rollerintake.hpp"
#include "subsystems/indexer.hpp"
#include "subsystems/climb.hpp"

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
	Climbing
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

constexpr int8_t CLIMB_1_PORT = 11;
constexpr int8_t CLIMB_2_PORT = 20;

constexpr int8_t HORIZONTAL_POD_PORT = 14;
constexpr int8_t VERTICAL_POD_PORT = 17;
constexpr int8_t GYRO_PORT = 5;
constexpr int8_t BALL_DISTANCE_PORT = 3;

constexpr double TRACK_WIDTH = 12;
constexpr double WHEEL_DIAMETER = 3;
constexpr double DRIVE_RPM = 600;
constexpr double CHASE_POWER = 2;

constexpr int32_t BALL_PRESENT_DISTANCE = 150;
constexpr int INTAKE_INTAKING_DIRECTION = 1;

constexpr double ODOM_WHEEL_DIAMETER = 2;
constexpr double HORIZONTAL_WHEEL_DISTANCE = 1.5625;
constexpr double VERTICAL_WHEEL_DISTANCE = -4.0625;

constexpr char BACK_LEFT_SOLENOID = 'B';
constexpr char BACK_RIGHT_SOLENOID = 'A';
constexpr char FRONT_LEFT_SOLENOID = 'C';
constexpr char FRONT_RIGHT_SOLENOID = 'D';
constexpr char ODOMETRY_SOLENOID = 'E';

pros::Controller driver(pros::controller_id_e_t::E_CONTROLLER_MASTER);

RobotState robotState = RobotState::Driving;


// MOTORS and PNEUMATICS
pros::adi::DigitalOut back_right_solenoid(BACK_RIGHT_SOLENOID);
pros::adi::DigitalOut back_left_solenoid(BACK_LEFT_SOLENOID);
pros::adi::DigitalOut front_right_solenoid(FRONT_RIGHT_SOLENOID);
pros::adi::DigitalOut front_left_solenoid(FRONT_LEFT_SOLENOID);
pros::adi::DigitalOut odometry_solenoid(ODOMETRY_SOLENOID);

pros::MotorGroup leftSide({FRONT_LEFT_PORT, MIDDLE_FRONT_LEFT_PORT, MIDDLE_BACK_LEFT_PORT, -BACK_LEFT_PORT});
pros::MotorGroup rightSide({-FRONT_RIGHT_PORT, -MIDDLE_FRONT_RIGHT_PORT, -MIDDLE_BACK_RIGHT_PORT, BACK_RIGHT_PORT});
pros::MotorGroup riGroup({INTAKE_PORT});
pros::MotorGroup climbGroup({-CLIMB_1_PORT, CLIMB_2_PORT});

// SENSORS
pros::Rotation horizontalPod(HORIZONTAL_POD_PORT);
pros::Rotation verticalPod(-VERTICAL_POD_PORT);
pros::IMU gyro(GYRO_PORT);
pros::Distance ballDistance(BALL_DISTANCE_PORT);

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
Indexer ind(back_right_solenoid, back_left_solenoid, front_right_solenoid, front_left_solenoid, odometry_solenoid);
Climb climb(climbGroup);
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
	case RobotState::Climbing:
		return "Climbing";
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
		climbGroup.set_current_limit_all(0);
	}

	if (state == RobotState::Intaking)
	{
		leftSide.set_current_limit_all(2200);
		rightSide.set_current_limit_all(2200);
		riGroup.set_current_limit_all(2400);
		climbGroup.set_current_limit_all(0);
	}

	if (state == RobotState::Climbing)
	{
		riGroup.set_current_limit_all(0);
		climbGroup.set_current_limit_all(2500);
		leftSide.set_current_limit_all(1875);
		rightSide.set_current_limit_all(1875);
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
            pros::lcd::print(3, "J");
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

void autoIntakeManager()
{
	while (!pros::competition::is_autonomous())
	{
		pros::delay(10);
	}
	while (pros::competition::is_autonomous())
	{
		if (ballDistance.get() < BALL_PRESENT_DISTANCE && riGroup.get_direction_all()[0] == INTAKE_INTAKING_DIRECTION)
		{
			riGroup.move(0);
		}

		pros::delay(10);
	}
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
		ri.spin(ri.STANDARD_MV);
	}
	else if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
	{
		if (robotState != RobotState::Intaking)
		{
			robotState = RobotState::Intaking;
			setcurrentstate(robotState);
		}
		ri.spin(-ri.STANDARD_MV);
	}
	else
	{
		ri.spin(0);
	}

	if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
	{
		ind.openFrontLeft();
	}
	if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
	{
		ind.openFrontRight();
	}

	if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
	{
		ind.openBack();
	}

	if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_A))
	{
		if (robotState != RobotState::Climbing)
		{
			robotState = RobotState::Climbing;
			setcurrentstate(robotState);
		}
		climb.moveClimb(-12000);
	}
	else if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
	{
		if (robotState != RobotState::Climbing)
		{
			robotState = RobotState::Climbing;
			setcurrentstate(robotState);
		}
		climb.moveClimb(12000);
	}
	else if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
	{
		if (robotState != RobotState::Climbing)
		{
			robotState = RobotState::Climbing;
			setcurrentstate(robotState);
		}
		climb.deployClimb_J();
		auto_climb_state = true;
	}
	else
	{
		climb.moveClimb(0);
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

ASSET(PB_M_1_txt);
ASSET(PB_M_2_txt);
ASSET(PB_M_3_txt);
ASSET(PB_M_4_txt);
ASSET(PB_M_3_1_txt);
ASSET(PB_M_1_1_txt);

ASSET(J_M_1_txt);
ASSET(J_M_2_txt);

ASSET(J_Q_1_txt);
ASSET(J_Q_2_txt);
ASSET(J_Q_3_txt);

void qualJ()
{

}

// The match function has the main calls you would do for an autonomous routine besides the non-drivebase motor calls
void matchJ()
{
	chassis.setPose({-34, -64, 0});
	pros::delay(100);
	chassis.moveToPoint(-34, -35, 3000);
	chassis.turnToHeading(90, 2000);
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
	std::vector<double> positions = climbGroup.get_position_all();
	std::vector<double> targetPositions = climbGroup.get_target_position_all();
		if (!auto_climb_state)
			pollController();
		else if (abs(positions[0] - targetPositions[0]) < 0.5)
			auto_climb_state = false;
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