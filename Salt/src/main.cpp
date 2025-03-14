#include "main.h"

// Import libraries
#include "lemlib/api.hpp"
#include "lemlib/util.hpp"
#include "subsystems/rollerIntake.hpp"
#include "subsystems/indexer.hpp"
#include "subsystems/sideStakes.hpp"
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
	Driving,
	Intaking,
	SideStakes,
	Autonomous
};

enum class AllianceColor
{
	Red,
	Blue,
	No_Ring
};

constexpr int8_t FRONT_LEFT_PORT = 14;
constexpr int8_t MIDDLE_FRONT_LEFT_PORT = 9;
constexpr int8_t MIDDLE_BACK_LEFT_PORT = 6;
constexpr int8_t BACK_LEFT_PORT = 8;
constexpr int8_t FRONT_RIGHT_PORT = 1;
constexpr int8_t MIDDLE_FRONT_RIGHT_PORT = 18;
constexpr int8_t MIDDLE_BACK_RIGHT_PORT = 19;
constexpr int8_t BACK_RIGHT_PORT = 20;

constexpr int8_t INTAKE_PORT_1 = 13;
constexpr int8_t INTAKE_PORT_2 = 12;
constexpr int8_t CONVEYOR_PORT = 15;

constexpr int8_t SIDESTAKES_PORT = 7;

constexpr int8_t GYRO_PORT_TOP = 17;
constexpr int8_t GYRO_PORT_BOTTOM = 16;
constexpr int8_t OPTICAL_PORT = 10;
constexpr int8_t ROTATION_PORT = 2;

constexpr double TRACK_WIDTH = 18.375;
constexpr double WHEEL_DIAMETER = 2.75;
constexpr double DRIVE_RPM = 600;
constexpr double CHASE_POWER = 0.75;

constexpr double ODOM_WHEEL_DIAMETER = 0.087890625;
constexpr double HORIZONTAL_WHEEL_DISTANCE = 2.0625;
constexpr double VERTICAL_WHEEL_DISTANCE = 2.4375;

constexpr char CLAMP_SOLENOID = 'G';
constexpr char WING_SOLENOID = 'H';
constexpr char HORIZONTAL_POD_PORT_1 = 'C';
constexpr char HORIZONTAL_POD_PORT_2 = 'D';
constexpr char VERTICAL_POD_PORT_1 = 'E';
constexpr char VERTICAL_POD_PORT_2 = 'F';

pros::Controller driver(pros::controller_id_e_t::E_CONTROLLER_MASTER);

RobotState robotState = RobotState::Driving;
AllianceColor alliance_color = AllianceColor::No_Ring;

// MOTORS and PNEUMATICS
pros::adi::DigitalOut clamp_solenoid(CLAMP_SOLENOID);
pros::adi::DigitalOut wing_solenoid(WING_SOLENOID);

pros::MotorGroup leftSide({-FRONT_LEFT_PORT, -MIDDLE_FRONT_LEFT_PORT, -MIDDLE_BACK_LEFT_PORT, -BACK_LEFT_PORT});
pros::MotorGroup rightSide({FRONT_RIGHT_PORT, MIDDLE_FRONT_RIGHT_PORT, MIDDLE_BACK_RIGHT_PORT, BACK_RIGHT_PORT});
pros::MotorGroup riGroup({-INTAKE_PORT_1, INTAKE_PORT_2});
pros::MotorGroup conveyorGroup({CONVEYOR_PORT});
pros::MotorGroup sideStakesGroup({SIDESTAKES_PORT});

// SENSORS
pros::adi::Encoder horizontalPod(VERTICAL_POD_PORT_1, VERTICAL_POD_PORT_2, false);
pros::adi::Encoder verticalPod(HORIZONTAL_POD_PORT_1, HORIZONTAL_POD_PORT_2, true);

pros::IMU gyro_top(GYRO_PORT_TOP);
pros::IMU gyro_bottom(GYRO_PORT_BOTTOM);
MergedIMU gyro(&gyro_top, &gyro_bottom, true);
pros::Optical ringColor(OPTICAL_PORT);
pros::Rotation rotationSensor(ROTATION_PORT);
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
	34,	 // proportional gain (kP)
	0,	 // integral gain (kI)
	150, // derivative gain (kD)
	0,	 // anti windup
	1,	 // small error range, in inches
	100, // small error range timeout, in milliseconds
	3,	 // large error range, in inches
	500, // large error range timeout, in milliseconds
	20	 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularController(
	5.5,	 // proportional gain (kP)
	0,	 // integral gain (kI)
	35,	 // derivative gain (kD)
	0,	 // anti windup
	5,	 // small error range, in degrees
	100, // small error range timeout, in milliseconds
	10,	 // large error range, in degrees
	300, // large error range timeout, in milliseconds
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
Indexer ind(clamp_solenoid, wing_solenoid);
SideStakes sideStakes(sideStakesGroup);
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
	case RobotState::Autonomous:
		return "Autonomous";	
	default:
		return "Unknown State";
	}
}

const char *toString(AllianceColor color)
{
	switch (color)
	{
	case AllianceColor::Red:
		return "Red";
	case AllianceColor::Blue:
		return "Blue";
	case AllianceColor::No_Ring:
		return "No Ring";
	default:
		return "Unknown Color";
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
		leftSide.set_current_limit_all(1687);
		rightSide.set_current_limit_all(1687);
		riGroup.set_current_limit_all(2000);
		conveyorGroup.set_current_limit_all(2500);
		sideStakesGroup.set_current_limit_all(0);
	}

	if (state == RobotState::SideStakes)
	{
		leftSide.set_current_limit_all(2187);
		rightSide.set_current_limit_all(2187);
		riGroup.set_current_limit_all(0);
		conveyorGroup.set_current_limit_all(0);
		sideStakesGroup.set_current_limit_all(2500);
	}
	if (state == RobotState::Autonomous)
	{
		leftSide.set_current_limit_all(1500);
		rightSide.set_current_limit_all(1500);
		riGroup.set_current_limit_all(2000);
		conveyorGroup.set_current_limit_all(2500);
		sideStakesGroup.set_current_limit_all(1500);
	}
}

void initialize()
{
    pros::lcd::initialize();
    chassis.calibrate(); // existing line

    leftSide.set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
    rightSide.set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
    ringColor.set_led_pwm(50);
	rotationSensor.set_position(0);
	rotationSensor.set_reversed(true);
	sideStakesGroup.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

    pros::Task screenTask([&]()
    {
        chassis.setPose({0, 0, 90});
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::lcd::print(3, "Salt");
#if defined(QUAL_AUTO)
            pros::lcd::print(4, "QUAL");
#elif defined(MATCH_AUTO)
            pros::lcd::print(4, "MATCH");
#endif
            pros::lcd::print(5, "Robot State: %s", toString(robotState));
            pros::lcd::print(6, "Alliance Color: %s", toString(alliance_color));
			double positions = sideStakesGroup.get_position();
			pros::lcd::print(7, "Rotation: %d", rotationSensor.get_position()/100);
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });
}

void allianceColor()
{
	pros::c::optical_rgb_s_t rgb_value;
	rgb_value = ringColor.get_rgb();
	if (rgb_value.red > rgb_value.blue)
		if (rgb_value.red > 1000)
			alliance_color = AllianceColor::Red;
		else
			alliance_color = AllianceColor::No_Ring;
	else
		if (rgb_value.blue > 1000)
			alliance_color = AllianceColor::Blue;
		else
			alliance_color = AllianceColor::No_Ring;
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
	if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
	{
		if (robotState != RobotState::Intaking)
		{
			robotState = RobotState::Intaking;
			setcurrentstate(robotState);
		}
		ri.spin(600);
		conveyor.spin(500);
	}
	else if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
	{
		if (robotState != RobotState::Intaking)
		{
			robotState = RobotState::Intaking;
			setcurrentstate(robotState);
		}
		ri.spin(-600);
		conveyor.spin(-500);
	}
	else
	{
		ri.spin(0);
		conveyor.spin(0);
	}

	if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
	{
		ind.openClamp();
	}

	if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
	{
		ind.openWing();
	}

	if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
    	if (robotState != RobotState::SideStakes) {
        	robotState = RobotState::SideStakes;
        	setcurrentstate(robotState);
    	}
    	static int ssSeqIndex = 0;
    	static const SideStakesPosition ssSequence[] = {
        	SideStakesPosition::STOCK,
        	SideStakesPosition::LOAD,
        	SideStakesPosition::SIDESTAKES,
        	SideStakesPosition::LOAD
    	};
    	const int sequenceLength = sizeof(ssSequence) / sizeof(ssSequence[0]);
    	ssSeqIndex = (ssSeqIndex + 1) % sequenceLength;
    	sideStakes.moveTo(ssSequence[ssSeqIndex], rotationSensor);
}

	if (!driver.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
		!driver.get_digital(pros::E_CONTROLLER_DIGITAL_L2) &&
		!driver.get_digital(pros::E_CONTROLLER_DIGITAL_R2) &&
		!driver.get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&
		!driver.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
	{
		if (robotState != RobotState::Driving)
		{
			robotState = RobotState::Driving;
			setcurrentstate(robotState);
		}
	}
}

ASSET(J_M_1_txt);

void start_intake()
{
	conveyor.spin(500);
	ri.spin(600);
}
void stop_intake()
{
	conveyor.spin(0);
	ri.spin(0);
}

void deploy_clamp()
{
	ind.openClamp();
}

void deploy_wing()
{
	ind.openWing();
}

void toggle_stakes(){
	static int ssSeqIndex = 0;
    static const SideStakesPosition ssSequence[] = {
        SideStakesPosition::STOCK,
        SideStakesPosition::LOAD,
        SideStakesPosition::SIDESTAKES,
        SideStakesPosition::LOAD
    };
    const int sequenceLength = sizeof(ssSequence) / sizeof(ssSequence[0]);
    ssSeqIndex = (ssSeqIndex + 1) % sequenceLength;
    sideStakes.moveTo(ssSequence[ssSeqIndex], rotationSensor);
}

void qual_auto()
{
	setcurrentstate(RobotState::Autonomous);
	chassis.setPose({-58.5, 32, 270});
	pros::delay(1000);
	// ind.openWing();
	// chassis.moveToPoint(-43.5, 32, 2000, {false}, false);
	// chassis.turnToPoint(-15, 8, 2000, {false}, false);
	// chassis.moveToPoint(-15, 8, 2000, {false}, false);
	chassis.moveToPoint(-40.5, 32, 2000, {false}, false);
	chassis.turnToHeading(180,2000);
	chassis.moveToPoint(-40.5, 48, 2000, {false}, false);
	chassis.turnToHeading(270,2000);
	chassis.moveToPoint(-19.7, 46, 2000, {false}, true);
	ind.openClamp();
	// chassis.turnToPoint(-16.5, 39.8, 500, {false}, false);
	// chassis.moveToPoint(-16.5, 39.8, 2000, {false}, false);


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
		allianceColor();
		pros::delay(10);
	}
}