
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
// #define QUAL_AUTO
#define MATCH_AUTO

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

void setcurrentstate(RobotState state);
void moveConveyorLiftTo(ConveyorLift &conveyorLift, ConveyorPosition fromPos, ConveyorPosition toPos);
void toggle();

// Initialize ports and key variables
bool riser_status = false;
bool allianceMode = false;

constexpr int8_t FRONT_LEFT_PORT = 15;
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

constexpr char HORIZONTAL_POD_PORT_1 = 'H';
constexpr char HORIZONTAL_POD_PORT_2 = 'G';
constexpr char VERTICAL_POD_PORT_1 = 'F';
constexpr char VERTICAL_POD_PORT_2 = 'E';
constexpr int8_t GYRO_PORT_TOP = 8;
constexpr int8_t GYRO_PORT_BOTTOM = 10;

constexpr double TRACK_WIDTH = 12;
constexpr double WHEEL_DIAMETER = 2.75;
constexpr double DRIVE_RPM = 600;
constexpr double CHASE_POWER = 0.89686;

constexpr double ODOM_WHEEL_DIAMETER = 0.087890625;
constexpr double HORIZONTAL_WHEEL_DISTANCE = -1.238;
constexpr double VERTICAL_WHEEL_DISTANCE = -0.674;

constexpr char STOPPER_SOLENOID_FRONT = 'C';
constexpr char STOPPER_SOLENOID_BACK = 'D';
constexpr char HOLDER_SOLENOID = 'B';
constexpr char MOVER_SOLENOID = 'A';

pros::Controller driver(pros::controller_id_e_t::E_CONTROLLER_MASTER);

RobotState robotState = RobotState::Casual;

RiseControl riserControl = RiseControl::Front;

// MOTORS and PNEUMATICS
pros::adi::DigitalOut stopper_solenoid_front(STOPPER_SOLENOID_FRONT);
pros::adi::DigitalOut stopper_solenoid_back(STOPPER_SOLENOID_BACK);
pros::adi::DigitalOut holder_solenoid(HOLDER_SOLENOID);
pros::adi::DigitalOut mover_solenoid(MOVER_SOLENOID);

pros::Motor frontLeft(FRONT_LEFT_PORT);
pros::Motor middleLeft(MIDDLE_LEFT_PORT);
pros::Motor backLeft(-BACK_LEFT_PORT);
pros::Motor frontRight(-FRONT_RIGHT_PORT);
pros::Motor middleRight(-MIDDLE_RIGHT_PORT);
pros::Motor backRight(BACK_RIGHT_PORT);
static bool _motor_init = []()
{
	int voltage = 12000 * 16 / 22;
	backLeft.set_voltage_limit(voltage);
	backRight.set_voltage_limit(voltage);
	return true;
}();
pros::MotorGroup leftSide({FRONT_LEFT_PORT, MIDDLE_LEFT_PORT, -BACK_LEFT_PORT});
pros::MotorGroup rightSide({-FRONT_RIGHT_PORT, -MIDDLE_RIGHT_PORT, BACK_RIGHT_PORT});
pros::Motor intake(INTAKE_PORT);
pros::Motor conveyorLiftLeftFront(-CONVEYOR_LIFT_LEFT_FRONT_PORT);
pros::Motor conveyorLiftRightFront(CONVEYOR_LIFT_RIGHT_FRONT_PORT);
pros::Motor conveyorLiftLeftBack(CONVEYOR_LIFT_LEFT_BACK_PORT);
pros::Motor conveyorLiftRightBack(-CONVEYOR_LIFT_RIGHT_BACK_PORT);
pros::Motor conveyorMotor(CONVEYOR_PORT);

// SENSORS
pros::adi::Encoder horizontalPod(HORIZONTAL_POD_PORT_1, HORIZONTAL_POD_PORT_2, true);
pros::adi::Encoder verticalPod(VERTICAL_POD_PORT_1, VERTICAL_POD_PORT_2, false);
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
	20,	 // proportional gain (kP)
	0,	 // integral gain (kI)
	150, // derivative gain (kD)
	0,	 // anti windup
	0.5, // small error range, in inches
	50,	 // small error range timeout, in milliseconds
	1.5, // large error range, in inches
	200, // large error range timeout, in milliseconds
	20	 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularController(
	6.5, // proportional gain (kP)
	0,	 // integral gain (kI)
	52,	 // derivative gain (kD)
	0,	 // anti windup
	2,	 // small error range, in degrees
	50,	 // small error range timeout, in milliseconds
	5,	 // large error range, in degrees
	150, // large error range timeout, in milliseconds
	0	 // maximum acceleration (slew)
);

lemlib::OdomSensors sensors(
	&verticalWheel,
	nullptr,
	&horizontalWheel,
	nullptr,
	&gyro);

lemlib::Chassis chassis(LLDrivetrain, linearController, angularController, sensors);

RollerIntake ri(intake);
Indexer ind(holder_solenoid, mover_solenoid);
ConveyorLift conveyorlift(conveyorLiftLeftFront, conveyorLiftRightFront, conveyorLiftLeftBack, conveyorLiftRightBack, stopper_solenoid_front, stopper_solenoid_back);
Conveyor conveyor(conveyorMotor);
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

void setcurrentstate(RobotState state)
{

	if (state == RobotState::Casual)
	{
		leftSide.set_current_limit_all(2500);
		rightSide.set_current_limit_all(2500);
		intake.set_current_limit(2500);
		conveyorMotor.set_current_limit(2500);
		conveyorLiftLeftFront.set_current_limit(0);
		conveyorLiftRightFront.set_current_limit(0);
		conveyorLiftLeftBack.set_current_limit(0);
		conveyorLiftRightBack.set_current_limit(0);
	}

	if (state == RobotState::SideStakes)
	{
		leftSide.set_current_limit_all(1500);
		rightSide.set_current_limit_all(1500);
		intake.set_current_limit_all(0);
		conveyorMotor.set_current_limit_all(1000);
		conveyorLiftLeftFront.set_current_limit_all(2500);
		conveyorLiftRightFront.set_current_limit_all(2500);
		conveyorLiftLeftBack.set_current_limit_all(2500);
		conveyorLiftRightBack.set_current_limit_all(2500);
	}
}

void initialize()
{
	pros::lcd::initialize();
	chassis.calibrate();

	leftSide.set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
	rightSide.set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
	conveyorLiftLeftFront.set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
	conveyorLiftRightFront.set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
	conveyorLiftLeftBack.set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);
	conveyorLiftRightBack.set_brake_mode_all(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);

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
// Poll controller for input
void pollController()
{
	// Create a static variable to track manual mode
	static bool manualMode = false;

	// R1 and R2 for intake and conveyor based on lift position
	if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
	{
		// Make sure we're in casual mode for intake/conveyor operation
		if (robotState != RobotState::Casual)
		{
			robotState = RobotState::Casual;
			setcurrentstate(robotState);
		}

		// Different behavior based on conveyorlift position
		if (conveyorlift.position == MOGO)
		{
			// In MOGO mode, spin both intake and conveyor
			ri.spin(ri.STANDARD_MV);
			conveyorMotor.move_velocity(160);
		}
		else if (conveyorlift.position == ALLIANCE || conveyorlift.position == SIDE)
		{
			// In ALLIANCE or SIDE mode, only spin the conveyor
			ri.spin(0); // Ensure intake is off
			conveyorMotor.move_velocity(160);
		}
		else if (conveyorlift.position == STOCK)
		{
			// In STOCK mode, don't spin either
			ri.spin(0);
			conveyorMotor.move_velocity(0);
		}
	}
	else if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
	{
		// Make sure we're in casual mode for intake/conveyor operation
		if (robotState != RobotState::Casual)
		{
			robotState = RobotState::Casual;
			setcurrentstate(robotState);
		}

		// Different behavior based on conveyorlift position
		if (conveyorlift.position == MOGO)
		{
			// In MOGO mode, spin both intake and conveyor in reverse
			ri.spin(-ri.STANDARD_MV);
			conveyorMotor.move_velocity(-160);
		}
		else if (conveyorlift.position == ALLIANCE || conveyorlift.position == SIDE)
		{
			// In ALLIANCE or SIDE mode, only spin the conveyor in reverse
			ri.spin(0); // Ensure intake is off
			conveyorMotor.move_velocity(-160);
		}
		else if (conveyorlift.position == STOCK)
		{
			// In STOCK mode, don't spin either
			ri.spin(0);
			conveyorMotor.move_velocity(0);
		}
	}
	else
	{
		// Stop both when no buttons are pressed (unless in autonomous)
		if (!pros::competition::is_autonomous())
		{
			ri.spin(0);
			conveyorMotor.move_voltage(0);
		}
	}

	// L1 for toggle function
	if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
	{
		toggle();
	}

	// L2 for goal clamp (previously on B)
	if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
	{
		ind.openHolder();
	}

	// A button toggles manual mode
	if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
	{
		manualMode = !manualMode;
	}

	// In manual mode, handle riser control and pneumatics
	if (manualMode)
	{
		// Up/Down for risers
		if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
		{
			// Temporarily switch to Side Stakes mode for manual operation
			RobotState previousState = robotState;
			robotState = RobotState::SideStakes;
			setcurrentstate(robotState);

			if (riserControl == RiseControl::Front)
			{
				conveyorLiftLeftFront.move_velocity(30);
				conveyorLiftRightFront.move_velocity(30);
			}
			else if (riserControl == RiseControl::Back)
			{
				conveyorLiftLeftBack.move_velocity(30);
				conveyorLiftRightBack.move_velocity(30);
			}
			else
			{
				conveyorLiftLeftFront.move_velocity(30);
				conveyorLiftRightFront.move_velocity(30);
				conveyorLiftLeftBack.move_velocity(30);
				conveyorLiftRightBack.move_velocity(30);
			}
		}
		else if (driver.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
		{
			// Temporarily switch to Side Stakes mode for manual operation
			RobotState previousState = robotState;
			robotState = RobotState::SideStakes;
			setcurrentstate(robotState);

			if (riserControl == RiseControl::Front)
			{
				conveyorLiftLeftFront.move_velocity(-30);
				conveyorLiftRightFront.move_velocity(-30);
			}
			else if (riserControl == RiseControl::Back)
			{
				conveyorLiftLeftBack.move_velocity(-30);
				conveyorLiftRightBack.move_velocity(-30);
			}
			else
			{
				conveyorLiftLeftFront.move_velocity(-30);
				conveyorLiftRightFront.move_velocity(-30);
				conveyorLiftLeftBack.move_velocity(-30);
				conveyorLiftRightBack.move_velocity(-30);
			}
		}
		else
		{
			// Stop motors when no button is pressed
			if (!pros::competition::is_autonomous())
			{
				conveyorLiftLeftBack.move_voltage(0);
				conveyorLiftRightBack.move_voltage(0);
				conveyorLiftLeftFront.move_voltage(0);
				conveyorLiftRightFront.move_voltage(0);

				// Reset to Casual mode if we were temporarily in Side Stakes mode
				if (robotState == RobotState::SideStakes)
				{
					robotState = RobotState::Casual;
					setcurrentstate(robotState);
				}
			}
		}

		// Left/Right for pneumatics in manual mode
		if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
		{
			conveyorlift.openStopperFront();
		}

		if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
		{
			conveyorlift.openStopperBack();
		}

		// In manual mode, B cycles through the riser control modes
		if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
		{
			int nextValue = (static_cast<int>(riserControl) + 1) % 3;
			riserControl = static_cast<RiseControl>(nextValue);
		}
	}
	else
	{
		if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
		{
			chassis.setPose(-56.5, 0, 90);
			toggle();
			pros::delay(100);
			conveyorMotor.move_velocity(130);
			intake.move_velocity(600);
			chassis.moveToPoint(-50.5, 0, 1100, {}, true);
			pros::delay(1100);
			conveyorMotor.move_velocity(0);
			intake.move_velocity(0);
			allianceMode = true;
			toggle();
			chassis.moveToPoint(-57.5, 0, 1000, {false}, true);
			pros::delay(300);
			conveyorMotor.move_velocity(130);
			pros::delay(1300);
			conveyorMotor.move_velocity(0);
			chassis.moveToPoint(-55.5, 0, 1000, {}, true);
			chassis.turnToHeading(135, 2500);
			toggle();
			pros::delay(400);
		}
	}

	// X button toggles alliance mode
	if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
	{
		allianceMode = !allianceMode;
	}

	// Y button toggles the mover
	if (driver.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
	{
		ind.openMover();
	}
}

void toggle()
{
	switch (conveyorlift.position)
	{
	case STOCK:
		// First transformation is always from STOCK to MOGO
		moveConveyorLiftTo(conveyorlift, STOCK, MOGO);
		conveyorlift.goingUp = true;
		break;
	case MOGO:
		// From MOGO, go to ALLIANCE if allianceMode is true, otherwise go to SIDE
		if (allianceMode)
		{
			moveConveyorLiftTo(conveyorlift, MOGO, ALLIANCE);
			conveyorlift.goingUp = true;
			allianceMode = false; // Reset after use
		}
		else
		{
			moveConveyorLiftTo(conveyorlift, MOGO, SIDE);
			conveyorlift.goingUp = true;
		}
		break;
	case ALLIANCE:
		// From ALLIANCE, only go down to MOGO
		moveConveyorLiftTo(conveyorlift, ALLIANCE, MOGO);
		conveyorlift.goingUp = false;
		break;
	case SIDE:
		// From SIDE, only go down to MOGO
		moveConveyorLiftTo(conveyorlift, SIDE, MOGO);
		conveyorlift.goingUp = false;
		break;
	default:
		break;
	}
}

void moveConveyorLiftTo(ConveyorLift &conveyorLift, ConveyorPosition fromPos, ConveyorPosition toPos)
{
	// Store original state to return to after movement
	RobotState previousState = robotState;

	// Set to Side Stakes mode for the movement
	robotState = RobotState::SideStakes;
	setcurrentstate(robotState);

	// Update the position
	conveyorLift.position = toPos;

	// Handle different transitions
	if (fromPos == STOCK && toPos == MOGO)
	{
		// STOCK to MOGO transition
		conveyorLift.openStopperFront();
		conveyorLift.motor_back_left.move_absolute(150, 150);
		conveyorLift.motor_back_right.move_absolute(150, 150);
		pros::delay(500);
		conveyorLift.motor_back_left.move_voltage(6000);
		conveyorLift.motor_back_right.move_voltage(6000);
		pros::delay(200);
		conveyorLift.openStopperBack();
		conveyorLift.motor_back_left.move_voltage(0);
		conveyorLift.motor_back_right.move_voltage(0);
	}
	else if (fromPos == MOGO && toPos == ALLIANCE)
	{
		// MOGO to ALLIANCE transition
		conveyorLift.openStopperBack();
		conveyorLift.openStopperFront();
		pros::delay(100);
		conveyorLift.motor_back_left.move_relative(80, 100);
		conveyorLift.motor_back_right.move_relative(80, 100);
		conveyorLift.motor_front_left.move_relative(80, 100);
		conveyorLift.motor_front_right.move_relative(80, 100);
		pros::delay(500);
		conveyorLift.openStopperFront();
		conveyorLift.motor_back_left.move_voltage(6000);
		conveyorLift.motor_back_right.move_voltage(6000);
		pros::delay(200);
		conveyorLift.openStopperBack();
		conveyorLift.motor_back_left.move_voltage(0);
		conveyorLift.motor_back_right.move_voltage(0);
	}
	else if (fromPos == MOGO && toPos == SIDE)
	{
		// MOGO to SIDE transition
		conveyorLift.openStopperBack();
		conveyorLift.openStopperFront();
		pros::delay(130);
		// conveyorLift.motor_back_left.move_relative(-20, 100);
		// conveyorLift.motor_back_right.move_relative(-20, 100);
		// pros::delay(100);
		conveyorLift.motor_back_left.move_relative(380, 100);
		conveyorLift.motor_back_right.move_relative(380, 100);
		pros::delay(100);
		conveyorLift.motor_front_left.move_relative(370, 100);
		conveyorLift.motor_front_right.move_relative(370, 100);
		pros::delay(900);
		conveyorLift.openStopperFront();
		conveyorLift.motor_back_left.move_voltage(6000);
		conveyorLift.motor_back_right.move_voltage(6000);
		pros::delay(250);
		conveyorLift.openStopperBack();
		conveyorLift.motor_back_left.move_voltage(0);
		conveyorLift.motor_back_right.move_voltage(0);
	}
	else if (fromPos == ALLIANCE && toPos == MOGO)
	{
		conveyorLift.openStopperFront();
		conveyorLift.openStopperBack();
		pros::delay(100);
		conveyorLift.motor_back_left.move_relative(-80, 100);
		conveyorLift.motor_back_right.move_relative(-80, 100);
		conveyorLift.motor_front_left.move_relative(-80, 100);
		conveyorLift.motor_front_right.move_relative(-80, 100);
		pros::delay(200);
		conveyorLift.motor_back_left.move_voltage(-6000);
		conveyorLift.motor_back_right.move_voltage(-6000);
		conveyorLift.motor_front_left.move_voltage(-6000);
		conveyorLift.motor_front_right.move_voltage(-6000);
		pros::delay(300);
		conveyorLift.openStopperFront();
		conveyorLift.motor_front_left.move_voltage(0);
		conveyorLift.motor_front_right.move_voltage(0);
		conveyorLift.motor_back_left.move_absolute(150, 150);
		conveyorLift.motor_back_right.move_absolute(150, 150);
		pros::delay(500);
		conveyorLift.motor_back_left.move_voltage(6000);
		conveyorLift.motor_back_right.move_voltage(6000);
		pros::delay(200);
		conveyorLift.openStopperBack();
		conveyorLift.motor_back_left.move_voltage(0);
		conveyorLift.motor_back_right.move_voltage(0);
	}
	else if (fromPos == SIDE && toPos == MOGO)
	{
		conveyorLift.openStopperBack();
		conveyorLift.openStopperFront();
		pros::delay(130);
		conveyorLift.motor_front_left.move_relative(30, 100);
		conveyorLift.motor_front_right.move_relative(30, 100);
		pros::delay(100);
		conveyorLift.motor_back_left.move_relative(-530, 100);
		conveyorLift.motor_back_right.move_relative(-530, 100);
		conveyorLift.motor_front_left.move_relative(-400, 100);
		conveyorLift.motor_front_right.move_relative(-400, 100);
		pros::delay(500);
		conveyorLift.motor_back_left.move_voltage(-6000);
		conveyorLift.motor_back_right.move_voltage(-6000);
		conveyorLift.motor_front_left.move_voltage(-6000);
		conveyorLift.motor_front_right.move_voltage(-6000);
		pros::delay(300);
		conveyorLift.openStopperFront();
		conveyorLift.motor_front_left.move_voltage(0);
		conveyorLift.motor_front_right.move_voltage(0);
		conveyorLift.motor_back_left.move_absolute(150, 150);
		conveyorLift.motor_back_right.move_absolute(150, 150);
		pros::delay(500);
		conveyorLift.motor_back_left.move_voltage(6000);
		conveyorLift.motor_back_right.move_voltage(6000);
		pros::delay(200);
		conveyorLift.openStopperBack();
		conveyorLift.motor_back_left.move_voltage(0);
		conveyorLift.motor_back_right.move_voltage(0);
	}

	// Return to previous state after movement is complete
	robotState = previousState;
	setcurrentstate(robotState);
}

// Set current limits based on robot state

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
	chassis.setPose(-56.5, 0, 90);
	toggle();
	pros::delay(100);
	conveyorMotor.move_velocity(130);
	intake.move_velocity(600);
	chassis.moveToPoint(-51.5, 0, 1000, {}, true);
	pros::delay(1100);
	conveyorMotor.move_velocity(0);
	intake.move_velocity(0);
	allianceMode = true;
	toggle();
	chassis.moveToPoint(-57.5, 0, 1000, {false}, true);
	pros::delay(300);
	conveyorMotor.move_velocity(130);
	pros::delay(1000);
	conveyorMotor.move_velocity(0);
	chassis.moveToPoint(-55.5, 0, 1000, {}, true);
	chassis.turnToHeading(135, 500);
	toggle();
	pros::delay(400);
	conveyorMotor.move_velocity(130);
	intake.move_velocity(600);
	chassis.moveToPoint(-8.5, -34.5, 1900, {true, 60}, true);
	pros::delay(1900);
	conveyorMotor.move_velocity(0);
	intake.move_velocity(0);
	chassis.moveToPoint(-11.5, -32, 1000, {false, 60}, true);
	chassis.turnToHeading(25, 1500, {AngularDirection::AUTO, 70}, false);
	chassis.setPose({0, 0, 90});
	chassis.moveToPoint(-20, 0, 1500, {false, 50}, true);
	pros::delay(1200);
	ind.openHolder();
	conveyorMotor.move_velocity(130);
	pros::delay(1500);
	conveyorMotor.move_velocity(0);
	chassis.moveToPoint(-16, 0, 1000, {true, 60}, true);
	chassis.turnToHeading(160, 800, {}, false);
	chassis.setPose({0, 0, 90});
	conveyorMotor.move_velocity(130);
	intake.move_velocity(600);
	chassis.moveToPoint(20, 0, 2200, {true, 70}, true);
	pros::delay(2200);
	conveyorMotor.move_velocity(0);
	intake.move_velocity(0);
	chassis.moveToPoint(13, 0, 1000, {false, 60}, false);
	chassis.setPose({0, 0, 90});
	chassis.turnToHeading(150, 600, {}, false);
	chassis.setPose({0, 0, 90});
	chassis.moveToPoint(8, 0, 1000, {true}, false);
	chassis.moveToPoint(4, 0, 400, {false, 60}, false);
	intake.move_velocity(600);
	conveyorMotor.move_velocity(130);
	chassis.moveToPoint(11, 0, 1700, {true, 70}, true);
	pros::delay(1700);
	conveyorMotor.move_velocity(0);
	intake.move_velocity(0);
	chassis.moveToPoint(3, 0, 1500, {false, 70}, false);
	chassis.setPose({0, 0, 90});
	chassis.turnToHeading(204, 1300, {}, false);
	chassis.setPose({0, 0, 90});
	conveyorMotor.move_velocity(130);
	intake.move_velocity(600);
	chassis.moveToPoint(27, 0, 2500, {}, false);
	pros::delay(1000);
	conveyorMotor.move_velocity(0);
	intake.move_velocity(0);
	chassis.setPose({0, 0, 90});
	chassis.turnToHeading(108, 800, {}, false);
	conveyorMotor.move_velocity(130);
	intake.move_velocity(600);
	chassis.setPose({0, 0, 90});
	chassis.moveToPoint(20, 0, 2000, {true, 70}, false);
	pros::delay(2000);
	conveyorMotor.move_velocity(0);
	intake.move_velocity(0);
	chassis.setPose({0, 0, 90});
	chassis.turnToHeading(60, 1000, {}, false);
	chassis.setPose({0, 0, 90});
	chassis.moveToPoint(6, 0, 2000, {true, 70}, false);
	chassis.turnToHeading(34, 1500, {}, false);
	// chassis.moveToPoint(12, 0, 1000, {true, 60}, false);
	// chassis.turnToHeading(5, 800, {}, false);
	// conveyorMotor.move_velocity(140);
	// intake.move_velocity(600);
	// chassis.setPose({0, 0, 90});
	// chassis.moveToPoint(12.5, 0, 1700, {true, 60}, false);
	// pros::delay(1700);
	// conveyorMotor.move_velocity(0);
	// intake.move_velocity(0);
	// ind.openMover();
	// pros::delay(200);
	// chassis.setPose({0, 0, 90});
	// chassis.turnToHeading(55, 1000, {}, false);
	// chassis.turnToHeading(70, 1400, {}, false);
	// ind.openMover();
	// pros::delay(200);
	// chassis.setPose({0, 0, 90});
	// conveyorMotor.move_velocity(140);
	// intake.move_velocity(600);
	// chassis.moveToPoint(6, 0, 1000, {true, 60}, false);
	// pros::delay(1000);
	// conveyorMotor.move_velocity(0);
	// intake.move_velocity(0);
	// chassis.setPose({0, 0, 90});
	// chassis.moveToPoint(-7, 0, 1200, {false, 80}, false);
	// chassis.turnToHeading(135, 1000, {}, false);
	// chassis.setPose({0, 0, 90});
	// chassis.moveToPoint(-10, 0, 1500, {false, 60}, false);
	// chassis.turnToHeading(250, 1500, {}, false);
	// chassis.setPose({0, 0, 90});
	// chassis.moveToPoint(-12, 0, 1500, {false, 60}, false);
	// ind.openHolder();
	// pros::delay(200);
	// chassis.moveToPoint(0, 0, 1500, {true, 60}, false);
}

// The match function has the main calls you would do for an autonomous routine besides the non-drivebase motor calls
void matchJ()
{
	// Score on alliance stake
	chassis.setPose(-56.5, 0, 90);
	toggle();
	pros::delay(100);
	conveyorMotor.move_velocity(130);
	intake.move_velocity(600);
	chassis.moveToPoint(-50.5, 0, 1100, {}, true);
	pros::delay(1100);
	conveyorMotor.move_velocity(0);
	intake.move_velocity(0);
	allianceMode = true;
	toggle();
	chassis.moveToPoint(-57.5, 0, 1000, {false}, true);
	pros::delay(300);
	conveyorMotor.move_velocity(130);
	pros::delay(1300);
	conveyorMotor.move_velocity(0);
	chassis.moveToPoint(-55.5, 0, 1000, {}, true);
	chassis.turnToHeading(135, 2500);
	toggle();
	pros::delay(400);
	// Go and pick up ring
	conveyorMotor.move_velocity(130);
	intake.move_velocity(600);
	chassis.moveToPoint(-8.5, -34.5, 1900, {true, 50}, true);
	pros::delay(2200);
	conveyorMotor.move_velocity(0);
	intake.move_velocity(0);
	chassis.moveToPoint(-11.5, -32, 1000, {false, 50}, true);
	chassis.turnToHeading(25, 2500, {AngularDirection::CCW_COUNTERCLOCKWISE, 50}, false);
	chassis.setPose({0, 0, 90});
	chassis.moveToPoint(-20, 0, 1500, {false, 50}, true);
	pros::delay(1200);
	ind.openHolder();
	conveyorMotor.move_velocity(130);
	pros::delay(1500);
	conveyorMotor.move_velocity(0);
	// Prepare to get third ring
	chassis.moveToPoint(-13, 0, 1000, {true, 50}, true);
	chassis.turnToHeading(33, 2500, {}, false);
	chassis.setPose({0, 0, 90});
	chassis.moveToPoint(-8, 0, 3000, {false, 50}, true);
	chassis.turnToHeading(204, 3500, {}, false);
	chassis.setPose({0, 0, 90});
	conveyorMotor.move_velocity(130);
	intake.move_velocity(600);
	chassis.moveToPoint(13, 0, 2500, {}, false);
	pros::delay(1500);
	conveyorMotor.move_velocity(0);
	intake.move_velocity(0);
	// Go for fourth ring
	chassis.moveToPoint(0, 0, 2000, {false, 50}, false);
	chassis.turnToHeading(130, 1000, {}, false);
	chassis.setPose({0, 0, 90});
	chassis.moveToPoint(10, 0, 1500, {true, 50}, false);
	chassis.moveToPoint(4, 0, 400, {false, 50}, false);
	intake.move_velocity(600);
	conveyorMotor.move_velocity(130);
	chassis.moveToPoint(14, 0, 1700, {true, 50}, true);
	pros::delay(2100);
	conveyorMotor.move_velocity(0);
	intake.move_velocity(0);
	// Prepare to giet another ring
	chassis.moveToPoint(-25, 0, 6000, {false, 50}, false);
	chassis.turnToHeading(186, 2500, {}, false);
	chassis.setPose({0, 0, 90});
	conveyorMotor.move_velocity(130);
	intake.move_velocity(600);
	chassis.moveToPoint(15, 0, 2500, {true, 50}, true);
	pros::delay(3000);
	conveyorMotor.move_velocity(0);
	intake.move_velocity(0);
	// Place mobile goal in corner
	chassis.setPose({0, 0, 90});
	chassis.turnToHeading(270, 2500, {}, false);
	pros::delay(500);
	ind.openHolder();
	chassis.setPose({0, 0, 90});
	chassis.moveToPoint(-20, 0, 2000, {false, 100}, false);
	chassis.moveToPoint(0, 0, 2000, {true, 50}, false);
	chassis.turnToHeading(140, 2500, {AngularDirection::CW_CLOCKWISE, 60}, false);
	chassis.setPose({0, 0, 90});
	chassis.moveToPoint(-21, 0, 2500, {false, 60}, false);
	chassis.setPose({0, 0, 90});
	chassis.moveToPoint(72, 0, 4000, {true, 60}, false);
	chassis.turnToHeading(225, 2500, {AngularDirection::CW_CLOCKWISE, 60}, false);
	chassis.setPose({0, 0, 90});
	chassis.moveToPoint(-24, 0, 3000, {false, 60}, true);
	pros::delay(1500);
	ind.openHolder();
	pros::delay(1500);
	chassis.setPose({0, 0, 90});
	chassis.turnToHeading(45, 1500, {}, false);
	chassis.setPose({0, 0, 90});
	chassis.moveToPoint(16, 0, 2000, {true, 60}, false);
	chassis.moveToPoint(8, 0, 400, {false, 60}, false);
	intake.move_velocity(600);
	conveyorMotor.move_velocity(130);
	chassis.moveToPoint(18, 0, 1700, {true, 70}, true);
	pros::delay(2100);
	conveyorMotor.move_velocity(0);
	intake.move_velocity(0);
	chassis.setPose({0, 0, 90});
	chassis.turnToHeading(210, 2500, {AngularDirection::CW_CLOCKWISE, 127}, false);
	chassis.setPose({0, 0, 90});
	chassis.moveToPoint(-40, 0, 4000, {false, 127}, false);
}

void autonomous_skills()
{
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
