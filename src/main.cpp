#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
using namespace okapi;

void initialize() {
	pros::lcd::initialize();
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
void autonomous() {}

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
Motor leftFront = Motor(19, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor rightFront = Motor(11, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor leftBack = Motor(10, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor rightBack = Motor(1, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
RotationSensor leftEnc = RotationSensor(5);
RotationSensor rightEnc = RotationSensor(6);
RotationSensor midEnc = RotationSensor(7);
IMU imu = IMU(12);

std::shared_ptr<OdomChassisController> chassis = ChassisControllerBuilder()
    .withMotors(leftFront, rightFront, rightBack, leftBack)
    .withSensors(leftEnc, rightEnc, midEnc)
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 10.25_in}, imev5GreenTPR})
    .withOdometry({{2.75_in, 7_in, 1_in, 2.75_in}, quadEncoderTPR})
    .buildOdometry();

std::shared_ptr<XDriveModel> model = std::dynamic_pointer_cast<XDriveModel> (chassis->getModel());

void opcontrol() {
    imu.calibrate();
    pros::delay(2000);

	Controller master = Controller();
	
    while(true) {
        HolonomicWheelSpeeds speeds = HolonomicMath::move(master.getAnalog(ControllerAnalog::leftY), master.getAnalog(ControllerAnalog::leftX), master.getAnalog(ControllerAnalog::rightX), imu.get() * degree);
        (model->getTopLeftMotor())->moveVoltage(12000 * speeds.frontLeft);
        (model->getTopRightMotor())->moveVoltage(12000 * speeds.frontRight);
        (model->getBottomLeftMotor())->moveVoltage(12000 * speeds.backLeft);
        (model->getBottomRightMotor())->moveVoltage(12000 * speeds.backRight);

        pros::lcd::print(1, "Front Left: %f", speeds.frontLeft);
        pros::lcd::print(2, "Front Right: %f", speeds.frontRight);
        pros::lcd::print(3, "Back Left: %f", speeds.backLeft);
        pros::lcd::print(4, "Back Right: %f", speeds.backRight);

        std::cout << "imu: " << imu.get() << std::endl;

        pros::delay(10);
    }
}
