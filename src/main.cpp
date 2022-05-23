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
ADIEncoder leftEnc = ADIEncoder('C', 'D');
ADIEncoder rightEnc = ADIEncoder('A', 'B');
ADIEncoder midEnc = ADIEncoder('E', 'F', true);
IMU imu = IMU(12);

std::shared_ptr<OdomChassisController> chassis = ChassisControllerBuilder()
    .withMotors(leftFront, rightFront, rightBack, leftBack)
    .withSensors(leftEnc, rightEnc, midEnc)
    .withDimensions(AbstractMotor::gearset::green, {{3.25_in, 10.25_in}, imev5GreenTPR})
    .withOdometry({{2.75_in, 13.5_in, 7_in, 2.75_in}, quadEncoderTPR})
    .buildOdometry();

std::shared_ptr<XDriveModel> model = std::static_pointer_cast<XDriveModel> (chassis->getModel());

// std::shared_ptr<AsyncHolonomicChassisController> hol = AsyncHolonomicChassisControllerBuilder()
//     .withOutput(chassis)
//     .withControllers
std::shared_ptr<IterativePosPIDController > xController = 
    std::make_shared<IterativePosPIDController> (0.037, 0.0, 0.00065, 0, TimeUtilFactory::withSettledUtilParams(2, 2, 100_ms));
std::shared_ptr<IterativePosPIDController> yController = 
    std::make_shared<IterativePosPIDController> (0.037, 0.0, 0.00065, 0, TimeUtilFactory::withSettledUtilParams(2, 2, 100_ms));
std::shared_ptr<IterativePosPIDController> thetaController = 
    std::make_shared<IterativePosPIDController> (0.037, 0.0, 0.00065, 0, TimeUtilFactory::withSettledUtilParams(2, 2, 100_ms));



void opcontrol() {
    std::shared_ptr<AsyncHolonomicChassisController> hol = 
    AsyncHolonomicChassisControllerBuilder()
        .withOutput(chassis)
        .withControllers(xController, yController, thetaController)
        .build();

    hol->setTarget({10_in, 0_in, 0_deg}, true);

    while(true) pros::delay(10);

	// Controller master = Controller();
	
    // while(true) {
    //     HolonomicWheelSpeeds speeds = HolonomicMath::move(master.getAnalog(ControllerAnalog::leftY), master.getAnalog(ControllerAnalog::leftX), master.getAnalog(ControllerAnalog::rightX), imu.get() * degree);
    //     (model->getTopLeftMotor())->moveVoltage(12000 * speeds.frontLeft);
    //     (model->getTopRightMotor())->moveVoltage(12000 * speeds.frontRight);
    //     (model->getBottomLeftMotor())->moveVoltage(12000 * speeds.backLeft);
    //     (model->getBottomRightMotor())->moveVoltage(12000 * speeds.backRight);

    //     pros::lcd::print(1, "Left Enc: %f", leftEnc.get());
    //     pros::lcd::print(2, "Mid Enc: %f", midEnc.get());
    //     pros::lcd::print(3, "Right Enc: %f", rightEnc.get());

    //     pros::lcd::print(5, "X: %f", chassis->getState().x.convert(inch));
    //     pros::lcd::print(6, "Y: %f", chassis->getState().y.convert(inch));
    //     pros::lcd::print(7, "Theta: %f", chassis->getState().theta.convert(degree));

    //     std::cout << "imu: " << imu.get() << std::endl;

    //     pros::delay(10);
    // }
}
