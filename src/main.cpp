#include "main.h"

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
}


void disabled() {}


void competition_initialize() {}


void autonomous() {}

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



void opcontrol() {
    std::shared_ptr<AsyncHolonomicChassisController> hol = AsyncHolonomicChassisControllerBuilder(chassis)
        .withDistPID(std::make_unique<IterativePosPIDController>(0.05, 0.0, 0.00065, 0.0, TimeUtilFactory::withSettledUtilParams(0.5, 10, 100_ms)))
        .withTurnPID(std::make_unique<IterativePosPIDController>(0.05, 0.0, 0.00065, 0.0, TimeUtilFactory::withSettledUtilParams(1, 30, 100_ms)))
        .build();

	hol->setTarget({2_ft, 2_ft, 45_deg}, true);

    while(true) {
        // std::cout << "weeeee" << std::endl;
        pros::delay(10);
    }
}
