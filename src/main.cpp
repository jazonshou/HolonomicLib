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

Trajectory path = {
        {0.0001,0,0.0018,0},
        {0.0003,0,0.0107,0.0251},
        {0.0009,0,0.0322,0.0603},
        {0.0019,0,0.068,0.1005},
        {0.0033,-0.0001,0.1181,0.1407},
        {0.0051,-0.0001,0.1826,0.1809},
        {0.0073,-0.0001,0.2596,0.2206},
        {0.0099,-0.0002,0.3526,0.2603},
        {0.0129,-0.0003,0.46,0.3005},
        {0.0163,-0.0003,0.5818,0.3405},
        {0.0201,-0.0004,0.7178,0.3805},
        {0.0243,-0.0005,0.8681,0.4204},
        {0.0289,-0.0007,1.0328,0.4602},
        {0.0339,-0.0008,1.2136,0.5},
        {0.0393,-0.001,1.4105,0.5402},
        {0.0451,-0.0012,1.6217,0.5803},
        {0.0513,-0.0015,1.8473,0.6202},
        {0.0579,-0.0018,2.0907,0.6602},
        {0.0649,-0.0021,2.3485,0.7002},
        {0.0723,-0.0025,2.6223,0.7401},
        {0.0801,-0.0029,2.9123,0.7801},
        {0.0883,-0.0034,3.2202,0.8201},
        {0.0968,-0.0039,3.5442,0.8601},
        {0.1058,-0.0045,3.8843,0.9001},
        {0.1152,-0.0052,4.2441,0.9401},
        {0.125,-0.0059,4.62,0.9801},
        {0.1351,-0.0068,5.0138,1.02},
        {0.1457,-0.0077,5.4273,1.06},
        {0.1566,-0.0087,5.8587,1.1},
        {0.168,-0.0099,6.3098,1.14},
        {0.1797,-0.0112,6.7823,1.18},
        {0.1918,-0.0126,7.2728,1.2201},
        {0.2043,-0.0141,7.7847,1.26},
        {0.2172,-0.0158,8.3181,1.3},
        {0.2305,-0.0177,8.873,1.34},
        {0.2441,-0.0197,9.4494,1.38},
        {0.2582,-0.022,10.0509,1.42},
        {0.2725,-0.0244,10.6738,1.46},
        {0.2873,-0.0271,11.3218,1.5},
        {0.3024,-0.03,11.9966,1.54},
        {0.3179,-0.0332,12.6947,1.58},
        {0.3337,-0.0366,13.4196,1.62},
        {0.3499,-0.0404,14.1732,1.66},
        {0.3664,-0.0445,14.9537,1.7},
        {0.3832,-0.0489,15.7627,1.74},
        {0.4004,-0.0536,16.6022,1.78},
        {0.4179,-0.0588,17.474,1.82},
        {0.4356,-0.0644,18.3761,1.86},
        {0.4536,-0.0704,19.3105,1.9},
        {0.4719,-0.0769,20.2789,1.94},
        {0.4904,-0.0839,21.2813,1.98},
        {0.5092,-0.0914,22.3195,2.02},
        {0.5281,-0.0995,23.3935,2.06},
        {0.5472,-0.1082,24.5051,2.1},
        {0.5665,-0.1175,25.6561,2.14},
        {0.5859,-0.1275,26.8446,2.18},
        {0.6053,-0.1381,28.0726,2.22},
        {0.6248,-0.1496,29.3399,2.26},
        {0.6444,-0.1617,30.6484,2.3},
        {0.6638,-0.1747,31.9963,2.34},
        {0.6832,-0.1884,33.3799,2.3717},
        {0.702,-0.2028,34.7779,2.3687},
        {0.7204,-0.2176,36.1866,2.364},
        {0.7383,-0.233,37.6097,2.3619},
        {0.7557,-0.249,39.0453,2.3622},
        {0.7726,-0.2655,40.4916,2.3652},
        {0.789,-0.2826,41.9522,2.3708},
        {0.805,-0.3003,43.4272,2.3791},
        {0.8204,-0.3185,44.9129,2.3902},
        {0.8354,-0.3373,46.4147,2.4042},
        {0.8499,-0.3567,47.9308,2.4212},
        {0.864,-0.3767,49.4613,2.4413},
        {0.8776,-0.3972,51.0078,2.4647},
        {0.8908,-0.4183,52.5723,2.491},
        {0.9035,-0.4399,54.1403,2.5},
        {0.9156,-0.4618,55.7066,2.5},
        {0.9272,-0.4839,57.2693,2.5},
        {0.9382,-0.5063,58.8284,2.5},
        {0.9487,-0.529,60.3857,2.5},
        {0.9588,-0.5519,61.9376,2.5},
        {0.9684,-0.575,63.4841,2.5},
        {0.9775,-0.5983,65.0253,2.5},
        {0.9863,-0.6217,66.5629,2.5},
        {0.9947,-0.6452,68.0952,2.5},
        {1.0027,-0.6689,69.6221,2.5},
        {1.0103,-0.6927,71.1436,2.5},
        {1.0177,-0.7166,72.6615,2.5},
        {1.0248,-0.7406,74.174,2.5},
        {1.0316,-0.7647,75.683,2.5},
        {1.0381,-0.7888,77.1866,2.5},
        {1.0444,-0.8129,78.6884,2.5},
        {1.0506,-0.8372,80.1866,2.5},
        {1.0565,-0.8615,81.6831,2.5},
        {1.0623,-0.8858,83.1759,2.5},
        {1.068,-0.9101,84.667,2.5},
        {1.0736,-0.9345,86.1581,2.5},
        {1.079,-0.9589,87.6474,2.5},
        {1.0844,-0.9833,89.1366,2.5},
        {1.0897,-1.0078,90.6259,2.5},
        {1.0951,-1.0322,92.1152,2.5},
        {1.1003,-1.0566,93.6063,2.5},
        {1.1057,-1.081,95.0991,2.5},
        {1.111,-1.1055,96.5938,2.5},
        {1.1164,-1.1299,98.0902,2.5},
        {1.1218,-1.1543,99.592,2.5},
        {1.1274,-1.1787,101.0974,2.5},
        {1.1331,-1.203,102.6064,2.5},
        {1.1389,-1.2273,104.1189,2.5},
        {1.1449,-1.2516,105.6386,2.5},
        {1.151,-1.2758,107.1637,2.5},
        {1.1574,-1.3,108.696,2.5},
        {1.164,-1.3241,110.2336,2.5},
        {1.1709,-1.3481,111.7783,2.5},
        {1.178,-1.3721,113.3321,2.5},
        {1.1855,-1.3959,114.8929,2.5},
        {1.1933,-1.4197,116.4628,2.5},
        {1.2015,-1.4433,118.0415,2.5},
        {1.2101,-1.4668,119.6293,2.5},
        {1.2191,-1.4901,121.2242,2.5},
        {1.2286,-1.5132,122.8298,2.5},
        {1.2385,-1.5362,124.4444,2.5},
        {1.249,-1.5589,126.0661,2.5},
        {1.26,-1.5813,127.6968,2.5},
        {1.2716,-1.6034,129.3329,2.4999},
        {1.2837,-1.6251,130.9636,2.4784},
        {1.2962,-1.6461,132.5746,2.4416},
        {1.3092,-1.6664,134.1694,2.4092},
        {1.3225,-1.6861,135.7464,2.3808},
        {1.3364,-1.7051,137.3091,2.3563},
        {1.3507,-1.7236,138.8575,2.3353},
        {1.3655,-1.7414,140.3915,2.3177},
        {1.3808,-1.7586,141.913,2.3034},
        {1.3967,-1.7752,143.422,2.2922},
        {1.4129,-1.7911,144.9112,2.2762},
        {1.4294,-1.8062,146.3665,2.2378},
        {1.4461,-1.8205,147.786,2.1978},
        {1.463,-1.834,149.1661,2.1578},
        {1.4799,-1.8467,150.5104,2.1178},
        {1.497,-1.8586,151.8135,2.0778},
        {1.514,-1.8697,153.0772,2.0378},
        {1.5311,-1.8801,154.3034,1.9978},
        {1.5481,-1.8898,155.4883,1.9578},
        {1.565,-1.8989,156.6339,1.9178},
        {1.5818,-1.9073,157.742,1.8778},
        {1.5984,-1.9152,158.8106,1.8378},
        {1.6148,-1.9224,159.8416,1.7978},
        {1.6311,-1.9292,160.8369,1.7578},
        {1.6471,-1.9354,161.7945,1.7178},
        {1.6628,-1.9411,162.7182,1.6778},
        {1.6783,-1.9464,163.6078,1.6378},
        {1.6935,-1.9513,164.4634,1.5978},
        {1.7084,-1.9559,165.2868,1.5578},
        {1.723,-1.96,166.078,1.5178},
        {1.7373,-1.9638,166.8405,1.4778},
        {1.7513,-1.9673,167.5708,1.4378},
        {1.7649,-1.9705,168.2725,1.3978},
        {1.7781,-1.9735,168.9474,1.3578},
        {1.791,-1.9761,169.5935,1.3178},
        {1.8036,-1.9786,170.2129,1.2778},
        {1.8157,-1.9808,170.8072,1.2378},
        {1.8275,-1.9829,171.3764,1.1978},
        {1.839,-1.9847,171.9205,1.1578},
        {1.85,-1.9864,172.4414,1.1177},
        {1.8607,-1.988,172.9391,1.0777},
        {1.871,-1.9893,173.4134,1.0377},
        {1.8809,-1.9906,173.8645,0.9977},
        {1.8904,-1.9917,174.2959,0.9577},
        {1.8995,-1.9927,174.704,0.9177},
        {1.9082,-1.9936,175.0924,0.8778},
        {1.9166,-1.9945,175.4612,0.8377},
        {1.9245,-1.9952,175.8084,0.7977},
        {1.9321,-1.9958,176.1378,0.7577},
        {1.9392,-1.9964,176.4475,0.7177},
        {1.946,-1.997,176.7374,0.6777},
        {1.9523,-1.9974,177.0095,0.6377},
        {1.9583,-1.9978,177.2637,0.5976},
        {1.9639,-1.9982,177.4982,0.5576},
        {1.9691,-1.9985,177.7166,0.5176},
        {1.9738,-1.9988,177.917,0.4775},
        {1.9782,-1.999,178.0996,0.4374},
        {1.9822,-1.9992,178.2643,0.3974},
        {1.9857,-1.9994,178.4129,0.3574},
        {1.9889,-1.9995,178.5435,0.3174},
        {1.9917,-1.9997,178.6581,0.2775},
        {1.9941,-1.9998,178.7566,0.2373},
        {1.996,-1.9998,178.8371,0.1972},
        {1.9976,-1.9999,178.9015,0.1574},
        {1.9988,-2,178.9499,0.1173},
        {1.9996,-2,178.9821,0.0767},
        {2,-2,178.9982,0.0281}
    }
;




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

std::shared_ptr<ExpandedXDriveModel> model = std::static_pointer_cast<ExpandedXDriveModel> (chassis->getModel());

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
    imu.calibrate();
    pros::delay(2000);
    std::shared_ptr<AsyncHolonomicChassisController> hol = 
    AsyncHolonomicChassisControllerBuilder()
        .withOutput(chassis)
        .withControllers(xController, yController, thetaController)
        .build();

    // hol->setTarget({-10_in, 20_in, 90_deg});
    hol->setTarget(path);
    hol->waitUntilSettled();
    Controller master = Controller();
    while(true) {
        pros::lcd::print(1, "X: %f", hol->getPose().x.convert(inch));
        pros::lcd::print(2, "Y: %f", hol->getPose().y.convert(inch));
        pros::lcd::print(3, "Theta: %f", hol->getPose().theta.convert(degree));

        model->cartesian(master.getAnalog(ControllerAnalog::leftY), 
                         master.getAnalog(ControllerAnalog::leftX), 
                         master.getAnalog(ControllerAnalog::rightX), 
                         imu.get() * degree, 
                         0.05);

        pros::delay(10);
    }
}
