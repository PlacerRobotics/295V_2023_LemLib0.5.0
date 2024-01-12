#include "main.h"
#include "lemlib/api.hpp"
// #include "global.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drive motors
pros::Motor lF(1, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor lB1(2, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor lB2(3, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rF(9, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rB1(5, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rB2(6, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor catapultMotor(8, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor intakeMotor(7, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

// motor groups
pros::MotorGroup leftMotors({lF, lB1, lB2}); // left motor group
pros::MotorGroup rightMotors({rF, rB1, rB2}); // right motor group
pros::Motor_Group allMotors({lF,lB1, lB2, rF, rB1, rB2});

// Inertial Sensor on port 2
pros::Imu imu(12);

// Distance sensor
pros::Distance cataDistance(11);

// Wings Pnematics
pros::ADIDigitalOut wing1('A');
pros::ADIDigitalOut wing2('B');

// Intake Pneumatics
pros::ADIDigitalOut intakePiston1('C');
pros::ADIDigitalOut intakePiston2('D');

// Rotation Sensor
pros::Rotation catapultRotation(10);

// Catapult
pros::Task loadCatapultTask{ [] {
    while (pros::Task::notify_take(true, TIMEOUT_MAX)) {
        const int pullbackAngle = 6000; 

        // normal shot
        catapultMotor.move_voltage(-12000); // 85
        pros::delay(1000);

        while(catapultRotation.get_angle() <= pullbackAngle){
            pros::delay(20);
            if(catapultRotation.get_angle() > pullbackAngle-1500){
                catapultMotor.move_voltage(-9000);
            }
        }

        catapultMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        catapultMotor.move_voltage(0);

        pros::delay(20);
    }
}
};

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 15, reversed (negative signs don't work due to a pros bug)
// pros::Rotation horizontalEnc(15, true);
// horizontal tracking wheel. 2.75" diameter, 3.7" offset, back of the robot (negative)
// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_4, -3.7);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              257.142857, // drivetrain rpm is 257.14
                              2 // chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
// note that in this example we use internal motor encoders (IMEs), so we don't pass vertical tracking wheels
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1, set to null for the time being
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy
ASSET(path1_txt);
ASSET(path2_txt);
ASSET(path3_txt);
ASSET(path4_txt);
ASSET(path5_txt);

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    // example movement: Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    // example movement: Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has travelled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // example movement: Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnTo(45, -45, 1000, true, 60);
    // example movement: Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has travelled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Travelled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.curvature(leftY, rightX);
        // delay to save resources
        pros::delay(10);
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            intakeMotor.move_voltage(12000);
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            intakeMotor.move_voltage(-12000);
        }
        else{
            intakeMotor.move_voltage(0);
        }
       if(cataDistance.get() <= 50){
        loadCatapultTask.notify();
       } 
       if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
        wing1.set_value(1);
        wing2.set_value(1);
       } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
        wing1.set_value(0);
        wing2.set_value(0);
       }
       if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
        intakePiston1.set_value(1);
        intakePiston2.set_value(1);
       } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
        intakePiston1.set_value(0);
        intakePiston2.set_value(0);
       }
    }
}