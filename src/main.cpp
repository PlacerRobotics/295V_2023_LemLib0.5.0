#include "main.h"
#include "lemlib/api.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drive motors
pros::Motor lF(1, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); // left motor front
pros::Motor lB1(2, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); // left motor back bottom
pros::Motor lB2(3, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES); // left motor back top
pros::Motor rF(9, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES); // right motor front
pros::Motor rB1(5, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES); // right motor back bottom
pros::Motor rB2(6, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); // right motor back top

pros::Motor catapultMotor(8, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); // situated in the back of the robot
pros::Motor intakeMotor(7, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); // situated in the front of the robot

// motor groups
pros::MotorGroup leftMotors({lF, lB1, lB2}); // left motor group
pros::MotorGroup rightMotors({rF, rB1, rB2}); // right motor group
pros::Motor_Group allMotors({lF,lB1, lB2, rF, rB1, rB2}); // all the motors in one group for general 

// Static files
ASSET(example_txt); // '.' replaced with "_" to make c++ happy
ASSET(path1_txt);
ASSET(path2_txt);
ASSET(path3_txt);
ASSET(path4_txt);
ASSET(path5_txt);

int auton = 3;

// Inertial Sensor on port 12
pros::Imu imu(12);

// Distance sensor for the catapult
pros::Distance cataDistance(11);

// Wings Pnematics
pros::ADIDigitalOut wing1('A');
pros::ADIDigitalOut wing2('B');

// Blocker/Lift Pneumatics
pros::ADIDigitalOut blockerPneumatics1('C');
pros::ADIDigitalOut blockerPneumatics2('D');

// Rotation Sensor on the Catapult
pros::Rotation catapultRotation(21);

// Catapult Task to make sure that the catapult goes and stay at the perfect angle 
pros::Task loadCatapultTask{ [] {
    while (pros::Task::notify_take(true, TIMEOUT_MAX)) {
        const int pullbackAngle = 5100; 

        // normal shot
        catapultMotor.move_voltage(-12000); // 85
        pros::delay(1000);

        while(catapultRotation.get_angle() <= pullbackAngle){
            pros::delay(10);
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
                              15, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              257.142857, // drivetrain rpm is 257.14
                              8 // chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(20, // proportional gain (kP)

                                            0, // integral gain (kI)

                                            40, // derivative gain (kD)

                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            2, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(6, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             30, // derivative gain (kD)
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

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    if (auton == 1){
        chassis.setPose(34.951, -68.602, 0);
        chassis.follow(path1_txt, 13, 1500, true, false);
        intakeMotor.move_voltage(12000);
        pros::delay(1000);
        intakeMotor.move_voltage(0);
        chassis.turnTo(-29.653, 68.602, 500);
        chassis.follow(path2_txt, 10, 1500);
        intakeMotor.move_voltage(-12000);
        pros::delay(1750);
        intakeMotor.move_voltage(0);
        chassis.follow(path3_txt, 10, 2000, false);
        wing1.set_value(1);
        wing2.set_value(1);
        // chassis.turnTo(11.161, 55.642, 750);
        // intakeMotor.move_voltage(12000);
    }
    if (auton == 2){
        // Point 1
        // chassis.moveToPose(-6, 48, 0, 2500);
        chassis.moveToPose(-6, 48, 0, 1500);
        // Point 2
        // chassis.turnTo(13, 48, 1000);
        // Trail 1
        // chassis.turnTo(3, 50, 500);
        chassis.turnTo(3, 48, 500);
        // chassis.moveToPose(13, 48, 90, 2500);
        // Trial 1
        // chassis.moveToPose(3, 50, 90, 1500);
        intakeMotor.move_voltage(12000);
        pros::delay(800);
        intakeMotor.move_voltage(0);
        chassis.turnTo(-10, 65, 750);
        chassis.moveToPose(-10, 65, -60, 1500);
        intakeMotor.move_voltage(-12000);
        pros::delay(2000);
        intakeMotor.move_voltage(0);
        chassis.turnTo(3,65, 750);
        intakeMotor.move_voltage(12000);
        pros::delay(1000);
        intakeMotor.move_voltage(0);
        // wing1.set_value(1);
        // wing2.set_value(1);
        chassis.turnTo(-14,65, 750);
        // chassis.moveToPose(3, 65, -90, 1500);
        // // intakeMotor.move_voltage(12000);
        // // pros::delay(1200);
        // // intakeMotor.move_voltage(0);
        // chassis.turnTo(-2,0, 300);
        // chassis.moveToPose(-2, 0, 0, 3000);
        // chassis.turnTo(-35, 2, 500);
        // chassis.moveToPose(-35, 2, -90, 750);

        /*
        // Point 1
        chassis.moveToPose(-6, 48, 0, 2000);
        // Point 2
        chassis.turnTo(3, 50, 500);
        chassis.moveToPose(3, 50, 90, 1500);
        intakeMotor.move_voltage(12000);
        pros::delay(1500);
        intakeMotor.move_voltage(0);
        chassis.turnTo(-14, 63, 300);
        chassis.moveToPose(-14, 63, -60, 1000);
        intakeMotor.move_voltage(-12000);
        pros::delay(2000);
        intakeMotor.move_voltage(0);
        chassis.turnTo(3,62, 500);
        chassis.moveToPose(3, 62, -90, 1500);
        intakeMotor.move_voltage(12000);
        pros::delay(1200);
        intakeMotor.move_voltage(0);
        chassis.turnTo(-2,0, 300);
        chassis.moveToPose(-2, 0, 0, 3000);
        chassis.turnTo(-35, 2, 500);
        chassis.moveToPose(-35, 2, -90, 750);
        */
    } if (auton == 3){
        chassis.setPose(-35.684, -67.436, 0);
        chassis.follow(path4_txt, 10, 2000, false, false);
        catapultMotor.move_voltage(-12000);
        pros::delay(4500);
        catapultMotor.move_voltage(0);
        chassis.waitUntilDone();
        // chassis.follow(path5_txt, 10, 2000, false, false);
        // wing1.set_value(1);
        // wing2.set_value(1);
    }
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
        allMotors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
        // move the chassis with curvature drive
        chassis.curvature(leftY, rightX, 0.3);
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
    //    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
    //     intakePiston1.set_value(1);
    //     intakePiston2.set_value(1);
    //    } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
    //     intakePiston1.set_value(0);
    //     intakePiston2.set_value(0);
    //    }
    }
}