#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"
#include <newlib.h>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drive motors
pros::Motor lF(10, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); // left motor front
pros::Motor lM(9, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); // left motor back bottom
pros::Motor lB(8, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); // left motor back top
pros::Motor rF(20, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES); // right motor front
pros::Motor rM(19, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES); // right motor back bottom
pros::Motor rB(18, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES); // right motor back top

pros::Motor catapultMotor(5, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); // situated in the back of the robot
pros::Motor intakeMotor(17, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); // situated in the front of the robot
pros::Motor catapultMotor2(0, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
// motor groups
pros::MotorGroup leftMotors({lF, lM, lB}); // left motor group
pros::MotorGroup rightMotors({rF, rM, rB}); // right motor group
pros::Motor_Group allMotors({lF, lM, lB, rF, rM, rB}); // all the motors in one group for general 

// Static files
ASSET(path_skills1_txt);
ASSET(path_skills2_txt);
ASSET(path_skills3_txt);
ASSET(path_skills4_txt);
ASSET(path_skills5_txt);
ASSET(path_skills6_txt);
ASSET(path_skills7_txt);
ASSET(path_skills8_txt);
ASSET(path_skills9_txt);
ASSET(cs1_txt);
ASSET(cs2_txt);
ASSET(cs3_txt);
ASSET(fs1_txt);
ASSET(fs2_txt);
ASSET(fs3_txt);
ASSET(fs4_txt);
ASSET(fs5_txt);
ASSET(fs6_txt);
int auton = 5;

// Inertial Sensor on port 12
pros::Imu imu(11);

// Distance sensor for the catapult
pros::Distance cataDistance(3);

// Wings Pnematics
pros::ADIDigitalOut wingRight('B'); // right side wing
pros::ADIDigitalOut wingLeft('A'); // left side wing
pros::ADIDigitalOut wingFrontRight('D'); // right side wing
pros::ADIDigitalOut wingFrontLeft('C'); // left side wing

// Blocker/Lift Pneumatics
pros::ADIDigitalOut blockerPneumatics1('C');
pros::ADIDigitalOut blockerPneumatics2('D');

// Rotation Sensor on the Catapult
pros::Rotation catapultRotation(14);

// Catapult Task to make sure that the catapult goes and stay at the perfect angle 
pros::Task loadCatapultTask{ [] {
    while (pros::Task::notify_take(true, TIMEOUT_MAX)) {
        const int pullbackAngle = 14777; 

        // normal shot
        catapultMotor.move_voltage(-12000);// 85
        catapultMotor2.move_voltage(-12000);
        pros::delay(1000);

        while(catapultRotation.get_angle() <= pullbackAngle){
            pros::delay(10);
            if(catapultRotation.get_angle() > pullbackAngle-3000){
                catapultMotor.move_voltage(-10000);
                catapultMotor2.move_voltage(-10000);
            }
        }

        catapultMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        catapultMotor.move_voltage(0);
        catapultMotor2.move_voltage(0);

        pros::delay(20);
    }
}
};

pros::Task loadCatapult{ [] {
    while (pros::Task::notify_take(true, TIMEOUT_MAX)) {
        const int pullbackAngle = 14777; 

        // normal shot
        catapultMotor.move_voltage(-12000);// 85
        pros::delay(1000);

        while(catapultRotation.get_angle() <= pullbackAngle){
            pros::delay(10);
            if(catapultRotation.get_angle() > pullbackAngle-3000){
                catapultMotor.move_voltage(-10000);
            }
        }

        catapultMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        catapultMotor.move_voltage(0);

        pros::delay(20);
    }
}
};

pros::Task elevationTask{ [] {
    while (pros::Task::notify_take(true, TIMEOUT_MAX)) {
        const int pullbackAngle = 13254; 

        // normal shot
        catapultMotor.move_voltage(-12000); // 85
        pros::delay(1000);

        while(catapultRotation.get_angle() <= pullbackAngle){
            pros::delay(10);
            if(catapultRotation.get_angle() > pullbackAngle-3000){
                catapultMotor.move_voltage(-10000);
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
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 257.14
                              8// chase power is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(20, // proportional gain (kP)

                                            0, // integral gain (kI)

                                            10, // derivative gain (kD)

                                            2, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            2, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            127 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             2, // anti windup
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
            pros::lcd::print(3, "Rotation: %d", catapultRotation.get_angle());
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
        allMotors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
        if(auton == 3){
            // Auton skills
            chassis.setPose(-34.583, -61.221, 180);
            chassis.follow(path_skills1_txt, 14, 1200, false);
            chassis.moveToPoint(-60.922, -38, 500, true);
            chassis.moveToPoint(-60.922, -28, 500, false, 127.0F);
            chassis.moveToPoint(-58.451, -42.194, 800);
            chassis.turnTo(46, -6, 500, false);
            // chassis.turnTo(-54.946, -32.68, 800);
            // chassis.follow(path_skills2_txt, 14, 1500, true);
            wingFrontLeft.set_value(1);
            chassis.waitUntilDone();
            // pros::delay(500);
            catapultMotor.move_voltage(-12000);
            catapultMotor2.move_voltage(-12000);
            for(int i = 0; i < 5; i++){
                pros::delay(5000);
                leftMotors.move_voltage(-1000);
                pros::delay(50);
                leftMotors.move_voltage(0);
            }
            // allMotors.move_voltage(2000);
            pros::delay(3000);
            // pros::delay(2000);
            // allMotors.move_voltage(0);
            wingFrontLeft.set_value(0);
            loadCatapultTask.notify();
            chassis.moveToPoint(-33, -35, 700, false);
            chassis.turnTo(-1.369, -34.683, 500, false);
            // chassis.waitUntilDone();            
            chassis.follow(path_skills3_txt, 14, 1800, false);
            pros::delay(700);
            wingRight.set_value(1);
            pros::delay(700);
            wingLeft.set_value(0);
            chassis.waitUntilDone();
            chassis.moveToPoint(-10.862, 30, 800);
            chassis.moveToPoint(-10.862, 42, 1000, false);
            chassis.moveToPoint(-11, 30, 800);
            wingLeft.set_value(0);
            wingRight.set_value(0);
            chassis.turnTo(-51, 46, 1200, false);
            chassis.follow(path_skills4_txt, 14, 1000, false);
            chassis.waitUntilDone();
            chassis.turnTo(-44.013, 65.337, 500, false);
            chassis.follow(path_skills5_txt, 14, 800, false);
            wingRight.set_value(1);
            pros::delay(300);
            wingRight.set_value(0);
            chassis.waitUntilDone();
            chassis.turnTo(-6.209, 62.29, 500, false);
            chassis.follow(path_skills6_txt, 14, 2000, false);
            chassis.waitUntilDone();
            chassis.turnTo(59.405, 46.877, 500, false);
            chassis.follow(path_skills7_txt, 14, 1000, false);
            pros::delay(300);
            wingRight.set_value(1);
            pros::delay(200);
            wingRight.set_value(0);
            chassis.waitUntilDone();
            chassis.moveToPoint(65, 42, 700, true);
            chassis.waitUntilDone();
            chassis.moveToPoint(70, 26, 700, false);
            chassis.waitUntilDone();
            chassis.moveToPoint(65, 42, 700, true);
            chassis.moveToPoint(70, 26, 700, false);
            chassis.waitUntilDone();
            // chassis.moveToPoint(68, 38, 800);
            // chassis.moveToPoint(68, 24.855, 1000, false);
            chassis.moveToPoint(60, 38, 700);
            chassis.waitUntilDone();
            chassis.turnTo(35.474, 32.096, 700, false);
            chassis.follow(path_skills8_txt, 14, 3000, false);
            chassis.waitUntilDone();
            // chassis.turnTo(11, 12, 300, false);
            // chassis.moveToPoint(11, 12, 600, false);
            chassis.turnTo(42, 8, 700, false);
            chassis.waitUntilDone();
            wingLeft.set_value(1);
            wingRight.set_value(1);
            chassis.moveToPoint(42, 8, 1000, false);
            chassis.moveToPoint(11, 28.408, 1000, true);
            pros::delay(100);
            wingLeft.set_value(0);
            wingRight.set_value(0);
            chassis.turnTo(11, -8, 600, false);
            chassis.moveToPoint(11, -8, 1000, false);
            chassis.turnTo(42, -8, 600, false);
            chassis.waitUntilDone();
            wingLeft.set_value(1);
            wingRight.set_value(1);
            chassis.moveToPoint(42, -8, 1000, false);
            chassis.moveToPoint(32, -8, 700);
            chassis.moveToPoint(42, -8, 700, false);
            chassis.moveToPoint(32, -8, 700);
            chassis.waitUntilDone();
            chassis.moveToPoint(11, -8, 700);
            pros::delay(100);
            wingLeft.set_value(0);
            wingRight.set_value(0);
            chassis.turnTo(11, -33, 600, false);
            chassis.moveToPoint(11, -33, 1200, false);
            chassis.turnTo(33, -40, 500, false);
            chassis.moveToPoint(34, -42, 700, false);
            chassis.turnTo(48.707, -52.93, 500, false);
            chassis.follow(path_skills9_txt, 14, 1200, false);
            chassis.moveToPoint(60.25, -45, 700);
            chassis.moveToPoint(60.25, -28, 700, false);
            chassis.moveToPoint(60.25, -45, 700);
        }
        if (auton == 2)
        {
            // Close Side Auton
            chassis.setPose(-36.92, -64.393, 0);
            chassis.follow(cs1_txt, 14, 1100, true);
            wingFrontLeft.set_value(1);
            intakeMotor.move_voltage(-12000); // intake
            pros::delay(200);
            wingFrontLeft.set_value(0);
            pros::delay(1200);
            // intakeMotor.move_voltage(0);
            chassis.waitUntilDone();
            chassis.turnTo(-42.761, -55.88, 500, false);
            chassis.follow(cs2_txt, 14, 1000, false);
            chassis.waitUntilDone();
            chassis.turnTo(-59.869, -27.256, 700, false);
            chassis.follow(cs3_txt, 14, 1000, false);
            chassis.waitUntilDone();
            // In and out
            chassis.moveToPoint(-62, -38, 700, true);
            chassis.moveToPoint(-62, -24, 500, false);
            chassis.moveToPoint(-62, -30, 700, true);
            // Win Point Tribal
            chassis.turnTo(-50, -59, 700, true);
            chassis.moveToPoint(-50, -59, 1200, true);
            chassis.waitUntilDone();
            wingRight.set_value(1);
            chassis.turnTo(0, 0, 2000, true);
            chassis.waitUntilDone();
            chassis.turnTo(0, -60, 500, true);
            // chassis.turnTo(0, -58, 1000, false);
            // chassis.turnTo(0, -58, 500, false);
            // chassis.turnTo(0, -59, 1000, true);
            chassis.waitUntilDone();
            chassis.moveToPoint(-16, -60, 1500, true);
            pros::delay(300);
            wingRight.set_value(0);
            pros::delay(300);
            intakeMotor.move_voltage(12000);

        } if(auton == 1){
            // Far Side Auton
            chassis.setPose(35.685, -54.879, 0);
            // // chassis.moveToPoint(35.685, -40, 200);
            // // chassis.moveToPoint(35.685, -54.879, 200, false);
            // chassis.turnTo(58, -25, 200, true, 127, false);
            // intakeMotor.move_voltage(12000);
            // pros::delay(300);
            // // intakeMotor.move_voltage(0);
            // chassis.turnTo(31.178, -33.014, 300);
            chassis.follow(fs1_txt, 14, 1200, true);
            wingFrontRight.set_value(1);
            pros::delay(300);
            wingFrontRight.set_value(0);
            // intakeMotor.move_voltage(-12000); // intake
            // wingRight.set_value(1);
            // pros::delay(300);
            // wingRight.set_value(0);
            pros::delay(1200);
            chassis.follow(fs2_txt, 14, 1000, false);
            chassis.turnTo(58, -25, 750, true, 127, false);
            intakeMotor.move_voltage(12000); // outtake
            pros::delay(500);
            intakeMotor.move_voltage(0);
            chassis.turnTo(24.001, -54.378, 800);
            chassis.follow(fs3_txt, 14, 1200);
            intakeMotor.move_voltage(-12000);
            pros::delay(1300);
            chassis.turnTo(18.603, -61.399, 1000);
            chassis.follow(fs4_txt, 14, 1000);
            chassis.turnTo(58, -25, 500);
            intakeMotor.move_voltage(12000);
            pros::delay(500);
            intakeMotor.move(0);
            chassis.turnTo(42.528, -54.044, 800, false); 
            // chassis.moveToPose(52.709, -52.208, 220, 700, {.forwards = false});      
            chassis.follow(fs5_txt, 14, 500, false);
            wingLeft.set_value(1);
            chassis.turnTo(0, 0, 700, false, 127, false);
            wingLeft.set_value(0);
            wingRight.set_value(1);
            chassis.turnTo(61.722, -43.863, 700, false);
            // chassis.moveToPoint(58.217, -46.7, 700);
            chassis.follow(fs6_txt, 14, 800, false);
            chassis.moveToPoint(60, -42, 1000, true, 127, false);
            chassis.moveToPoint(66, -24, 700, false);
            pros::delay(200);
            wingRight.set_value(0);
        }
        if (auton == 4){
            chassis.setPose(-36.92, -64.393, 0);
            chassis.follow(cs1_txt, 14, 1100, true);
            wingLeft.set_value(1);
            intakeMotor.move_voltage(-12000); // intake
            pros::delay(400);
            wingLeft.set_value(0);
            pros::delay(1200);
            // intakeMotor.move_voltage(0);
            chassis.waitUntilDone();
            chassis.turnTo(-70, -9, 700);
            wingLeft.set_value(1);
            wingRight.set_value(1);

        }
        if(auton == 5){
            allMotors.move_voltage(-12000);
            pros::delay(1000);
            allMotors.move_voltage(0);
            pros::delay(100);
            allMotors.move_voltage(12000);
            pros::delay(300);
            allMotors.move_voltage(0);
            pros::delay(100);
            allMotors.move_voltage(-12000);
            pros::delay(500);
            allMotors.move_voltage(0);
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
        catapultRotation.set_reversed(0);
        // allMotors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.curvature(leftY, rightX);
        // delay to save resources
        pros::delay(10);
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            intakeMotor.move_voltage(12000); // outake
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            intakeMotor.move_voltage(-12000); // intake
        }
        else{
            intakeMotor.move_voltage(0);
        }
       if(cataDistance.get() <= 30){
        loadCatapultTask.notify();
       } 
       if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
        wingRight.set_value(1);
        wingLeft.set_value(1);
       } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
        wingRight.set_value(0);
        wingLeft.set_value(0);
       } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        loadCatapultTask.notify();
       }
       if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
        elevationTask.notify();
       }
       if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        catapultMotor.move_voltage(-12000 );
        pros::delay(700);
        catapultMotor.move_voltage(0);
       }
       if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
        wingFrontLeft.set_value(1);
        wingFrontRight.set_value(1);
       } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
        wingFrontRight.set_value(0);
        wingFrontLeft.set_value(0);
       }
    }
}