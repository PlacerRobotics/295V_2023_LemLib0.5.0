#include "main.h"
#include "lemlib/api.hpp"

// Catapult motor
extern pros::Motor catapultMotor;

// Intake motor
extern pros::Motor intakeMotor;

// Distance sensor for catapult
extern pros::Distance cataDistance;

// Wings
extern pros::ADIDigitalOut wing1;
extern pros::ADIDigitalOut wing2;

// Intake pistons
extern pros::ADIDigitalOut intakePiston1;
extern pros::ADIDigitalOut intakePiston2;

// Catapult rotation
extern pros::Rotation catapultRotation;