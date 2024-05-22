#include "main.h"
#include "lemlib/api.hpp"
#include "liblvgl/lvgl.h"

pros::Motor front_left_motor(-1, pros::v5::MotorGears::green);
pros::Motor middle_left_motor(-3, pros::v5::MotorGears::green);
pros::Motor back_left_motor(-13, pros::v5::MotorGears::green);
pros::Motor front_right_motor(2, pros::v5::MotorGears::green);
pros::Motor middle_right_motor(4, pros::v5::MotorGears::green);
pros::Motor back_right_motor(5, pros::v5::MotorGears::green);

pros::MotorGroup left_motor_group({-1, -3, -13}, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);

pros::MotorGroup right_motor_group({2, 4, 5}, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);

lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              10.8, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              333, // drivetrain rpm is 360
                              2 
);

pros::Imu imu(19);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2.2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              14, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);

void initialize() {
    // pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    // pros::Task screen_task([&]() {
    //     while (true) {
    //         // print robot location to the brain screen
    //         pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    //         pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    //         pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    //         // delay to save resources
    //         pros::delay(20);
    //     }
    // });
    LV_IMG_DECLARE(cheese);
    lv_obj_t * img = lv_img_create(lv_scr_act());
    lv_img_set_src(img, &cheese);

    // lv_obj_t * img = lv_gif_create(lv_scr_act());
    // lv_gif_set_src(img, "/usd/life.gif");
    lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.turnToHeading(120, 100000);
}

void opcontrol() {
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX*.7, false, 0.95);

        // delay to save resources
        pros::delay(25);
    }
}