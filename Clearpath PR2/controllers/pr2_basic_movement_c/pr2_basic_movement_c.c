#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <webots/position_sensor.h>
#include <math.h>

#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 16

// PR2 constants
#define MAX_WHEEL_SPEED 3.0        // maximum velocity for the wheels [rad / s]
#define WHEELS_DISTANCE 0.4492     // distance between 2 caster wheels (the four wheels are located in square) [m]
#define SUB_WHEELS_DISTANCE 0.098  // distance between 2 sub wheels of a caster wheel [m]
#define WHEEL_RADIUS 0.08          // wheel radius

#define TOLERANCE 0.05  // arbitrary value
#define ALMOST_EQUAL(a, b) ((a < b + TOLERANCE) && (a > b - TOLERANCE))

// helper constants to distinguish the motors
enum { FLL_WHEEL, FLR_WHEEL, FRL_WHEEL, FRR_WHEEL, BLL_WHEEL, BLR_WHEEL, BRL_WHEEL, BRR_WHEEL };
enum { FL_ROTATION, FR_ROTATION, BL_ROTATION, BR_ROTATION };

// PR2 motors and their sensors
static WbDeviceTag wheel_motors[8];
static WbDeviceTag wheel_sensors[8];
static WbDeviceTag rotation_motors[4];
static WbDeviceTag rotation_sensors[4];

// Simpler step function
static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

// Retrieve all the pointers to the PR2 devices
static void initialize_devices() {
  int i;
  wheel_motors[FLL_WHEEL] = wb_robot_get_device("fl_caster_l_wheel_joint");
  wheel_motors[FLR_WHEEL] = wb_robot_get_device("fl_caster_r_wheel_joint");
  wheel_motors[FRL_WHEEL] = wb_robot_get_device("fr_caster_l_wheel_joint");
  wheel_motors[FRR_WHEEL] = wb_robot_get_device("fr_caster_r_wheel_joint");
  wheel_motors[BLL_WHEEL] = wb_robot_get_device("bl_caster_l_wheel_joint");
  wheel_motors[BLR_WHEEL] = wb_robot_get_device("bl_caster_r_wheel_joint");
  wheel_motors[BRL_WHEEL] = wb_robot_get_device("br_caster_l_wheel_joint");
  wheel_motors[BRR_WHEEL] = wb_robot_get_device("br_caster_r_wheel_joint");
  for (i = FLL_WHEEL; i <= BRR_WHEEL; ++i)
    wheel_sensors[i] = wb_motor_get_position_sensor(wheel_motors[i]);

  rotation_motors[FL_ROTATION] = wb_robot_get_device("fl_caster_rotation_joint");
  rotation_motors[FR_ROTATION] = wb_robot_get_device("fr_caster_rotation_joint");
  rotation_motors[BL_ROTATION] = wb_robot_get_device("bl_caster_rotation_joint");
  rotation_motors[BR_ROTATION] = wb_robot_get_device("br_caster_rotation_joint");
  for (i = FL_ROTATION; i <= BR_ROTATION; ++i)
    rotation_sensors[i] = wb_motor_get_position_sensor(rotation_motors[i]);
    }
    
// enable the robot devices
static void enable_devices() {
  int i = 0;
  for (i = 0; i < 8; ++i) {
    wb_position_sensor_enable(wheel_sensors[i], TIME_STEP);
    // init the motors for speed control
    wb_motor_set_position(wheel_motors[i], INFINITY);
    wb_motor_set_velocity(wheel_motors[i], 0.0);
  }

  for (i = 0; i < 4; ++i)
    wb_position_sensor_enable(rotation_sensors[i], TIME_STEP);
    }
// set the speeds of the robot wheels
static void set_wheels_speeds(double fll, double flr, double frl, double frr, double bll, double blr, double brl, double brr) {
  wb_motor_set_velocity(wheel_motors[FLL_WHEEL], fll);
  wb_motor_set_velocity(wheel_motors[FLR_WHEEL], flr);
  wb_motor_set_velocity(wheel_motors[FRL_WHEEL], frl);
  wb_motor_set_velocity(wheel_motors[FRR_WHEEL], frr);
  wb_motor_set_velocity(wheel_motors[BLL_WHEEL], bll);
  wb_motor_set_velocity(wheel_motors[BLR_WHEEL], blr);
  wb_motor_set_velocity(wheel_motors[BRL_WHEEL], brl);
  wb_motor_set_velocity(wheel_motors[BRR_WHEEL], brr);
}


static void set_wheels_speed(double speed) {
  set_wheels_speeds(speed, speed, speed, speed, speed, speed, speed, speed);
}

static void stop_wheels() {
  set_wheels_speeds(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

// enable/disable the torques on the wheels motors
static void enable_passive_wheels(bool enable) {
  static double torques[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  int i;
  if (enable) {
    for (i = 0; i < 8; ++i) {
      torques[i] = wb_motor_get_available_torque(wheel_motors[i]);
      wb_motor_set_available_torque(wheel_motors[i], 0.0);
    }
  } else {
    for (i = 0; i < 8; ++i)
      wb_motor_set_available_torque(wheel_motors[i], torques[i]);
  }
}

// Set the rotation wheels angles.
// If wait_on_feedback is true, the function is left when the rotational motors have reached their target positions.
static void set_rotation_wheels_angles(double fl, double fr, double bl, double br, bool wait_on_feedback) {
  if (wait_on_feedback) {
    stop_wheels();
    enable_passive_wheels(true);
  }

  wb_motor_set_position(rotation_motors[FL_ROTATION], fl);
  wb_motor_set_position(rotation_motors[FR_ROTATION], fr);
  wb_motor_set_position(rotation_motors[BL_ROTATION], bl);
  wb_motor_set_position(rotation_motors[BR_ROTATION], br);

  if (wait_on_feedback) {
    const double target[4] = {fl, fr, bl, br};

    while (true) {
      bool all_reached = true;
      int i;
      for (i = 0; i < 4; ++i) {
        double current_position = wb_position_sensor_get_value(rotation_sensors[i]);
        if (!ALMOST_EQUAL(current_position, target[i])) {
          all_reached = false;
          break;
        }
      }

      if (all_reached)
        break;
      else
        step();
    }

    enable_passive_wheels(false);
  }
}

// High level function to rotate the robot around itself of a given angle [rad]
// Note: the angle can be negative
static void robot_rotate(double angle) {
  stop_wheels();

  set_rotation_wheels_angles(3.0 * M_PI_4, M_PI_4, -3.0 * M_PI_4, -M_PI_4, true);
  const double max_wheel_speed = angle > 0 ? MAX_WHEEL_SPEED : -MAX_WHEEL_SPEED;
  set_wheels_speed(max_wheel_speed);

  double initial_wheel0_position = wb_position_sensor_get_value(wheel_sensors[FLL_WHEEL]);
  // expected travel distance done by the wheel
  double expected_travel_distance = fabs(angle * 0.5 * (WHEELS_DISTANCE + SUB_WHEELS_DISTANCE));

  while (true) {
    double wheel0_position = wb_position_sensor_get_value(wheel_sensors[FLL_WHEEL]);
    // travel distance done by the wheel
    double wheel0_travel_distance = fabs(WHEEL_RADIUS * (wheel0_position - initial_wheel0_position));

    if (wheel0_travel_distance > expected_travel_distance)
      break;

    // reduce the speed before reaching the target
    if (expected_travel_distance - wheel0_travel_distance < 0.025)
      set_wheels_speed(0.1 * max_wheel_speed);

    step();
  }

  // reset wheels
  set_rotation_wheels_angles(0.0, 0.0, 0.0, 0.0, true);
  stop_wheels();
}

// High level function to go forward for a given distance [m]
// Note: the distance can be negative
static void robot_go_forward(double distance) {
  double max_wheel_speed = distance > 0 ? MAX_WHEEL_SPEED : -MAX_WHEEL_SPEED;
  set_wheels_speed(max_wheel_speed);

  double initial_wheel0_position = wb_position_sensor_get_value(wheel_sensors[FLL_WHEEL]);

  while (true) {
    double wheel0_position = wb_position_sensor_get_value(wheel_sensors[FLL_WHEEL]);
    // travel distance done by the wheel
    double wheel0_travel_distance = fabs(WHEEL_RADIUS * (wheel0_position - initial_wheel0_position));

    if (wheel0_travel_distance > fabs(distance))
      break;

    // reduce the speed before reaching the target
    if (fabs(distance) - wheel0_travel_distance < 0.025)
      set_wheels_speed(0.1 * max_wheel_speed);

    step();
  }

  stop_wheels();
}


int main(int argc, char **argv) {
  wb_robot_init();
  

  initialize_devices();
  wb_keyboard_enable(TIME_STEP);
  enable_devices();
  //set_initial_position();
  
  robot_go_forward(0.0);
  
  // main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    int key = wb_keyboard_get_key();
    
    if (key == 'S') {
    robot_go_forward(-0.35);
    }
    else if (key == 'A') {
    robot_rotate(-1);
    }
    else if (key == 'D') {
    robot_rotate(1);
    }
    else if (key == 'W') {
    robot_go_forward(0.35);
    }
    }
    
     wb_robot_cleanup();

  return EXIT_SUCCESS;
}



  
  

