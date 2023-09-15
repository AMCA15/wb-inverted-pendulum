/*
 * File:          inverted_pendulum.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <math.h>
#include <stdio.h>
#include <webots/accelerometer.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/utils/ansi_codes.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 16

#define MAX_LINEAR_SPEED 1.0
#define MAX_ANGULAR_VELOCITY 5.0
#define MAX_LINEAR_POSITION 0.3
#define DISTANCE_SENSOR_NUM 4
#define WHEEL_RADIUS 0.1

#define NUM_STATES 6
#define NUM_INPUTS 2

enum DistanceSensor {
  DS_FL,
  DS_FR,
  DS_RL,
  DS_RR,
};

enum AXIS { AXIS_X, AXIS_Y, AXIS_Z };

enum STATES {
  STATE_X,
  STATE_X_DOT,
  STATE_PHI,
  STATE_PHI_DOT,
  STATE_THETA,
  STATE_THETA_DOT
};

enum INPUS { INPUT_LEFT_MOTOR, INPUT_RIGHT_MOTOR };

double K[NUM_INPUTS][NUM_STATES] = {{-10, -10, 12, 1, -80, -10},
                                    {-10, -10, -12, -1, -80, -10}};

double states[NUM_STATES][1];

double r[NUM_STATES][1];

/**
 * Multiplies two matrices and stores the result in a third matrix.
 *
 * @param a Pointer to the first matrix.
 * @param b Pointer to the second matrix.
 * @param c Pointer to the matrix where the result will be stored.
 * @param m Number of rows in matrix a.
 * @param n Number of columns in matrix a and number of rows in matrix b.
 * @param p Number of columns in matrix b.
 */
void matrix_multiply(double* a, double* b, double* c, int m, int n, int p) {
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < p; j++) {
      double sum = 0;
      for (int k = 0; k < n; k++) {
        sum += a[i * n + k] * b[k * p + j];
      }
      c[i * p + j] = sum;
    }
  }
}

/**
 * Subtracts two vectors and stores the result in a third vector.
 *
 * @param a Pointer to the first vector.
 * @param b Pointer to the second vector.
 * @param c Pointer to the vector where the result will be stored.
 * @param n Number of elements in the vectors.
 */
void vector_subtract(double* a, double* b, double* c, int n) {
  for (int i = 0; i < n; i++) {
    c[i] = a[i] - b[i];
  }
}

void print_vector3(char* title, const double* v) {
  printf("%s%13s%s: %s%9f %s%9f %s%9f%s\n", ANSI_BOLD, title, ANSI_RESET,
         ANSI_RED_FOREGROUND, v[AXIS_X], ANSI_GREEN_FOREGROUND, v[AXIS_Y],
         ANSI_BLUE_FOREGROUND, v[AXIS_Z], ANSI_RESET);
}

void print_controller_mode(bool linear_speed, bool angular_rate) {
  printf(
      "%s Controller Mode:%s Linear %s%s - Angular %s%s\n", ANSI_BOLD, ANSI_RESET,
      linear_speed ? ANSI_GREEN_FOREGROUND "SPEED" : ANSI_BLUE_FOREGROUND "DISPLACEMENT",
      ANSI_RESET,
      angular_rate ? ANSI_GREEN_FOREGROUND "VELOCITY" : ANSI_BLUE_FOREGROUND "ANGLE",
      ANSI_RESET);
}

double limiter(double value, double min, double max) {
  return fmax(min, fmin(max, value));
}

void set_linear_displacement(double value) {
  r[STATE_X][0] = value;
}

void set_linear_speed(double value) {
  r[STATE_X_DOT][0] = limiter(value, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
}

void set_angular_position(double value) {
  r[STATE_PHI][0] = value;
}

void set_angular_rate(double value) {
  r[STATE_PHI_DOT][0] = limiter(value, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
}

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char** argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  WbDeviceTag ps[DISTANCE_SENSOR_NUM];
  char ps_names[DISTANCE_SENSOR_NUM][20] = {"FL distance sensor", "FR distance sensor",
                                            "RL distance sensor", "RR distance sensor"};
  for (int i = 0; i < DISTANCE_SENSOR_NUM; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  WbDeviceTag acceletometer = wb_robot_get_device("accelerometer");
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  WbDeviceTag inertial_unit = wb_robot_get_device("inertial unit");
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag left_motor_encoder = wb_robot_get_device("left wheel sensor");
  WbDeviceTag right_motor_encoder = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_motor_encoder, TIME_STEP);
  wb_position_sensor_enable(right_motor_encoder, TIME_STEP);

  wb_accelerometer_enable(acceletometer, TIME_STEP);
  wb_gyro_enable(gyro, TIME_STEP);
  wb_inertial_unit_enable(inertial_unit, TIME_STEP);
  wb_gps_enable(gps, TIME_STEP);

  WbDeviceTag left_motor = wb_position_sensor_get_motor(left_motor_encoder);
  WbDeviceTag right_motor = wb_position_sensor_get_motor(right_motor_encoder);
  WbDeviceTag linear_motor = wb_robot_get_device("linear motor");

  // Init keyboard
  wb_keyboard_enable(TIME_STEP * 10000);
  bool enable_controller = true;
  bool enable_lin_spd_mode = true;
  bool enable_ang_rat_mode = true;
  double linear_motor_position_val = wb_motor_get_target_position(linear_motor);
  wb_motor_set_velocity(linear_motor, 0.5);

  double prev_position = 0;
  double position = 0;
  double speed = 0;
  double pos_offset = 0;
  double rotation = 0;

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    int key = wb_keyboard_get_key();
    if (key == WB_KEYBOARD_SHIFT + 'C') {
      enable_controller = true;
    } else if (key == 'C') {
      enable_controller = false;
    } else if (key == 'W') {
      if (enable_lin_spd_mode) {
        set_linear_speed(r[STATE_X_DOT][0] + 0.1);
      } else {
        set_linear_displacement(r[STATE_X][0] + 0.1);
      }
    } else if (key == 'S') {
      if (enable_lin_spd_mode) {
        set_linear_speed(r[STATE_X_DOT][0] - 0.1);
      } else {
        set_linear_displacement(r[STATE_X][0] - 0.1);
      }
    } else if (key == 'Z') {
      if (enable_lin_spd_mode) {
        set_linear_speed(0);
      } else {
        set_linear_displacement(0);
      }
    } else if (key == 'A') {
      if (enable_ang_rat_mode) {
        set_angular_rate(r[STATE_PHI_DOT][0] + 0.1);
      } else {
        set_angular_position(r[STATE_PHI][0] + 0.1);
      }
    } else if (key == 'D') {
      if (enable_ang_rat_mode) {
        set_angular_rate(r[STATE_PHI_DOT][0] - 0.1);
      } else {
        set_angular_position(r[STATE_PHI][0] - 0.1);
      }
    } else if (key == 'X') {
      if (enable_ang_rat_mode) {
        set_angular_rate(0);
      } else {
        set_angular_position(0);
      }
    } else if (key == 'B') {
      linear_motor_position_val =
          limiter(linear_motor_position_val + 0.05, 0, MAX_LINEAR_POSITION);
    } else if (key == 'V') {
      linear_motor_position_val =
          limiter(linear_motor_position_val - 0.05, 0, MAX_LINEAR_POSITION);
    } else if (key == 'K') {
      enable_lin_spd_mode = false;
      r[STATE_X][0] = 0;
      r[STATE_X_DOT][0] = 0;
      pos_offset = position;
    } else if (key == WB_KEYBOARD_SHIFT + 'K') {
      r[STATE_X][0] = 0;
      r[STATE_X_DOT][0] = 0;
      enable_lin_spd_mode = true;
    } else if (key == 'L') {
      r[STATE_PHI][0] = 0;
      r[STATE_PHI_DOT][0] = 0;
      rotation = 0;
      enable_ang_rat_mode = false;
    } else if (key == WB_KEYBOARD_SHIFT + 'L') {
      r[STATE_PHI][0] = 0;
      r[STATE_PHI_DOT][0] = 0;
      rotation = 0;
      enable_ang_rat_mode = true;
    }

    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    const double val_ds[DISTANCE_SENSOR_NUM] = {
        wb_distance_sensor_get_value(ps[DS_FL]), wb_distance_sensor_get_value(ps[DS_FR]),
        wb_distance_sensor_get_value(ps[DS_RL]), wb_distance_sensor_get_value(ps[DS_RR])};

    const double* val_accel = wb_accelerometer_get_values(acceletometer);
    const double* val_gyro = wb_gyro_get_values(gyro);
    const double* val_imu = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
    const double* val_gps_pos = wb_gps_get_values(gps);
    const double* val_gps_vel = wb_gps_get_speed_vector(gps);
    const double left_motor_position = wb_position_sensor_get_value(left_motor_encoder);
    const double right_motor_position = wb_position_sensor_get_value(right_motor_encoder);

    ANSI_CLEAR_CONSOLE();
    print_controller_mode(enable_lin_spd_mode, enable_ang_rat_mode);
    print_vector3("Accelerometer", val_accel);
    print_vector3("Gyro", val_gyro);
    print_vector3("Inertial unit", val_imu);
    print_vector3("GPS position", val_gps_pos);
    print_vector3("GPS velocity", val_gps_vel);
    printf("%sDistance sensor:%s %.2f, %.2f, %.2f, %.2f\n", ANSI_BOLD, ANSI_RESET,
           val_ds[DS_FL], val_ds[DS_FR], val_ds[DS_RL], val_ds[DS_RR]);
    printf("%sMotors position:%s Left %9f - Right %9f\n", ANSI_BOLD, ANSI_RESET,
           left_motor_position, right_motor_position);

    position = (left_motor_position + right_motor_position) * WHEEL_RADIUS / 2;
    speed = (position - prev_position) / TIME_STEP * 1000;
    prev_position = position;
    rotation += val_gyro[AXIS_Z] * TIME_STEP / 1000;

    states[STATE_X][0] = enable_lin_spd_mode ? 0 : position - pos_offset;
    states[STATE_X_DOT][0] = speed;
    states[STATE_PHI][0] = enable_ang_rat_mode ? 0 : rotation;
    states[STATE_PHI_DOT][0] = val_gyro[AXIS_Z];
    states[STATE_THETA][0] = val_imu[AXIS_Y];
    states[STATE_THETA_DOT][0] = val_gyro[AXIS_Y];

    const double error[NUM_STATES][NUM_INPUTS];
    double input[NUM_INPUTS][1];
    vector_subtract((double*)r, (double*)states, (double*)error, NUM_STATES);
    matrix_multiply((double*)K, (double*)error, (double*)input, NUM_INPUTS, NUM_STATES,
                    1);
    if (!enable_controller) {
      input[INPUT_LEFT_MOTOR][0] = 0;
      input[INPUT_RIGHT_MOTOR][0] = 0;
    }
    printf("%-6sMotors Inputs:%s Left %9f - Right %9f\n", ANSI_BOLD, ANSI_RESET,
           input[INPUT_LEFT_MOTOR][0], input[INPUT_RIGHT_MOTOR][0]);
    printf("%sReference:%s X %9f X_dot %9f PHI %9f PHI_dot %9f THETA %9f THETA_DOT %9f\n",
           ANSI_BOLD, ANSI_RESET, r[STATE_X][0], r[STATE_X_DOT][0], r[STATE_PHI][0],
           r[STATE_PHI_DOT][0], r[STATE_THETA][0], r[STATE_THETA_DOT][0]);
    printf("%-7sStates:%s X %9f X_dot %9f PHI %9f PHI_dot %9f THETA %9f THETA_DOT %9f\n",
           ANSI_BOLD, ANSI_RESET, states[STATE_X][0], states[STATE_X_DOT][0],
           states[STATE_PHI][0], states[STATE_PHI_DOT][0], states[STATE_THETA][0],
           states[STATE_THETA_DOT][0]);

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */

    wb_motor_set_position(linear_motor, linear_motor_position_val);

    wb_motor_set_torque(left_motor, input[INPUT_LEFT_MOTOR][0]);
    wb_motor_set_torque(right_motor, input[INPUT_RIGHT_MOTOR][0]);
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
