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

/*
 * You may want to add macros here.
 */
#define TIME_STEP 16

#define MAX_SPEED 10
#define DISTANCE_SENSOR_NUM 4

enum DistanceSensor {
  DS_FL,
  DS_FR,
  DS_RL,
  DS_RR,
};

enum AXIS { AXIS_X, AXIS_Y, AXIS_Z };

enum STATES { STATE_X, STATE_X_DOT, STATE_THETA, STATE_THETA_DOT };

double K[1][4] = {{-1.5291, -3.4149, -47.1491, -11.9149}};

double states[4][1] = {{0}, {0}, {0}, {0}};

double B[4][1] = {{0}, {1}, {0}, {-1}};

double Bk[4][4];

double r[4][1] = {{0}, {0}, {0}, {0}};

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

void printVector3(char* title, const double* v) {
  printf("%s: %f, %f, %f\n", title, v[AXIS_X], v[AXIS_Y], v[AXIS_Z]);
}

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char** argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  // Precompute Bk
  matrix_multiply((double*)B, (double*)K, (double*)Bk, 4, 1, 4);

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

  // Init keyboard
  wb_keyboard_enable(TIME_STEP * 1000);
  bool enable_controller = true;

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
    
    printf("Distance sensor values %f, %f, %f, %f\n", val_ds[DS_FL], val_ds[DS_FR],
           val_ds[DS_RL], val_ds[DS_RR]);
    printVector3("Accelerometer", val_accel);
    printVector3("Gyro", val_gyro);
    printVector3("Inertial unit", val_imu);
    printVector3("GPS position", val_gps_pos);
    printVector3("GPS velocity", val_gps_vel);
    printf("Motors position: Left %f - Right %f\n", left_motor_position,
           right_motor_position);

    states[STATE_X][0] = val_gps_pos[AXIS_X];
    states[STATE_X_DOT][0] = val_gps_vel[AXIS_X];
    states[STATE_THETA][0] = val_imu[AXIS_Y];
    states[STATE_THETA_DOT][0] = val_gyro[AXIS_Y];

    const double error[4][1] = {{0}, {0}, {0}, {0}};
    double input[1][1] = {0};
    vector_subtract((double*)r, (double*)states, (double*)error, 4);
    matrix_multiply((double*)K, (double*)error, (double*)input, 1, 4, 1);
    if (!enable_controller) {
      input[0][0] = 0;
    }
    printf("Input %f\n", input[0][0]);

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */

    wb_motor_set_torque(left_motor, input[0][0]);
    wb_motor_set_torque(right_motor, input[0][0]);
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
