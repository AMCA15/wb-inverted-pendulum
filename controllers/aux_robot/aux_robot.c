/*
 * File:          aux_robot.c
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
#include <webots/motor.h>
#include <webots/robot.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 16
#define MAX_LINEAR_MOTOR_POSITION 0.015

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

  WbDeviceTag left_motor = wb_robot_get_device("robot1 left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("robot1 right wheel motor");
  WbDeviceTag linear_motor = wb_robot_get_device("robot1 linear motor");

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */

    // if (wb_robot_get_time() >= 5.0) {
    //   wb_motor_set_position(linear_motor, MAX_LINEAR_MOTOR_POSITION);
    // }

    if (wb_robot_get_time() >= 52.3) {
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
    } else if (wb_robot_get_time() >= 47.4) {
      wb_motor_set_velocity(left_motor, 10);
      wb_motor_set_velocity(right_motor, 10);
    } else if (wb_robot_get_time() >= 12) {
      wb_motor_set_velocity(left_motor, 3);
      wb_motor_set_velocity(right_motor, -3);
    } else if (wb_robot_get_time() >= 11) {
      wb_motor_set_position(linear_motor, MAX_LINEAR_MOTOR_POSITION);
    } else if (wb_robot_get_time() >= 10.5) {
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
    } else if (wb_robot_get_time() >= 3.78) {
      wb_motor_set_velocity(left_motor, 5);
      wb_motor_set_velocity(right_motor, 5);
    } else if (wb_robot_get_time() >= 2.0) {
      wb_motor_set_velocity(left_motor, 5);
      wb_motor_set_velocity(right_motor, -5);
    }

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
