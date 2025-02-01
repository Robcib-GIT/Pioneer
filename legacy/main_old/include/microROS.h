#ifndef microrROS_H
#define microrROS_H

#include <Arduino.h>
#include <odometry.h>
#include "pinout.h"
#include "main.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }

#define samplingTime 20 //ms    

extern const float wheel_circumference;
extern const float wheels_y_distance;

void initialize();
void cmd_velCallback(const void * msgin); // es imprescindible que sea static
void motorControlCallback(rcl_timer_t* timer, int64_t last_call_time); // es imprescindible que sea static
void publishData();

void start();
void error_loop(); // hace que este método sea estático, es decir, sea de la clase y no del objeto

void syncTime();
struct timespec getTime();

#endif // microrROS_H







