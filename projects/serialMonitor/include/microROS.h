#ifndef microrROS_H
#define microrROS_H

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>

#include <string.h>


class microRos{
    private:
        rcl_subscription_t subscriber;
        rcl_publisher_t publisher;
        geometry_msgs__msg__Twist cmd_vel_msg;
        std_msgs__msg__String serial_monitor_msg;


        rclc_executor_t executor;
        rclc_support_t support;
        rcl_allocator_t allocator;
        rcl_node_t node;

    public:
        microRos();
        void initialize();
        static void cmd_velCallback(const void * msgin); // es imprescindible que sea static
        void publish();
        void start();


};



#endif