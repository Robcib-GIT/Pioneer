#include "microROS.h"

rcl_subscription_t cmd_vel_subscriber;
rcl_publisher_t odom_publisher;

geometry_msgs__msg__Twist cmd_vel_msg;
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__Int32 encodervalue1;
std_msgs__msg__Int32 encodervalue2;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_timer_t controlTimer;

Odometry odometry;
unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

void initialize(){
    set_microros_serial_transports(Serial);
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);

    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

    // create cmd_vel subscriber
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    //create odometry publisher
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"));

    // create timer, to be used in the control loop
    RCCHECK(rclc_timer_init_default(
        &controlTimer,
        &support,
        RCL_MS_TO_NS(samplingTime),
        motorControlCallback));


    //create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); 
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_velCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &controlTimer));

}


// suscriber callback
void cmd_velCallback(const void * msgin)
{  
    prev_cmd_time = millis();
}

//function which controlles the motor
void motorControlCallback(rcl_timer_t* timer, int64_t last_call_time) {
    //linear velocity and angular velocity send cmd_vel topic
    float linearVelocity = cmd_vel_msg.linear.x;
    float angularVelocity = cmd_vel_msg.angular.z;
    //linear and angular velocities are converted to leftwheel and rightwheel velocities
    float vL = (linearVelocity - (angularVelocity * 1 / 2)) * 20;
    float vR = (linearVelocity + (angularVelocity * 1 / 2)) * 20;
    //current wheel rpm is calculated
    float currentRpmL = motor1.calculateRPM();
    float currentRpmR = motor2.calculateRPM();
    //pid controlled is used for generating the pwm signal
    float actuating_signal_LW = motor1.computePID(vL, currentRpmL);
    float actuating_signal_RW = motor2.computePID(vR, currentRpmR);
    //if the velocity is zero, the motors are stopped
    if (vL == 0 && vR == 0) {
        motor1.stop();
        motor2.stop();
        actuating_signal_LW = 0;
        actuating_signal_RW = 0;
    } else {
        motor1.move(actuating_signal_LW);
        motor2.move(actuating_signal_RW);
    }
    //odometry
    float average_rps_x = ((float)(currentRpmL + currentRpmR) / 2) / 60.0;  // RPM
    float linear_x = average_rps_x * wheel_circumference;                  // m/s
    float average_rps_a = ((float)(-currentRpmL + currentRpmR) / 2) / 60.0;
    float angular_z = (average_rps_a * wheel_circumference) / (wheels_y_distance / 2.0);  //  rad/s
    float linear_y = 0;
    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(vel_dt, linear_x, linear_y, angular_z);
    publishData();
}


//function which publishes wheel odometry.
void publishData() {
    odom_msg = odometry.getData();

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

void start(){
    delay(100);
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void error_loop() {
    while (1) {
        digitalWrite(LED, !digitalRead(LED));
        delay(100);
    }
}

void syncTime() {
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}


struct timespec getTime() {
    struct timespec tp = { 0 };
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}