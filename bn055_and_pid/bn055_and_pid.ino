#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

// Motor pin definitions
const int motor_fr_pwm = PA0;  // PWM pin for front right motor speed
const int motor_fr_dir1 = PA4; // Direction pin 1 for front right motor
const int motor_fr_dir2 = PA5; // Direction pin 2 for front right motor
const int motor_fl_pwm = PA1;  // PWM pin for front left motor speed
const int motor_fl_dir1 = PA6; // Direction pin 1 for front left motor
const int motor_fl_dir2 = PA7; // Direction pin 2 for front left motor
const int motor_rl_pwm = PA2;  // PWM pin for rear left motor speed
const int motor_rl_dir1 = PB0; // Direction pin 1 for rear left motor
const int motor_rl_dir2 = PB1; // Direction pin 2 for rear left motor
const int motor_rr_pwm = PA3;  // PWM pin for rear right motor speed
const int motor_rr_dir1 = PB10; // Direction pin 1 for rear right motor
const int motor_rr_dir2 = PB11; // Direction pin 2 for rear right motor

double Target, current_yaw, Output;

// PID parameters
float kp = 2, ki = 5, kd = 1;
PID myPID(&current_yaw, &Output, &Target, kp, ki, kd, DIRECT);

// ROS setup
ros::NodeHandle nh;
std_msgs::Float32 yaw;
ros::Publisher yaw_angle("yaw_angle", &yaw);

// PID tuning callback
void tuning(const std_msgs::Float32MultiArray& pid_pars) {
    kp = pid_pars.data[0];
    ki = pid_pars.data[1];
    kd = pid_pars.data[2];
    myPID.SetTunings(kp, ki, kd);
}
ros::Subscriber<std_msgs::Float32MultiArray> sub1("pid_tuning", &tuning);

// Targeting callback
void targeting(const std_msgs::Float32& target_yaw) {
    Target = target_yaw.data;
}
ros::Subscriber<std_msgs::Float32> sub2("target_yaw", &targeting);

// BNO055 initialization
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void Rotate_left(float speed) {
   
}

void Rotate_right(float speed) {
   
}

void setup() {
    // Initialize the motors and ROS
    pinMode(motor_fr_pwm, OUTPUT);
    pinMode(motor_fr_dir1, OUTPUT);
    pinMode(motor_fr_dir2, OUTPUT);
    pinMode(motor_fl_pwm, OUTPUT);
    pinMode(motor_fl_dir1, OUTPUT);
    pinMode(motor_fl_dir2, OUTPUT);
    pinMode(motor_rl_pwm, OUTPUT);
    pinMode(motor_rl_dir1, OUTPUT);
    pinMode(motor_rl_dir2, OUTPUT);
    pinMode(motor_rr_pwm, OUTPUT);
    pinMode(motor_rr_dir1, OUTPUT);
    pinMode(motor_rr_dir2, OUTPUT);
    
    current_yaw = 0;
    Target = 0;
    nh.initNode();
    nh.advertise(yaw_angle);
    yaw.data = 0;

    // Initialize BNO055 sensor
    if (!bno.begin()) {
        // Handle initialization error
        while (1);
    }
}

void loop() {
    // BNO055 data
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    yaw.data = orientationData.orientation.x;
    current_yaw = yaw.data;

    // PID computation
    myPID.Compute();
    if (Output > 0) {
        Rotate_left(Output);
    } else {
        Rotate_right(Output); // 
    }

    // Publish current yaw angle
    yaw_angle.publish(&yaw);

    nh.spinOnce();
    delay(10);
}
