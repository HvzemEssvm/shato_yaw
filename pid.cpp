/** Editor: Hazem Essam
 *
 * purpose: This pid.cpp file subscribes to "yaw_angle" & "Destination_Topic" and updates the pid controller with the current angle and the destination angle.
 * then, it returns the output value to be used in controlling the motors of shato.
 *
 * Dependencies and Excutables:
 * roscpp
 * std_msgs
 *
 * Remember to write the following in CMakeLists.txt
 * add_executable(pid_sub src/pid_subscriber.cpp)
 * target_link_libraries(pid_sub ${catkin_LIBRARIES})
 */
#include <stdio.h>
#include "std_msgs/Float32.h"
#include <iostream>
#include <chrono> 

std_msgs::Float32 currAngle=0, destAngle=0;
long long lastTime=0,currentTime;
double dt;
float iTerm=0,prevError=0;

float kp = 2; // change this term when tuning
float ki = 0.02; // change this term when tuning
float kd = 0.011; // change this term when tuning

long long millis()
{
 return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
}

void yawCallBack(const std_msgs::Float32::ConstPtr& msg)
{
    currAngle = msg->data;
    ROS_INFO("Current Angle: %f°", currAngle);
}

void destCallBack(const std_msgs::Float32::ConstPtr& msg)
{
    destAngle = msg->data;
}


float pidControl()
{
    float error, pTerm,dTerm,output;

    error = destAngle - currAngle;
    pTerm = kp * error;
    iTerm += ki * error * dt;
    dTerm = kd * (error - prevError) / dt;
    prevError = error;
    output = pTerm + iTerm + dTerm;
    return output;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_sub");
    ros::NodeHandle ns;
    ros::Subscriber currSub = ns.subscribe("yaw_angle", 1000, yawCallBack);
    ros::Subscriber destSub = ns.subscribe("Destination_Topic", 1000, destCallBack);
    ros::Publisher outputSub = ns.advertise<std_msgs::Float32>("pid_output", 1000);

    ros::Rate rate(100);
    while(ros::ok())
    {
        currentTime = millis();
        dt = (currentTime - lastTime)/1000;
        lastTime = currentTime;

        float output = pidControl();
        
        outputSub.publish(output);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
 
}
