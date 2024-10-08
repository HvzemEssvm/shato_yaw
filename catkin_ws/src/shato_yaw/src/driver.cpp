/** Editor: Hazem Essam
  * 
  * Purpose: This driver cpp code takes the desired angle from the user (-180°≤yaw≤+180°)
  * and publishes it on a topic named "Destination_Topic" to be subscribed on by the PID class 
  * also, it handles the invalid input as shown below.
  * 
  * Remember to add the following in the CMakeLists.txt file:
  *
  * (catkin REQUIRED COMPONENTS
  * roscpp
  * std_msgs
  * )
  *
  * catkin_package(
  * CATKIN_DEPENDS roscpp std_msgs
  * )
  *
  * add_executable(driver src/driver.cpp)
  * target_link_libraries(driver ${catkin_LIBRARIES})
  * 
  * Remember to add the following in the Package.xml file:
  * 
  * <depend>roscpp</depend>
  * <depend>std_msgs</depend>
  * 
  */ 
  
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream> 
#include <limits>

float kp=2, ki=0.02,kd=0.011;
char confirm;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"Driver");
    ros::NodeHandle nd;
    ros::Publisher driver = nd.advertise<std_msgs::Float32>("Destination_Topic", 1000);
    ros::Publisher pidPub = nd.advertise<std_msgs::Float32MultiArray>("pid_tuning",1000);
    ros::Rate loop_rate(1);
    std_msgs::Float32 angle;
    std_msgs::Float32MultiArray pid_params;
    pid_params.data.resize(3);
    pid_params.data[0] = kp;
    pid_params.data[1] = ki;
    pid_params.data[2] = kd;

    ROS_INFO("The PID parameters are intially assgined as <Kp = 2, KI = 0.2 and Kd = 0.011> ");

    while(ros::ok())
    {
        ROS_INFO("The current PID Parameters are <Kp = %f, KI = %f and Kd = %f> ",kp, ki, kd);
        ROS_INFO("Press (Y) to change PID Parameters: ");
        std::cin >> confirm;
        if (confirm == 'Y' || confirm == 'y')
        {
            ROS_INFO("Enter New P-Gain (Kp): ");
            std::cin >> kp;
            pid_params.data[0] = kp;
            ROS_INFO("Enter New I-Gain (Ki): ");
            std::cin >> ki;
            pid_params.data[1] = ki;
            ROS_INFO("Enter New D-Gain (Kd): ");
            std::cin >> kd;
            pid_params.data[2] = kd;

            pidPub.publish(pid_params);
        }

        bool validInput = false;
        do
        {
            ROS_INFO("Enter A Valid Desired Angle (-180<=yaw<=+180): ");
            std::cin >> angle.data;
            // The following block of code is used for invalid error handling
            if (std::cin.fail()) {
                std::cin.clear(); 
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); 
                ROS_WARN("Invalid input. Please enter a numeric value.");
            }
            else if (angle.data > 180 || angle.data < -180) {
                ROS_WARN("Invalid angle. Please enter a value between -180° and 180°.");
            }
            else {
                validInput = true; // Valid input
            }
            //----------------------------------------------------------------
        }while(!validInput);
        driver.publish(angle);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

