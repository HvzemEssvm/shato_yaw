# shato_yaw

1. The Workspace is included in **catkin_ws** Directory
 * Remember to run the following
 ```
 cd <enterHereWorkspaceDirectory>
 catkin_make
 source devel/setup.bash
 rosrun shato_yaw driver
 ```
2. Included **Helloworld** to test the rosserial with the stm32

3. Included **pid.cpp** to incase we don't want to use the <PID_v1.h> and its purpose is written in the header of the file.

4. Included ``roslib`` and ``Rosserial_Arduino_Library`` put these files in 
 ``/Arduino/libraries`` instead of the previous folders as this includes modifications for rosserial
 
5. Included yaw_and_rosserial_pub which retrieves the value from the MPU6050 and publishes the angle on ``yaw angle`` topic
