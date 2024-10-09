#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

// Motor pin definitions for l29x DRIVER
#define motor_fr_pwm PB0
#define motor_fl_pwm PB1
#define motor_rl_pwm PB10
#define motor_rr_pwm PB11
#define motor_fr_dir1 PA0
#define motor_fr_dir2 PA1
#define motor_fl_dir1 PA2
#define motor_fl_dir2 PA3
#define motor_rl_dir1 PA4
#define motor_rl_dir2 PA5
#define motor_rr_dir1 PA6
#define motor_rr_dir2 PA7

// for Cytron Define the following, uncomment them, comment the above definitions also uncomment cytron class and modify the code
// #define motor_fr_pwm PB0
// #define motor_fl_pwm PB1
// #define motor_rl_pwm PB10
// #define motor_rr_pwm PB11
// #define motor_fr_dir PA0
// #define motor_rr_dir PA1
// #define motor_fl_dir PA2
// #define motor_rl_dir PA3

double Target, current_yaw, Output;

// PID parameters
float kp = 2, ki = 0.02, kd = 0.01;
PID myPID(&current_yaw, &Output, &Target, kp, ki, kd, DIRECT);

// PID tuning callback
void tuning(const std_msgs::Float32MultiArray& pid_pars) {
    kp = pid_pars.data[0];
    ki = pid_pars.data[1];
    kd = pid_pars.data[2];
    myPID.SetTunings(kp, ki, kd);
}

// Targeting callback
void targeting(const std_msgs::Float32& target_yaw) {
    Target = target_yaw.data;
    if (Target<0) // h
      Target += 360; // h this will change the range <-180~180> to <0~360>
}

// ROS setup
ros::NodeHandle nh;
std_msgs::Float32 yaw;
ros::Publisher yaw_angle("yaw_angle", &yaw);
ros::Subscriber<std_msgs::Float32> sub2("Destination_Topic", &targeting);
ros::Subscriber<std_msgs::Float32MultiArray> sub1("pid_tuning", &tuning);

// BNO055 initialization
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

class motorDriver
{
  protected:
    virtual void Rotate_anticlockwise() = 0;
    virtual void Rotate_clockwise() = 0;
    virtual void stopMotors() = 0;
  public:
    void changePWM(double pwm) // h
    {
      if (pwm<0)
        pwm *= -1;
        
      if (pwm>255)
        pwm = 255;
      
      analogWrite(motor_fr_pwm,pwm);
      analogWrite(motor_fl_pwm,pwm);
      analogWrite(motor_rr_pwm,pwm);
      analogWrite(motor_rl_pwm,pwm);
    }
};

class L29X : public motorDriver
{
  public:

    void Rotate_anticlockwise() override
    {
      digitalWrite(motor_fr_dir1,HIGH);
      digitalWrite(motor_fr_dir2,LOW);
      digitalWrite(motor_fl_dir1,LOW);
      digitalWrite(motor_fl_dir2,HIGH);
      digitalWrite(motor_rr_dir1,HIGH);
      digitalWrite(motor_rr_dir2,LOW);
      digitalWrite(motor_rl_dir1,LOW);
      digitalWrite(motor_rl_dir2,HIGH);
    }

    void Rotate_clockwise() override
    {
      digitalWrite(motor_fr_dir1,LOW);
      digitalWrite(motor_fr_dir2,HIGH);
      digitalWrite(motor_fl_dir1,HIGH);
      digitalWrite(motor_fl_dir2,LOW);
      digitalWrite(motor_rr_dir1,LOW);
      digitalWrite(motor_rr_dir2,HIGH);
      digitalWrite(motor_rl_dir1,HIGH);
      digitalWrite(motor_rl_dir2,LOW);
    }

    void stopMotors() override
    {
      changePWM(0);
      digitalWrite(motor_fr_dir1,LOW);
      digitalWrite(motor_fr_dir2,LOW);
      digitalWrite(motor_fl_dir1,LOW);
      digitalWrite(motor_fl_dir2,LOW);
      digitalWrite(motor_rr_dir1,LOW);
      digitalWrite(motor_rr_dir2,LOW);
      digitalWrite(motor_rl_dir1,LOW);
      digitalWrite(motor_rl_dir2,LOW);
    }
};

// class cytron : public motorDriver
// {
//   public:

//     void Rotate_anticlockwise() override
//     {
//       digitalWrite(motor_fr_dir,HIGH);
//       digitalWrite(motor_rr_dir,HIGH);
//       digitalWrite(motor_fl_dir,LOW);
//       digitalWrite(motor_rl_dir,LOW);
//     }

//     void Rotate_clockwise() override
//     {
//       digitalWrite(motor_fr_dir,LOW);
//       digitalWrite(motor_rr_dir,LOW);
//       digitalWrite(motor_fl_dir,HIGH);
//       digitalWrite(motor_rl_dir,HIGH);
//     }

//     void stopMotors() override
//     {
//       changePWM(0);
//     }
// };

L29X L298;

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

    if (Output > 0) 
    {
      L298.Rotate_anticlockwise();
      L298.changePWM(Output);
    } 
    else if (Output == 0)
    {
      L298.stopMotors();
    }
    else 
    {
      L298.Rotate_clockwise();
      L298.changePWM(Output);
    }

    // Publish current yaw angle
    yaw_angle.publish(&yaw);

    nh.spinOnce();
    delay(10);
}
