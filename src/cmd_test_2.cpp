// Shmulik Edelman
// shmulike@post.bgu.ac.il
//------------------------------------------------------------

// -------------------- Includer directories --------------------
#include "ros/ros.h"
#include "ros/time.h"
#include <signal.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "math.h"


// -------------------- Define constants --------------------
#define sinAmp1 25
#define N_links 4
#define PI 3.14159265
#define step 10000.0
#define step_factor 6.0

// -------------------- Global variables --------------------

//#define MAX_PWM_angle 65                     // 255
float counter = 0;
int fase = 1;
std_msgs:: Float32MultiArray joint_cmd_array;

float arr[N_links] = {0.0};
ros::Publisher pub_1, pub_led;

// -------------------- Functions defenition --------------------

void mySigintHandler(int sig);

int main(int argc, char **argv)
{
    std_msgs:: Bool IRLed;
    IRLed.data = false;
    ros::init(argc, argv, "controller_2", ros::init_options::NoSigintHandler);
    //ros::init(argc, argv, "controller_1");
    ros::NodeHandle n;
    signal(SIGINT, mySigintHandler);
    // -------------------- Publishers --------------------
    pub_1 = n.advertise<std_msgs::Float32MultiArray>("/robot_snake_1/joint_cmd", 1000);
    pub_led = n.advertise<std_msgs::Bool>("/robot_snake_1/toggle_led", 1000);
    // -------------------- Subscribers --------------------
    /*
    ros::Subscriber sub_1 = n.subscribe("/robot_snake_1/joint_val",     1000, get_joint_val);
    ros::Subscriber sub_2 = n.subscribe("/robot_snake_1/joint_cmd",     1000, get_joint_cmd);
    ros::Subscriber sub_3 = n.subscribe("/robot_snake_1/linear_val",    1000, get_linear_val);
    ros::Subscriber sub_4 = n.subscribe("/robot_snake_1/linear_cmd",    1000, get_linear_cmd);
    ros::Subscriber sub_5 = n.subscribe("/robot_snake_1/tension_val",   1000, get_tension_val);
    */
    ros::Rate loop_rate(100);
    

    ROS_WARN("->\tController node launched !");
    ROS_WARN("\tWaiting now for 5 seconds to start");

    arr[0] = 0;
    arr[1] = 0;
    arr[2] = 0;
    arr[3] = 0;
    joint_cmd_array.data.clear();
    for (int joint_i=0; joint_i<N_links; joint_i++){
        joint_cmd_array.data.push_back( arr[joint_i] );    
    }
    while (!ros::ok());
    pub_1.publish(joint_cmd_array);
    ros::spinOnce();
    loop_rate.sleep();
    ros::Duration(20, 0).sleep();
    ROS_WARN("\tStarting");

    while (ros::ok())
    {
            IRLed.data = true;
            ROS_WARN("Fase : %d\tCounter : %f", fase, counter);
            if (counter < 0.25)
            {
                counter += 2.0*PI/(step*step_factor);
                fase = 1;
            }
            if (counter >= 0.25 && counter < 1.25)
            {
                counter += 2.0*PI/(step*step_factor);
                fase = 2;
            }
            if (counter >= 1.25 && counter < 1.5)
            {
                counter += 2.0*PI/(step*step_factor);
                fase = 3;
            }

            switch(fase){
                case 1:
                arr[0] =  sinAmp1 * sin(counter*2*PI);
                arr[1] = 0;
                arr[2] = -sinAmp1 * sin(counter*2*PI);
                arr[3] = 0;
                break;

                case 2:
                arr[0] =  sinAmp1 * sin(counter*2*PI);
                arr[1] =  sinAmp1 * cos(counter*2*PI);
                arr[2] = -sinAmp1 * sin(counter*2*PI);
                arr[3] = -sinAmp1 * cos(counter*2*PI);
                break;

                case 3:
                arr[0] = sinAmp1 * sin(counter*2*PI);;
                arr[1] = 0;
                arr[2] = -sinAmp1 * sin(counter*2*PI);
                arr[3] = 0;
                break;
            }            
            
            joint_cmd_array.data.clear();
            for (int joint_i=0; joint_i<N_links; joint_i++){
                joint_cmd_array.data.push_back( arr[joint_i] );    
            }
            
            pub_1.publish(joint_cmd_array);
            pub_led.publish(IRLed);

        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
    return 0;
}

// ------------------------------------------------------------
// -------------------- Functions --------------------

void mySigintHandler(int sig){
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
    joint_cmd_array.data.clear();
    for (int joint_i=0; joint_i<N_links; joint_i++){
        joint_cmd_array.data.push_back( 0.0 );    
    }
    ROS_ERROR("--> Shutting Down. Get all joints to Home-Position");
    pub_1.publish(joint_cmd_array);

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

