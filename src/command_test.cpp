// Shmulik Edelman
// shmulike@post.bgu.ac.il
//------------------------------------------------------------

// -------------------- Includer directories --------------------
#include "ros/ros.h"
#include "ros/time.h"
#include <signal.h>
#include "std_msgs/Float32MultiArray.h"
#include "math.h"


// -------------------- Define constants --------------------
#define sinAmp1 25
#define sinAmp2 25
#define N_links 4
#define PI 3.14159265
#define step 10000.0

// -------------------- Global variables --------------------

//#define MAX_PWM_angle 65                     // 255
float counter = 0;
int fase = 1;
std_msgs:: Float32MultiArray joint_cmd_array;
float arr[N_links] = {0.0};
ros::Publisher pub_1;

// -------------------- Functions defenition --------------------
/*
void get_joint_val(const std_msgs::Float32MultiArray::ConstPtr& msg);
void get_joint_cmd(const std_msgs::Float32MultiArray::ConstPtr& msg);
void get_linear_val(const std_msgs::Float32::ConstPtr& msg);
void get_linear_cmd(const std_msgs::Float32::ConstPtr& msg);
void get_tension_val(const std_msgs::Float32MultiArray::ConstPtr& msg);
*/
void mySigintHandler(int sig);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_2", ros::init_options::NoSigintHandler);
    //ros::init(argc, argv, "controller_1");
    ros::NodeHandle n;
    signal(SIGINT, mySigintHandler);
    // -------------------- Piblishers --------------------
    pub_1 = n.advertise<std_msgs::Float32MultiArray>("/robot_snake_1/joint_cmd", 1000);
    // -------------------- Subscribers --------------------
    /*
    ros::Subscriber sub_1 = n.subscribe("/robot_snake_1/joint_val",     1000, get_joint_val);
    ros::Subscriber sub_2 = n.subscribe("/robot_snake_1/joint_cmd",     1000, get_joint_cmd);
    ros::Subscriber sub_3 = n.subscribe("/robot_snake_1/linear_val",    1000, get_linear_val);
    ros::Subscriber sub_4 = n.subscribe("/robot_snake_1/linear_cmd",    1000, get_linear_cmd);
    ros::Subscriber sub_5 = n.subscribe("/robot_snake_1/tension_val",   1000, get_tension_val);
    */
    ros::Rate loop_rate(100);
    

    ROS_WARN("-> Controller node launched !");
    while (ros::ok())
    { 
            
            ROS_WARN("fase : %d\tCounter : %f", fase, counter);
            if (counter < 1)
            {
                counter += 2.0*PI/step;
                fase = 1;
            }
            if (counter >= 1 && counter < 2)
            {
                counter += 2.0*PI/(step*6.0);
                fase = 2;
            }
            if (counter >= 2 && counter < 2.25)
            {
                counter += 2.0*PI/(step*6.0);
                fase = 3;
            }

            switch(fase){
                case 1:
                arr[0] = 0;
                arr[1] = 0;
                arr[2] = sinAmp1 * sin(counter*2*PI);
                arr[3] = 0;
                break;

                case 2:
                arr[0] = sinAmp1 * sin(counter*2*PI);
                arr[1] = 0;
                arr[2] = -sinAmp1 * sin(counter*2*PI);
                arr[3] = 0;
                break;

                case 3:
                arr[0] = 0;
                arr[1] = -sinAmp2 * sin(counter*2*PI);
                arr[2] = 0;
                arr[3] = -sinAmp2 * sin(counter*2*PI);
                break;
            }            
            
            joint_cmd_array.data.clear();
            for (int joint_i=0; joint_i<N_links; joint_i++){
                joint_cmd_array.data.push_back( arr[joint_i] );    
            }
            
            pub_1.publish(joint_cmd_array);
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
    ROS_ERROR("--> Shutting Down. Stopping all motors");
    pub_1.publish(joint_cmd_array);

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

