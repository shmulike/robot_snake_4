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
#define N_links 4
#define PI 3.14159265
#define step 10000.0
#define step_factor 3.0

// -------------------- Global variables --------------------

//#define MAX_PWM_angle 65                     // 255
float counter = 0;
int fase = 1;
std_msgs:: Float32MultiArray joint_cmd_array;
float arr[N_links] = {0.0};
ros::Publisher pub_1;

// -------------------- Functions defenition --------------------

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
            if (counter < 0.25)                     // GO UP
            {
                counter += 2.0*PI/(step*step_factor);
                fase = 1;
            }
            else if (counter >= 0.25 && counter < 2)              // GO from center to RIGHT->LEFT->center
            {
                if (fase==1) counter = 1;           // If we go from fase 1 to 2 this will happen only at the first time
                counter += 2.0*PI/(step*step_factor);
                fase = 2;
            }
            else if (counter >= 2 && counter < 2.5)                  // GO down
            {
                counter += 2.0*PI/(step*step_factor);
                fase = 3;
            }
            else if (counter >=2.5 && counter < 4)                 // GO from center to RIGHT->LEFT->center
            {
                if (fase==3) counter = 3;
                counter += 2.0*PI/(step*step_factor);
                fase = 4;
            }
            else if (counter >=4 && counter < 4.25)                 // GO from center to RIGHT->LEFT->center
            {
                counter += 2.0*PI/(step*step_factor);
                fase = 5;
            }


            switch(fase){
                case 1:
                arr[0] =  sinAmp1 * sin(counter*2*PI);
                arr[1] = 0;
                arr[2] = -sinAmp1 * sin(counter*2*PI);
                arr[3] = 0;
                break;

                case 2:
                arr[0] = 0;
                arr[1] =  sinAmp1 * cos(counter*2*PI);
                arr[2] = 0;
                arr[3] = -sinAmp1 * cos(counter*2*PI);
                break;

                case 3:
                arr[0] = sinAmp1 * sin(counter*2*PI);;
                arr[1] = 0;
                arr[2] = -sinAmp1 * sin(counter*2*PI);
                arr[3] = 0;
                break;

                case 4:
                arr[0] = 0;
                arr[1] =  sinAmp1 * cos(counter*2*PI);
                arr[2] = 0;
                arr[3] = -sinAmp1 * cos(counter*2*PI);
                break;

                case 5:
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

