// Shmulik Edelman
// shmulike@post.bgu.ac.il
//------------------------------------------------------------

// -------------------- Includer directories --------------------
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "math.h"

// -------------------- Define constants --------------------
#define N_links 1
#define angle_limit 20
#define w 0.01
// -------------------- Global variables --------------------
float joint_val[N_links]={0};
std_msgs::Float32MultiArray joint_cmd;
float count = 0;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_1");
    ros::NodeHandle n;
    // -------------------- Piblishers --------------------
    ros::Publisher  pub_1 = n.advertise<std_msgs::Float32MultiArray>("/robot_snake_1/joint_cmd", 1000);
    ros::Rate loop_rate(100);
    ROS_INFO("-> Sending joint command !");
    while (ros::ok())
    { 
        
        for (int i=0; i<N_links;i++){
            joint_val[i] = angle_limit * sin(M_PI*w*count/180);
        }
        joint_cmd.data.clear();
        
        for (int i=0; i<N_links; i++){
            joint_cmd.data.push_back( joint_val[i] );
        }
        

        count++;
        pub_1.publish(joint_cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
    return 0;
}
