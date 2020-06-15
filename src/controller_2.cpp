// Shmulik Edelman
// shmulike@post.bgu.ac.il
//------------------------------------------------------------

// -------------------- Includer directories --------------------
#include "ros/ros.h"
#include "ros/time.h"
#include <signal.h>
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "math.h"

#include <iostream>
#include <stack>
#include <ctime>
#include "../include/my_header.h"
//#include <chrono>
//#include <iostream>

#include <iostream>
#include <cstdio>
#include <ctime>

#define N_links 4
#define N_tensions N_links*2
#define N_motors N_links*2

#define string_left 0
#define string_right 1

#define eps_angle 0.06

#define Kp_tension 100       //40
#define Kd_tension 0      
#define Ki_tension 1       //1
#define eps_tension 0.03
//#define MAX_PWM_tension 80                     // 255

#define Ki_limit 800
//-----------------------------------------------------------
#define limit_angle_error 40          // joint angle limit [deg]
#define limit_angle_warn 35
#define limit_max_tension 12        // String tension limit [Kg]
#define limit_min_tension 0.1        // String tension limit [Kg]

// -------------------- Global variables --------------------




float Kp_angle[N_links]={25,20,15,15};
float Ki_angle[N_links]={0.1 ,0.05 ,0.05 ,0.020 };
float Kd_angle[N_links]={2,2,2,2};

float MAX_PWM_angle[N_links]={110,100,90,70};

float MAX_PWM_tension[N_links]={110,50,80,50};


//double string_tension[N_links]={1,1.5,1.2,1.8};
double string_tension[N_links]={1.5,2.0,1.7,2.3};

double joint_error[N_links] = {0}, joint_error_sum[N_links] = {0}, joint_previous_error[N_links] = {0};
float joint_val[N_links]={0}, joint_cmd[N_links]={0};

float linear_val = 0, linear_cmd = 0;
int motor_cmd[2][N_links] = {0}, motor_cmd_flat[N_links*2] = {0};
float tension_val[2][N_links] = {0};
float tension_error[2][N_links] = {0}, tension_error_sum[2][N_links] = {0}, tension_previous_error[2][N_links] = {0};
float pwm_temp = 0;
std_msgs::Int32MultiArray motor_cmd_PWM;
bool alive[2] = {0};        // [0] Joints -- [1] Tensions
ros::Publisher pub_1;
// -------------------- Functions defenition --------------------
void get_joint_val(const std_msgs::Float32MultiArray::ConstPtr& msg);
void get_joint_cmd(const std_msgs::Float32MultiArray::ConstPtr& msg);
void get_linear_val(const std_msgs::Float32::ConstPtr& msg);
void get_linear_cmd(const std_msgs::Float32::ConstPtr& msg);
void get_tension_val(const std_msgs::Float32MultiArray::ConstPtr& msg);
void mySigintHandler(int sig);


std::stack<clock_t> tictoc_stack;


std::clock_t static tic(){ return std::clock(); }

double static toc(std::clock_t start){
     return ((std::clock() - start ) /((double) CLOCKS_PER_SEC))*1000;}

        std::clock_t temp_time=std::clock();
        std::clock_t time_joint[N_links]={temp_time,temp_time,temp_time,temp_time};
        std::clock_t time_tension[N_links]={temp_time,temp_time,temp_time,temp_time};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_1", ros::init_options::NoSigintHandler);
    //ros::init(argc, argv, "controller_1");
    ros::NodeHandle n;
    signal(SIGINT, mySigintHandler);
    // -------------------- Piblishers --------------------
    pub_1 = n.advertise<std_msgs::Int32MultiArray>("/robot_snake_1/motor_cmd", 1000);
    // -------------------- Subscribers --------------------
    ros::Subscriber sub_1 = n.subscribe("/robot_snake_1/joint_val",     1000, get_joint_val);
    ros::Subscriber sub_2 = n.subscribe("/robot_snake_1/joint_cmd",     1000, get_joint_cmd);
    ros::Subscriber sub_3 = n.subscribe("/robot_snake_1/linear_val",    1000, get_linear_val);
    ros::Subscriber sub_4 = n.subscribe("/robot_snake_1/linear_cmd",    1000, get_linear_cmd);
    ros::Subscriber sub_5 = n.subscribe("/robot_snake_1/tension_val",   1000, get_tension_val);
    ros::Rate loop_rate(100);

    //ROS_WARN("--> Waiting for all nodes to publish !");
    //while (!alive[0]) ROS_INFO("teensy not working %d", alive[0]);
    //while (!alive[1]) ROS_INFO("tension not working %d", alive[1]);
    //while (!alive[0] && !alive[1]);
    //ROS_INFO("-> wait 5 sec");
    //ros::Duration(5, 0).sleep();
    ROS_WARN("-> Controller node launched !");
    while (ros::ok())
    { 
        if (!alive[0] || !alive[1])
        {
            if (!alive[0])
                ROS_ERROR("Waiting for teensy 'Joints' to wake up");
            if (!alive[1])
                ROS_ERROR("Waiting for teensy 'TenstionMeter' to wake up");
            ros::Duration(2, 0).sleep();
        }
        else
        {



            for (int joint_i=0; joint_i<N_links; joint_i+=2)
            {
               
                joint_previous_error[joint_i] = joint_error[joint_i]; /////
                joint_error[joint_i] = joint_cmd[joint_i] - joint_val[joint_i];
                double diff_angle=((joint_error[joint_i]-joint_previous_error[joint_i])/toc(time_joint[joint_i]));

                time_joint[joint_i]=tic();

                if (fabs(joint_error_sum[joint_i])<Ki_limit){
                    joint_error_sum[joint_i] += joint_error[joint_i];
                }
                if (fabs(joint_error[joint_i]) < eps_angle){
                    joint_error_sum[joint_i]=0;
                }

                pwm_temp = joint_error[joint_i]*Kp_angle[joint_i] + joint_error_sum[joint_i]*Ki_angle[joint_i] + diff_angle*Kd_angle[N_links];
                motor_cmd[0][joint_i] = limit(pwm_temp, MAX_PWM_angle[joint_i]);


                tension_previous_error[1][joint_i] = tension_error[1][joint_i] ;   
                tension_error[1][joint_i] = (string_tension[joint_i] - tension_val[1][joint_i]);

                double diff_tension=((tension_error[1][joint_i]-tension_previous_error[1][joint_i])/toc(time_tension[joint_i]));
                time_tension[joint_i]=tic();

                if (fabs(tension_error_sum[1][joint_i])<Ki_limit){
                    tension_error_sum[1][joint_i] += tension_error[1][joint_i];
                }
                if (fabs(tension_error[1][joint_i]) < eps_tension){
                    tension_error_sum[1][joint_i]=0;
                }

                    pwm_temp=tension_error[1][joint_i]*Kp_tension + tension_error_sum[1][joint_i]*Ki_tension + diff_tension*Kd_tension;
                    motor_cmd[1][joint_i] = limit(pwm_temp, MAX_PWM_tension[N_links]);

                if (fabs(joint_val[joint_i])>limit_angle_warn){
                    joint_error_sum[joint_i] = 0;
                  ROS_WARN("-> Joint #%d - angle got to its limit: %.2f",joint_i, joint_val[joint_i]);
                    if (fabs(joint_val[joint_i])>limit_angle_error){
                        joint_error_sum[joint_i] = 0;
                        motor_cmd[0][joint_i] = -MAX_PWM_angle[N_links]*signOf(joint_val[joint_i]);
                        motor_cmd[1][joint_i] = 0;
                        ROS_ERROR("-> Joint #%d - angle got to its MAX limit: %.2f\t--> BacKi_angleng off",joint_i,joint_val[joint_i]);
                    }
                }
                
                for (int j=0; j<2; j++){
                    if (tension_val[j][joint_i]>limit_max_tension){
                        motor_cmd[0][joint_i] = 0;
                        motor_cmd[1][joint_i] = -MAX_PWM_tension[N_links];
                        ROS_FATAL("String #%d - string got to its max limit: %.2f",2*joint_i+j,tension_val[j][joint_i]);
                    }
                    else if(tension_val[j][joint_i]<limit_min_tension)
                    {
                        motor_cmd[1][joint_i] = MAX_PWM_tension[N_links]*2;
                        ROS_FATAL("String #%d - string got to its min limit: %.2f",2*joint_i+j,tension_val[j][joint_i]);

                    }
                }
                
                // ---------- End check limits
                
               ROS_INFO("Joint #%d:\tPos/cmd= %.3f/%.1f;\tPWM=[%d,%d];\tT1= %.2f;\tT2= %.2f\tE_sum=%.4f\tkP=%.4f\tE_sum=%.4f\tkP=%.4f",joint_i, joint_val[joint_i],joint_cmd[joint_i], motor_cmd[0][joint_i], motor_cmd[1][joint_i], tension_val[0][joint_i], tension_val[1][joint_i],joint_error_sum[joint_i],joint_error[joint_i]*Kp_angle[joint_i],tension_error[1][joint_i]*Kp_tension , tension_error_sum[1][joint_i]);
            } // End of links loop
            for (int joint_i=1; joint_i<N_links; joint_i+=2)
            {
               
                joint_previous_error[joint_i] = joint_error[joint_i]; /////
                joint_error[joint_i] = joint_cmd[joint_i] - joint_val[joint_i];
                double diff_angle=((joint_error[joint_i]-joint_previous_error[joint_i])/toc(time_joint[joint_i]));

                time_joint[joint_i]=tic();

                if (fabs(joint_error_sum[joint_i])<Ki_limit){
                    joint_error_sum[joint_i] += joint_error[joint_i];
                }
                if (fabs(joint_error[joint_i]) < eps_angle){
                    joint_error_sum[joint_i]=0;
                }

                tension_previous_error[string_left][joint_i] = tension_error[string_left][joint_i] ;   
                tension_error[string_left][joint_i] = (string_tension[joint_i] - tension_val[string_left][joint_i]);

                double diff_tension=((tension_error[string_left][joint_i]-tension_previous_error[string_left][joint_i])/toc(time_tension[joint_i]));
                time_tension[joint_i]=tic();

                if (fabs(tension_error_sum[string_left][joint_i])<Ki_limit){
                    tension_error_sum[string_left][joint_i] += tension_error[string_left][joint_i];
                }
                if (fabs(tension_error[string_left][joint_i]) < eps_tension){
                    tension_error_sum[string_left][joint_i]=0;
                }



                pwm_temp = joint_error[joint_i]*Kp_angle[joint_i] + joint_error_sum[joint_i]*Ki_angle[joint_i] + diff_angle*Kd_angle[N_links];
                motor_cmd[string_left][joint_i] = limit(pwm_temp, MAX_PWM_angle[joint_i]);
                
                pwm_temp=tension_error[string_left][joint_i]*Kp_tension + tension_error_sum[string_left][joint_i]*Ki_tension + diff_tension*Kd_tension;
                motor_cmd[string_left][joint_i] = limit(pwm_temp, MAX_PWM_tension[N_links]);
                
                if (fabs(joint_val[joint_i])>limit_angle_warn){
                    joint_error_sum[joint_i] = 0;
                  ROS_WARN("-> Joint #%d - angle got to its limit: %.2f",joint_i, joint_val[joint_i]);
                    if (fabs(joint_val[joint_i])>limit_angle_error){
                        joint_error_sum[joint_i] = 0;
                        motor_cmd[string_left][joint_i] = -MAX_PWM_angle[N_links]*signOf(joint_val[joint_i]);
                        motor_cmd[string_right][joint_i] = 0;
                        ROS_ERROR("-> Joint #%d - angle got to its MAX limit: %.2f\t--> BacKi_angleng off",joint_i,joint_val[joint_i]);
                    }
                }
                
                for (int j=0; j<2; j++){
                    if (tension_val[j][joint_i]>limit_max_tension){
                        motor_cmd[string_left][joint_i] = 0;
                        motor_cmd[string_right][joint_i] = -MAX_PWM_tension[N_links];
                        ROS_FATAL("String #%d - string got to its max limit: %.2f",2*joint_i+j,tension_val[j][joint_i]);
                    }
                    else if(tension_val[j][joint_i]<limit_min_tension)
                    {
                        motor_cmd[string_right][joint_i] = MAX_PWM_tension[N_links];
                        ROS_FATAL("String #%d - string got to its min limit: %.2f",2*joint_i+j,tension_val[j][joint_i]);

                    }
                }
                
                // ---------- End check limits
                
               ROS_INFO("Joint #%d:\tPos/cmd= %.3f/%.1f;\tPWM=[%d,%d];\tT1= %.2f;\tT2= %.2f\tE_sum=%.4f\tkP=%.4f\tE_sum=%.4f\tkP=%.4f",joint_i, joint_val[joint_i],joint_cmd[joint_i], motor_cmd[0][joint_i], motor_cmd[1][joint_i], tension_val[0][joint_i], tension_val[1][joint_i],joint_error_sum[joint_i],joint_error[joint_i]*Kp_angle[joint_i],tension_error[1][joint_i]*Kp_tension , tension_error_sum[1][joint_i]);
            } // End of links loop
            ROS_INFO("--------------------");

            /*for (int joint_i=0; joint_i<2; joint_i++){
                for (int j=0; j<2; j++){
                    motor_cmd[j][joint_i] = 0;
                }
            }*/

            motor_cmd_PWM.data.clear();
            for (int joint_i=0; joint_i<N_links; joint_i++){
                for (int j=0; j<2; j++){
                  motor_cmd_PWM.data.push_back(motor_cmd[j][joint_i] );
                    
                }
            }
            
            pub_1.publish(motor_cmd_PWM);
        } // end of else
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
    return 0;
}

// ------------------------------------------------------------
// -------------------- Functions --------------------
void get_joint_val(const std_msgs::Float32MultiArray::ConstPtr& msg){
    alive[0] = true;
    for (int i=0; i<N_links; i++){
        //joint_val[i] = msg->data[i] - joint_offset[i];
        joint_val[i] = msg->data[i];
    }
}

void get_joint_cmd(const std_msgs::Float32MultiArray::ConstPtr& msg){
    for (int i=0; i<N_links; i++){
        joint_cmd[i] = msg->data[i];
    }
}

void get_linear_val(const std_msgs::Float32::ConstPtr& msg){
    linear_val = msg->data;
}

void get_linear_cmd(const std_msgs::Float32::ConstPtr& msg){
    linear_cmd = msg->data;
}

void get_tension_val(const std_msgs::Float32MultiArray::ConstPtr& msg){
    alive[1] = true;
    for (int joint_i=0; joint_i<N_links; joint_i++){
        for (int j=0; j<2; j++){
            tension_val[j][joint_i] =  fmax(msg->data[joint_i*2+j],0.0);
            //tension_val[j][i] = tension_to_Kg (i*2+j, msg->data[i*2+j]);
        }
    }
}

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
    motor_cmd_PWM.data.clear();
    for (int joint_i=0; joint_i<N_links*2; joint_i++){
        motor_cmd_PWM.data.push_back( 0 );
    }
    ROS_ERROR("--> Shutting Down. Stopping all motors\tsig: %d", sig);
    pub_1.publish(motor_cmd_PWM);

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

// -------------------- Move to new header file

