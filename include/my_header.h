#ifndef my_header
#define my_header

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"

//#define shmulik 5.5

//void flat_matrix(float **from, float to[], int N_size);
//void flat_matrix_2(float **from, std_msgs::Int32MultiArray to, int N_size);
//void vec_add (float vec_A[], float *vec_B, float *vec_res, int N_size);
//void vec_sub (float vec_A[], float *vec_B, float *vec_res, int N_size);
//void vec_dot (float vec_A[], float *vec_B, float *vec_res, int N_size);
int limit(float num, float lim);
double tension_to_Kg (int LoadCellNum, int val);
float signOf (float num);

#endif