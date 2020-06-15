#include "my_header.h"

/*
void flat_matrix(float **from, float to[], int N_size){
    for (int i=0; i<N_size; i++){
        for (int j=0; j<2; j++){
            to[i*2+j] = from[j][i];
        }
    }
}

void flat_matrix_2(float **from, std_msgs::Int32MultiArray to, int N_size){
    to.data.clear();
    for (int i=0; i<N_size; i++){
        for (int j=0; j<2; j++){
            //to[i*2+j] = from[j][i];
            to.data.push_back( round(from[j][i]) );
        }
    }
}

void vec_add (float *vec_A, float *vec_B, float *vec_res, int N_size){
    for (int i=0; i<N_size; i++){
        vec_res[i] = vec_A[i] + vec_B[i];
    }
}

void vec_sub (float *vec_A, float *vec_B, float *vec_res, int N_size){
    for (int i=0; i<N_size; i++){
        vec_res[i] = vec_A[i] - vec_B[i];
    }
}

void vec_dot (float *vec_A, float *vec_B, float *vec_res, int N_size){
    for (int i=0; i<N_size; i++){
        vec_res[i] = vec_A[i] * vec_B[i];
    }
}
*/

int limit(float num, float lim){
    if (num>lim)
        return lim;
    else if (num<-lim)
        return -lim;
    return round(num);
}

double tension_to_Kg (int LoadCellNum, int val){
    double P[8][3] = {{0.00000000,  0.00002348, 1.47896687},
                     {0.00000000,  0.00002385,	2.88188982},
                     {0.00000000,  0.00002414,	1.27118260},
                     {0.00000000,  0.00002028,	-4.84178993},
                     {0.00000000,  0.00002322,	3.38832415},
                     {0.00000000,  0.00002559,	4.95853685},
                     {0.00000000,	0.00002408,	6.59883064},
                     {0.00000000,	0.00002436,	1.13493471}};
    double res = P[LoadCellNum][0]*pow(val,2) + P[LoadCellNum][1]*val + P[LoadCellNum][2];
    return(res);
}

float signOf (float num){
    if (num>=0)
        return 1;
    return -1;
}