//==========================================================
//Shmulik Edelman
//shmulike@post.bgu.ac.il
// This code reading form MULTI HX711 Load-cells aplifier and sending there value as ROS-message
//==========================================================

//=====[ INCULDE ]==========================================
#include "HX711-multi.h"
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

//=====[ Constants ]========================================
#define PRINT 0
#define N_joints 4
#define CLK A0      // clock pin to the load cell amp
#define DOUT1 A1    // data pin to the first lca
#define DOUT2 A2    // data pin to the second lca
#define DOUT3 A3    // data pin to the third lca
#define DOUT4 A4    // data pin to the third lca
#define DOUT5 A5    // data pin to the third lca
#define DOUT6 A6    // data pin to the third lca
#define DOUT7 A7    // data pin to the third lca
#define DOUT8 A8    // data pin to the third lca
#define BOOT_MESSAGE "MIT_ML_SCALE V0.8"
#define TARE_TIMEOUT_SECONDS 4
//#define CHANNEL_COUNT sizeof(DOUTS)/sizeof(byte)
#define N_tensions 8

//=====[ VARIABLES ]=========================================
ros::NodeHandle nh;
byte DOUTS[N_tensions] = {DOUT1, DOUT2, DOUT3, DOUT4, DOUT5, DOUT6, DOUT7, DOUT8};
int tension_order[N_tensions] = {1, 2, 5, 6, 3, 4, 7, 8};
int k;
long int result[N_tensions];
//double result[N_tensions];
float arr[N_tensions]={0};
HX711MULTI scales(N_joints*2, DOUTS, CLK);

std_msgs::Float32MultiArray tension_val;
ros::Publisher pub_tensions("/robot_snake_1/tension_val", &tension_val);

   double P[N_tensions][3] = {{0.00000000, 0.00002348, 1.47896687},
                              {0.00000000, 0.00002385, 2.88188982},
                              {0.00000000, 0.00002414, 1.27118260},
                              {0.00000000, 0.00002028,-4.84178993},
                              {0.00000000, 0.00002322, 3.38832415},
                              {0.00000000, 0.00002559, 4.95853685},
                              {0.00000000, 0.00002408, 6.59883064},
                              {0.00000000, 0.00002436, 1.13493471}};
                     
//=====[ SETUP ]=============================================
void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_tensions);
  tension_val.data_length = N_tensions;
  if (PRINT){
    Serial.begin(115200);
    while(!Serial);
    Serial.println("Serial Communication started");
  }
}

//=====[ LOOP ]==============================================
void loop() {
  scales.readRaw(result);                // Read raw data from all the tension-meters
  //scales.readRawAvg(result, 10);

  // ---------- Convert RAW data to Kg
  for (int i=0; i<(N_joints*2); i++){
    k = tension_order[i]-1;
    arr[i] = P[k][0]*pow(result[k],2) + P[k][1]*result[k] + P[k][2];
    //Serial.print(result[k]);
    //Serial.print(arr[i]);
    //Serial.print(" ; ");
  }

  // PRINT ALL VALUES
  if (PRINT){
    Serial.print("Tension: ");
    for (int i=0; i<(N_joints*2); i++){
      //Serial.print(result[k]);
      //Serial.print("Joint ");
      Serial.print(arr[i],2);
      Serial.print(" ; ");
    }
    Serial.print('\n');
  }
  
  tension_val.data = arr;
  pub_tensions.publish(&tension_val);
  nh.spinOnce();
  delay(2);
}
