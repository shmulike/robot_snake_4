//==============================================================================
//Shmulik Edelman
//shmulike@post.bgu.ac.il
//==============================================================================

//=====[ INCULDE ]==============================================================
#include "RLS_Encoder.h"
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <i2c_t3.h>
#define PRINT 0

//=====[ Constants ]========================================
#define N_links 4
//float joint_offset[] = {91.46, 277.87, 268.33, 92.277};
float joint_offset[N_links] = {4, 0, 0, 5};

union u_tag {
  byte b[4];
  float fval;
} u;

//=====[ VARIABLES ]============================================================
RLS_Encoder enc;
ros::NodeHandle nh;
byte data[4] = {0};
//int value = 0;
float arr[N_links] = {0};
std_msgs::Float32MultiArray joint_ang;
ros::Publisher pub_joints("/robot_snake_1/joint_val", &joint_ang);
int count = 0;
uint8_t slave_add[3] = {100, 101, 102};
uint8_t slave_1 = 0x66;

//=====[ FUNCTIONS ]================================================================
float wrapTo180(float val) {
  return (val > 180) ? val - 360.0 : val;
}
//=====[ SETUP ]================================================================
void setup() {
  Serial.begin(115200); while (!Serial);
  //Serial.println("Start");
  enc.begin();
  //enc.set_read();
  //enc.start_response();
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_joints);
  joint_ang.data_length = N_links;
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  //Wire.setDefaultTimeout(250000); // 250ms default timeout
}

//=====[ LOOP ]=================================================================
void loop() {
  //Serial.println(x++);
  //Serial.print("line:\t");

  // Read first=this encoder position

  Serial2.flush();
  /*
    while (Serial2.available()){
      if (Serial2.read()=='1'){
        arr[0] = enc.get_pos()-joint_offset[0];
        break;
        }
    }
  */

  // Read others encoder position
  arr[0] = wrapTo180(enc.get_pos() - joint_offset[0]);
  for (int i = 1; i < N_links; i++) {
    Wire.requestFrom(slave_add[i - 1], sizeof(data));
    if (Wire.getError()) {
      Serial.print("--Wire Error: ");
      Serial.print(Wire.getError());
    }
    else {
      Wire.read(data, Wire.available());
      for (int i = 0; i < 4; i++)
        u.b[i] = data[i];
      //arr[i] = u.fval - joint_offset[i];
      arr[i] = wrapTo180(u.fval - joint_offset[i]);
    }
  }

  // print Encoder values
  if (PRINT) {
    for (int i = 0; i < N_links; i++) {
      Serial.print("val ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(arr[i], 3);
      Serial.print("\t");
    }
    Serial.print("\n");
  }
  joint_ang.data = arr;
  pub_joints.publish(&joint_ang);
  nh.spinOnce();
  delay(2);
}
//==============================================================================
