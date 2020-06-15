//==============================================================================
//Shmulik Edelman
//shmulike@post.bgu.ac.il
//==============================================================================

//=====[ INCULDE ]==============================================================
#include "RLS_Encoder.h"
#include <i2c_t3.h>

//=====[ Constants ]========================================
//#define slave_1 8
//=====[ VARIABLES ]============================================================
RLS_Encoder enc;
float value = 0;
uint8_t slave_1 = 100;

//=====[ Function declaraion ]========================================
void requestEvent();

//=====[ SETUP ]================================================================
void setup() {
  //Serial.begin(115200); while(!Serial);
  enc.begin(); delay(5);
  enc.set_read(); delay(5);
  Wire.begin(I2C_SLAVE, slave_1, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  delay(10);
  Wire.onRequest(requestEvent);
  value = 0;
}

//=====[ LOOP ]=================================================================
void loop() {

  Serial2.flush();
  while (Serial2.available()){
    value = enc.get_pos();
    Serial.println(value,3);
    break;
  }
  /*value = enc.get_pos();
  Serial.println(value,3);*/
  delay(2);
}
//==============================================================================

void requestEvent(){      
  //digitalWrite(13,HIGH); delay(100);
  //digitalWrite(13,LOW);  delay(100);
  
  Serial2.flush();
  // Read first=this encoder position
  while (Serial2.available()){
    //if (Serial2.read()=='1'){
      value = enc.get_pos();
      break;
    //}
    }
  byte *data = (byte *)&value;
  Wire.write(data[0]);
  Wire.write(data[1]);
  Wire.write(data[2]);
  Wire.write(data[3]);
}
