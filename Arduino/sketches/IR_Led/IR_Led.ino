/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Bool.h>
#define IRLedPin 8

ros::NodeHandle nh;

void IRLed( const std_msgs::Bool& toggle_msg){
  if (toggle_msg.data)
    digitalWrite(IRLedPin, HIGH);   // turn the led ON
  else
    digitalWrite(IRLedPin, LOW);   // turn the led OFF
}

ros::Subscriber<std_msgs::Bool> sub("/robot_snake_1/toggle_led", &IRLed );

void setup()
{
  pinMode(IRLedPin, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
