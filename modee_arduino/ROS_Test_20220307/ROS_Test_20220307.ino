#include <ros.h>
#include <std_msgs/Int8.h>

#include <std_msgs/String.h>

ros::NodeHandle nh;

void messageCb( const std_msgs::Int8& msg){
}

ros::Subscriber<std_msgs::Int8> sub("volume", messageCb );


void setup() {
  // put your setup code here, to run once:
  // ROS Set
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
}
