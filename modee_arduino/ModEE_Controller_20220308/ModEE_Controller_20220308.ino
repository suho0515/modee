/*******************************************************************************
  Copyright 2016 ROBOTIS CO., LTD.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*******************************************************************************/

#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Int16MultiArray.h>

#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#define DXL_SERIAL   Serial
#define DEBUG_SERIAL soft_serial
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
#define DXL_SERIAL   Serial
#define DEBUG_SERIAL SerialUSB
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL SerialUSB
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
#define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
// For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
// Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#else // Other boards when using DynamixelShield
#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif


const uint8_t DXL_ID_1 = 1;
const uint8_t DXL_ID_2 = 2;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;



// Test Data for height control
// 0~50mm
int height_input[50];


ros::NodeHandle nh;

int val = -1;

int command[6];

void volumeMsgCb( const std_msgs::Int8& msg){
  //value_control(1, 10, 10, int(msg.data));
  //char buf[10];
  //itoa(msg.data, buf, 10);
  //nh.loginfo(buf);
  val = msg.data;

}

ros::Subscriber<std_msgs::Int8> vol_sub("volume", volumeMsgCb );

void commandMsgCb( const std_msgs::String& msg){
  //char buf[10];
  //itoa(msg.data[0], buf, 10);
  //nh.loginfo(buf);
  //val = msg.data;

  command = msg.data;

}

ros::Subscriber<std_msgs::String> cmd_sub("controller_command", commandMsgCb );

void setup() {
  // put your setup code here, to run once:


  // Use UART port of DYNAMIXEL Shield to debug.
  //DEBUG_SERIAL.begin(115200);
  //while (!DEBUG_SERIAL);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID_1);
  dxl.ping(DXL_ID_2);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID_1);
  dxl.setOperatingMode(DXL_ID_1, OP_POSITION);
  dxl.torqueOn(DXL_ID_1);
  dxl.torqueOff(DXL_ID_2);
  dxl.setOperatingMode(DXL_ID_2, OP_VELOCITY);
  dxl.torqueOn(DXL_ID_2);

  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_1, 30 );
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_1, 10 );
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_2, 30);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_2, 10);

  // ROS Set
  Serial.begin(115200);
  nh.initNode();
  //nh.subscribe(vol_sub);
  nh.subscribe(cmd_sub);
  
  /*
  int i;
  for(i=0; i < 50; i = i+1) {
    height_input[i] = i;
  }
  */

  
  // height_to_degree test
  /*
  int i;
  int degree;
  for(i=-25; i <= 25; i = i+1) {
    degree = height_to_degree(i);
    Serial.println(degree);
  }
  */

  // value_to_height
  /*
  int i;
  float height;
  for(i=0; i <= 100; i = i+1) {
    height = value_to_height(i);
    Serial.println(height);
  }
  */

  // value_to_degree
  /*
  int i;
  float degree;
  for(i=0; i <= 100; i = i+1) {
    degree = value_to_degree(i);
    Serial.println(degree);
  }
  */
  
  // height_control test
  /*
  int i;
  for(i=-25; i <= 25; i = i+1) {
    height_control(1, 30, 30, i);
  }
  */

  // value_control test
  /*
  int i;
  for(i=0; i <= 100; i = i+1) {
    value_control(1, 30, 30, i);
  }
  */
  
  // move test
  /*
  int value_before = 0;
  int value_after = 100;
  move(value_before, value_after);
  */
  
  

}

void loop() {  
  nh.spinOnce();

  
  if(command[0]==2) {
    
    int goal_vol = command[5];
    
    //char buf[10];
    //itoa(goal_vol, buf, 10);
    //nh.loginfo(buf);
    //nh.loginfo(String(volume));
    value_control(1, 30, 30, val); 
    if(val < goal_vol){
      move_up(goal_vol);
    }
    else if(val > goal_vol){
      move_down(goal_vol);
    }
    else {
      nh.loginfo("work is done");
    }
    
  }
  
  //int value_before = 0;
  //int value_after = 100;
  //move(value_before, value_after);

  // for ros communication
  // https://answers.ros.org/question/214256/fake-laserscan-on-arduino-2560-the-laserscan-data-seems-to-transfer-correctly-except-the-program-hangs-up-after-11-18-successful-data-transfers/
  //delay(10);
}





boolean rotary_control(const uint8_t DXL_ID, int velocity, int acceleration) {
  /*
  // error if velocity is bigger than velocity_limit
  int velocity_limit;
  velocity_limit = abs(dxl.readControlTableItem(VELOCITY_LIMIT, DXL_ID, 10));
  if(velocity > velocity_limit) return false;

  // error if velocity is bigger than velocity_limit
  int acceleration_limit;
  acceleration_limit = abs(dxl.readControlTableItem(ACCELERATION_LIMIT, DXL_ID, 10));
  if(acceleration > acceleration_limit) return false;
  */
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, acceleration );

  if(dxl.setGoalVelocity(DXL_ID, velocity)){
      //delay(1000);
      //DEBUG_SERIAL.print("Present Velocity : ");
      //DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_ID_2)); DEBUG_SERIAL.println();
  }
  else return false;

  // return if process is succesful
  return true;  
}


boolean height_control(const uint8_t DXL_ID, int velocity, int acceleration, float height) {

  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, velocity );
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, acceleration );

  float degree;
  degree = height_to_degree(height);
  Serial.println(String(degree));

  if(dxl.setGoalPosition(DXL_ID, degree, UNIT_DEGREE)){
      //delay(1000);
      //DEBUG_SERIAL.print("Present Position : ");
      //DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE)); DEBUG_SERIAL.println();
  }
  else return false;

  // return if process is succesful
  return true;  
}

boolean value_control(const uint8_t DXL_ID, int velocity, int acceleration, int value) {

  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, velocity );
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID, acceleration );

  float height;
  height = value_to_height(float(value));
  //Serial.println(String(degree));
  
  float degree;
  degree = height_to_degree(height);

  if(dxl.setGoalPosition(DXL_ID, degree, UNIT_DEGREE)){
      //delay(1000);
      //DEBUG_SERIAL.print("Present Position : ");
      //DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE)); DEBUG_SERIAL.println();
  }
  else return false;

  // return if process is succesful
  return true;  
}

float height_to_degree(float height) {
  // 25.0 : length of cam follower
  // 25mm is highst height
  // -25mm is lowest height
  //height = height-25.0;
  float degree = 180.0 - ((asin(height/25.0)*180.0)/PI+90.0);
  //if(degree < 0) degree = 90.0 + degree;
  return degree;
}

float value_to_height(int value) {
  // displacement of value 0 to 100: 21.5mm
  // value 0: height -25.0mm
  // value 100: height -5.0mm
  // y = ax + b -> height = a*value + b
  // a = (y2 - y1)/(x2 - x1) = (-5.0 - -25.0)/(100 - 0) = 20/100 = 0.2
  float height = (0.2*float(value))- 25.0;
  return height;
}

float value_to_degree(int value) {
  float height;
  height = value_to_height(value);

  float degree;
  degree = height_to_degree(height);
  
  return degree;
}


boolean move(int value_before, int value_after) {
  value_control(1, 2, 1, value_before);
  float goal_degree = value_to_degree(value_before);
  rotary_control(2, 40, 20);
  while(dxl.getPresentPosition(1, UNIT_DEGREE) < goal_degree - 3.0 or dxl.getPresentPosition(1, UNIT_DEGREE) > goal_degree + 3.0);
  rotary_control(2, 0, 10);
  while(dxl.getPresentVelocity(2) != 0);

  value_control(1, 2, 1, value_after);
  goal_degree = value_to_degree(value_after);
  rotary_control(2, -40, 20);
  while(dxl.getPresentPosition(1, UNIT_DEGREE) < goal_degree - 3.0 or dxl.getPresentPosition(1, UNIT_DEGREE) > goal_degree + 3.0);
  rotary_control(2, 0, 10);
  while(dxl.getPresentVelocity(2) != 0);
}

boolean move_down(int value) {
  value_control(1, 2, 1, value);
  float goal_degree = value_to_degree(value);
  rotary_control(2, 40, 20);
  while(dxl.getPresentPosition(1, UNIT_DEGREE) < goal_degree - 3.0 or dxl.getPresentPosition(1, UNIT_DEGREE) > goal_degree + 3.0);
  rotary_control(2, 0, 10);
  while(dxl.getPresentVelocity(2) != 0);
}

boolean move_up(int value) {
  value_control(1, 2, 1, value);
  float goal_degree = value_to_degree(value);
  rotary_control(2, -40, 20);
  while(dxl.getPresentPosition(1, UNIT_DEGREE) < goal_degree - 3.0 or dxl.getPresentPosition(1, UNIT_DEGREE) > goal_degree + 3.0);
  rotary_control(2, 0, 10);
  while(dxl.getPresentVelocity(2) != 0);
}
