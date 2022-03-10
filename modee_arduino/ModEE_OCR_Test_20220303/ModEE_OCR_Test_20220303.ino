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

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL);

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
  //dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_2, 30);
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_2, 10 );
}

volatile char cRxData;
volatile int iRxFlag;

String str_value;

bool record_flag = false;
float degree;
void serialEvent(){
    iRxFlag = 1;
    //cRxData = Serial.read();

    //Serial.write(Serial.read());

    str_value = Serial.readStringUntil('\n');
    Serial.println(str_value);
}

void loop() {
  // put your main code here, to run repeatedly:
  int i;
  float value_1;
  float value_2;
  int idx;
  int length;
  
  float f_present_position = 0.0;
  if(iRxFlag == 1){
        iRxFlag = 0;
        record_flag = true;

    idx = str_value.indexOf(" ");
    length = str_value.length();

    

    value_1 = (str_value.substring(0,idx)).toFloat();
    value_2 = (str_value.substring(idx+1,length)).toFloat();
    
    if (value_1 > 100) return;
    else {

      

      dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_1, 100 );
      dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_1, 1 );
      
      //if (dxl.setGoalPosition(DXL_ID_1, degree, UNIT_DEGREE) and dxl.setGoalVelocity(DXL_ID_2, value_2)) {
          
      //}
    }
  }

  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_1, 10 );
  dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID_1, 1 );
  // function : y=ax+b : degree=0.6x+180.0
    // 210.0: the degree when value is 0
    // 270.0: the degree when value is 100
    // a = (y2-y1)/(x2-x1) = (270.0-210.0)/(100-0) = 60/100 = 0.6
    
  degree = (0.5 * 0) + 220.0;
  dxl.setGoalPosition(DXL_ID_1, degree, UNIT_DEGREE);
  
  dxl.setGoalVelocity(DXL_ID_2, 0);
  delay(1000);
  dxl.setGoalVelocity(DXL_ID_2, 25);
  delay(1000);

  if(record_flag) {
    if(dxl.getPresentVelocity(DXL_ID_1) < 70) {
      DEBUG_SERIAL.println(dxl.getPresentVelocity(DXL_ID_1)); 
    }
    if((dxl.getPresentPosition(DXL_ID_1, UNIT_DEGREE) > degree-3.0) and (dxl.getPresentPosition(DXL_ID_1, UNIT_DEGREE) < degree+3.0) and (dxl.getPresentVelocity(DXL_ID_1)) == 0.0) {
      record_flag = false;
    }
    
  }


}
