#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Int16MultiArray
from dynamixel_workbench_msgs.msg import DynamixelStateList
from dynamixel_workbench_msgs.srv import DynamixelCommand, DynamixelCommandRequest
import numpy as np

class ModEE:

  def __init__(self):
    # Subscriber
    ## it should subscribe valid volume.
    self.vol_sub = rospy.Subscriber("valid_volume",Int8,self.volCallback)
    ## dynamixel state subscriber. it's needed for feedback control
    self.dxl_state_sub = rospy.Subscriber("/dynamixel_workbench/dynamixel_state",DynamixelStateList,self.dxlStateCallback)
    ## command input to control modee
    self.cmd_sub = rospy.Subscriber("command",Int16MultiArray,self.cmdCallback)
    ## center different percentage subscriber
    self.center_diff_percentage_sub = rospy.Subscriber("center_difference_percentage",Int8,self.centerDiffPercentageCallback)

    # Services
    ## command service which control dynamixels
    # Wait for the service client /dynamixel_workbench/dynamixel_command to be running
    dxl_command_service_name = "/dynamixel_workbench/dynamixel_command"
    rospy.wait_for_service(dxl_command_service_name)
    # Create the connection to the service
    self.dxl_command_service = rospy.ServiceProxy(dxl_command_service_name, DynamixelCommand)


    
    self.ctr_cmd_pub = rospy.Publisher("controller_command", Int16MultiArray, queue_size=10)
    self.state_pub = rospy.Publisher("controller_state", Int8, queue_size=10)

    # Timer
    ## we would use ros timer for processing tasks.
    self.timer = rospy.Timer(rospy.Duration(0.001), self.timerCallback)

    # Variables
    ## the valid volume.
    self.valid_vol = 0
    ## dynamixel state list
    self.dxl_state_list = []
    ## raw degree value of vertical motion unit actuator
    self.present_position = 0
    ## which action mode will do
    self.mode = 0
    self.vel_1 = 0
    self.acc_1 = 0
    self.vel_2 = 0
    self.acc_2 = 0
    self.goal_vol = 0
    ## feedback state
    self.feedback_state = 0
    self.feedback_flag = False
    self.goal_position = 0
    ## center different percentage
    self.center_diff_percentage = 0
    ## count for detailed control
    self.ctr_cnt = 0


    self.pre_sign = 0

    self.state = -1

    # draft test
    #self.control_position(1,0)

    # for i in range(360):
    #   position = self.degree_to_position(i)
    #   print(position)

    #self.control_degree(1,0)

    # for i in range(-25,25,1):
    #   degree = self.height_to_degree(i)
    #   print(degree)

    self.stop()
    self.control_height(1, 25)

    # for i in range(0,100,1):
    #   height = self.volume_to_height(i)
    #   print(height)

    #self.control_volume(1, 100)

    #self.move_to_origin()

    #self.move_to_present_volume()

    # for i in range(0,100,1):
    #   position = self.volume_to_position(i)
    #   print(position)

    #self.volume_down()
    #rospy.sleep(10)
    #self.stop()

    #self.set_operating_mode(1,2)



  def volCallback(self,data):
    """!
    @brief subscribe valid volume
    @details 
    
    @param[in] data: ros Int8 message
    
    @note input constraints: 
    @n - none
    @note output constraints: 
    @n - none
    @return none
    """
    self.cur_vol = data.data
    self.valid_vol = data.data
    #print(self.cur_vol)
    
  def dxlStateCallback(self,data):
    """!
    @brief subscribe dynamixel state message
    @details with recognizing the state of dynamixel, we can do feedback control.
    
    @param[in] data: ros DynamixelStateList message
    
    @note input constraints: 
    @n - none
    @note output constraints: 
    @n - none
    @return none
    """
    self.dxl_state_list = data
    self.present_position = self.dxl_state_list.dynamixel_state[1].present_position
    #print(self.dxl_state_list)
    #print(self.dxl_state_list.dynamixel_state[1])
    #print(self.dxl_state_list.dynamixel_state[1].present_position)
    #print(self.present_position)

  def cmdCallback(self,data):
    """!
    @brief subscribe dynamixel state message
    @details with recognizing the state of dynamixel, we can do feedback control.
    
    @param[in] data: ros DynamixelStateList message
    
    @note input constraints: 
    @n - none
    @note output constraints: 
    @n - none
    @return none
    """
    #print(data.data)
    self.mode = data.data[0]
    self.vel_1 = data.data[1]
    self.acc_1 = data.data[2]
    self.vel_2 = data.data[3]
    self.acc_2 = data.data[4]
    self.goal_vol = data.data[5]

  def centerDiffPercentageCallback(self, data):
    """!
    @brief subscribe dynamixel state message
    @details with recognizing the state of dynamixel, we can do feedback control.
    
    @param[in] data: ros DynamixelStateList message
    
    @note input constraints: 
    @n - none
    @note output constraints: 
    @n - none
    @return none
    """
    self.center_diff_percentage = data.data

  def timerCallback(self, *args):
    #print("Test")
    
    if self.mode == 0:
      #print("NOTHING MODE")
      ctrCmdMsg = Int16MultiArray()
      ctrCmdMsg.data = [0, 0]
      self.ctr_cmd_pub.publish(ctrCmdMsg)
    elif self.mode == 1:
      print("MANUAL MODE")
      
    elif self.mode == 2:
      print("FEEDBACK MODE")
      

      # if(self.feedback_state == 0):
        
      #   if(self.feedback_flag == False):
      #     self.feedback_flag = True
      #     self.move_to_origin()
        
      #   offset = 3
      #   if(self.present_position < 0+offset and self.present_position > 0-offset):
      #     self.feedback_state = 1
      #     self.feedback_flag = False
      #     print("")

      if(self.feedback_state == 0):
        
        if(self.feedback_flag == False):
          self.feedback_flag = True
          self.move_to_present_volume()
          self.valid_position = self.volume_to_position(self.valid_vol)
          self.goal_position = self.volume_to_position(self.goal_vol)
          
        print(self.valid_position)
        print(self.present_position)

        offset = 50
        if((self.present_position < self.valid_position + offset) and (self.present_position > self.valid_position - offset) ):
          self.feedback_state = 1
          self.feedback_flag = False
          self.stop()
          print("state 1 is complete")

      elif(self.feedback_state == 1):
        
        # if(self.feedback_flag == False):
        #   self.feedback_flag = True

        #sign = self.calc_sign(self.goal_vol, self.valid_vol)
        diff = self.calc_diff(self.goal_vol, self.valid_vol)
        print(diff)
        if(diff > 0):
          # move up
          print("MOVE_UP")
          if(abs(diff) < 3):
            self.volume_up(10, -20)
          else:
            self.volume_up(30, -60)
        elif(diff < 0):
          # move down
          print("MOVE_DOWN")
          if(abs(diff) < 3):
            self.volume_down(10, 20)
          else:
            self.volume_down(30, 60)
        else:
          # stop modee
          print("STOP")
          self.stop()
          #self.move_to_origin()
          self.feedback_state = 2
          #self.mode = 0

      elif(self.feedback_state == 2):
        if(self.center_diff_percentage > 10):
          self.volume_down(5, 10)
          self.stop()
          self.ctr_cnt = 0
        elif(self.center_diff_percentage < -10):
          self.volume_up(5, -10)
          self.stop()
          self.ctr_cnt = 0
        else:
          if(self.ctr_cnt > 1000):
            # stop modee
            print("STOP")
            self.stop()
            self.move_to_origin()
            self.feedback_state = 0
            self.mode = 0
            self.ctr_cnt = 0
          self.ctr_cnt = self.ctr_cnt + 1

      print(self.feedback_state)


      # if(self.feedback_flag == False):
      #   self.feedback_flag = True
        
      #   self.move_to_origin()
        # self.move_to_present_volume()

      #   sign = self.calc_sign(self.goal_vol, self.cur_vol)
      #   if(sign > 0):
      #     # move up
      #     print("MOVE_UP")
      #     self.state = 1
      #     ctrCmdMsg = Int16MultiArray()
      #     ctrCmdMsg.data = [1, self.goal_vol]
      #     self.ctr_cmd_pub.publish(ctrCmdMsg)
      #   elif(sign < 0):
      #     # move down
      #     print("MOVE_DOWN")
      #     self.state = 2
      #     ctrCmdMsg = Int16MultiArray()
      #     ctrCmdMsg.data = [2, self.goal_vol]
      #     self.ctr_cmd_pub.publish(ctrCmdMsg)
      #   else:
      #     # stop modee
      #     print("STOP")
      #     self.state = 3
      #     ctrCmdMsg = Int16MultiArray()
      #     ctrCmdMsg.data = [3, 0]
      #     self.ctr_cmd_pub.publish(ctrCmdMsg)
          
      #     self.feedback_flag = False
      #     self.set_mode(0)
      #   self.pre_sign = sign

      # if(self.feedback_flag == True):
      #   sign = self.calc_sign(self.goal_vol, self.cur_vol)
      #   if(self.pre_sign != sign):
      #     if(sign > 0):
      #       # move up
      #       print("MOVE_UP")
      #       self.state = 1
      #       ctrCmdMsg = Int16MultiArray()
      #       ctrCmdMsg.data = [1, self.goal_vol]
      #       self.ctr_cmd_pub.publish(ctrCmdMsg)
      #     elif(sign < 0):
      #       # move down
      #       print("MOVE_DOWN")
      #       self.state = 2
      #       ctrCmdMsg = Int16MultiArray()
      #       ctrCmdMsg.data = [2, self.goal_vol]
      #       self.ctr_cmd_pub.publish(ctrCmdMsg)
      #     else:
      #       # stop modee
      #       print("STOP")
      #       self.state = 3
      #       ctrCmdMsg = Int16MultiArray()
      #       ctrCmdMsg.data = [3, 0]
      #       self.ctr_cmd_pub.publish(ctrCmdMsg)
            
      #       self.feedback_flag = False
      #       self.set_mode(0)
      #   ctrCmdMsg = Int16MultiArray()
      #   ctrCmdMsg.data = [4, 0]
      #   self.ctr_cmd_pub.publish(ctrCmdMsg)
      #   self.pre_sign = sign
        
    elif self.mode == 3:
      print("ASPIRATING MODE")
      ctrCmdMsg = Int16MultiArray()
      ctrCmdMsg.data = [3, 0]
      self.ctr_cmd_pub.publish(ctrCmdMsg)
    elif self.mode == 4:
      print("DISPENSING MODE")
    elif self.mode == 5:
      print("PUMPING MODE")
    elif self.mode == 6:
      print("STOP MODE")

    stateMsg = Int8()
    stateMsg.data = self.state
    self.state_pub.publish(stateMsg)

  def calc_sign(self, goal_vol, cur_vol):
    res = goal_vol - cur_vol
    if(res > 0):
      return 1
    elif(res < 0):
      return -1
    else:
      return 0

  def calc_diff(self, goal_vol, cur_vol):
    res = goal_vol - cur_vol
    return res


  def set_mode(self, mode):
    self.mode = mode

  def move_to_origin(self):
    """!
    @brief control position of actuator
    @details 
    
    @param[in] data: ros Int8 message
    
    @note input constraints: 
    @n - none
    @note output constraints: 
    @n - none
    @return none
    """
    self.control_position(1,0)
    
  def move_to_present_volume(self):
    """!
    @brief control position of actuator
    @details 
    
    @param[in] data: ros Int8 message
    
    @note input constraints: 
    @n - none
    @note output constraints: 
    @n - none
    @return none
    """
    self.control_volume(1,self.valid_vol)

  def control_position(self, id, position):
    """!
    @brief control position of actuator
    @details 
    
    @param[in] data: ros Int8 message
    
    @note input constraints: 
    @n - none
    @note output constraints: 
    @n - none
    @return none
    """

    #self.set_operating_mode(id,3)

    dxl_cmd_req = DynamixelCommandRequest()
    dxl_cmd_req.id = id
    dxl_cmd_req.addr_name = "Profile_Acceleration"
    dxl_cmd_req.value = 30
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Profile_Velocity"
    dxl_cmd_req.value = 30
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Goal_Position"
    dxl_cmd_req.value = position
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    #print(self.cur_vol)

  def control_degree(self, id, degree):
    """!
    @brief control position of actuator
    @details 
    
    @param[in] data: ros Int8 message
    
    @note input constraints: 
    @n - none
    @note output constraints: 
    @n - none
    @return none
    """

    #self.set_operating_mode(id,3)

    position = self.degree_to_position(degree)
    dxl_cmd_req = DynamixelCommandRequest()
    dxl_cmd_req.id = id
    dxl_cmd_req.addr_name = "Profile_Acceleration"
    dxl_cmd_req.value = 30
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Profile_Velocity"
    dxl_cmd_req.value = 30
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Goal_Position"
    dxl_cmd_req.value = position
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    #print(self.cur_vol)

  def control_height(self, id, height):
    """!
    @brief control position of actuator
    @details 
    
    @param[in] data: ros Int8 message
    
    @note input constraints: 
    @n - none
    @note output constraints: 
    @n - none
    @return none
    """

    #self.set_operating_mode(id,3)

    degree = self.height_to_degree(height)
    position = self.degree_to_position(degree)
    dxl_cmd_req = DynamixelCommandRequest()
    dxl_cmd_req.id = id
    dxl_cmd_req.addr_name = "Profile_Acceleration"
    dxl_cmd_req.value = 30
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Profile_Velocity"
    dxl_cmd_req.value = 30
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Goal_Position"
    dxl_cmd_req.value = position
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    #print(self.cur_vol)

  def control_volume(self, id, volume):
    """!
    @brief control position of actuator
    @details 
    
    @param[in] data: ros Int8 message
    
    @note input constraints: 
    @n - none
    @note output constraints: 
    @n - none
    @return none
    """

    #self.set_operating_mode(id,3)

    height = self.volume_to_height(volume)
    degree = self.height_to_degree(height)
    position = self.degree_to_position(degree)
    dxl_cmd_req = DynamixelCommandRequest()
    dxl_cmd_req.id = id
    dxl_cmd_req.addr_name = "Profile_Acceleration"
    dxl_cmd_req.value = 30
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Profile_Velocity"
    dxl_cmd_req.value = 30
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Goal_Position"
    dxl_cmd_req.value = position
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    #print(self.cur_vol)

  def volume_up(self,acc=10,vel=-30):
    """!
    @brief control position of actuator
    @details 
    
    @param[in] data: ros Int8 message
    
    @note input constraints: 
    @n - none
    @note output constraints: 
    @n - none
    @return none
    """
    dxl_cmd_req = DynamixelCommandRequest()

    dxl_cmd_req.id = 2

    dxl_cmd_req.addr_name = "Profile_Acceleration"
    dxl_cmd_req.value = acc
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    # dxl_cmd_req.addr_name = "Profile_Velocity"
    # dxl_cmd_req.value = -30
    # result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Goal_Velocity"
    dxl_cmd_req.value = vel
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.id = 1

    #self.set_operating_mode(1,1)

    dxl_cmd_req.addr_name = "Profile_Acceleration"
    dxl_cmd_req.value = 1
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Profile_Velocity"
    dxl_cmd_req.value = 1
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    position = self.volume_to_position(0)
    dxl_cmd_req.addr_name = "Goal_Position"
    dxl_cmd_req.value = position
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)



  def volume_down(self,acc=10,vel=30):
    """!
    @brief control position of actuator
    @details 
    
    @param[in] data: ros Int8 message
    
    @note input constraints: 
    @n - none
    @note output constraints: 
    @n - none
    @return none
    """
    dxl_cmd_req = DynamixelCommandRequest()

    dxl_cmd_req.id = 2

    dxl_cmd_req.addr_name = "Profile_Acceleration"
    dxl_cmd_req.value = acc
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    # dxl_cmd_req.addr_name = "Profile_Velocity"
    # dxl_cmd_req.value = 40
    # result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Goal_Velocity"
    dxl_cmd_req.value = vel
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)

    dxl_cmd_req.id = 1

    #self.set_operating_mode(1,1)

    dxl_cmd_req.addr_name = "Profile_Acceleration"
    dxl_cmd_req.value = 1
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Profile_Velocity"
    dxl_cmd_req.value = 1
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    position = self.volume_to_position(0)
    dxl_cmd_req.addr_name = "Goal_Position"
    dxl_cmd_req.value = position
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)



  def stop(self):
    """!
    @brief control position of actuator
    @details 
    
    @param[in] data: ros Int8 message
    
    @note input constraints: 
    @n - none
    @note output constraints: 
    @n - none
    @return none
    """
    dxl_cmd_req = DynamixelCommandRequest()

    dxl_cmd_req.id = 1

    dxl_cmd_req.addr_name = "Profile_Acceleration"
    dxl_cmd_req.value = 1
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Profile_Velocity"
    dxl_cmd_req.value = 1
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Goal_Position"
    dxl_cmd_req.value = self.present_position
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    

    dxl_cmd_req.id = 2

    dxl_cmd_req.addr_name = "Profile_Acceleration"
    dxl_cmd_req.value = 0
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Profile_Velocity"
    dxl_cmd_req.value = 0
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Goal_Velocity"
    dxl_cmd_req.value = 0
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)

  def set_operating_mode(self, id, mode):
    """!
    @brief control position of actuator
    @details 
    
    @param[in] data: ros Int8 message
    
    @note input constraints: 
    @n - none
    @note output constraints: 
    @n - none
    @return none
    """
    dxl_cmd_req = DynamixelCommandRequest()

    dxl_cmd_req.id = id
    
    dxl_cmd_req.addr_name = "Torque_Enable"
    dxl_cmd_req.value = False
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    
    dxl_cmd_req.addr_name = "Operating_Mode"
    dxl_cmd_req.value = mode
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)

    dxl_cmd_req.addr_name = "Torque_Enable"
    dxl_cmd_req.value = True
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)

  def degree_to_position(self, degree):
    """!
    @brief convert actuator degree to position
    @details position = degree*(4,095/360)
    
    @param[in] data: ros Int8 message
    
    @note input constraints: 
    @n - none
    @note output constraints: 
    @n - none
    @return none
    """
    #print(degree*float(4095/360))
    position = degree*int(4095/360)
    return position

  def height_to_degree(self, height):
    """!
    @brief convert actuator degree to position
    @details position = degree*(4,095/360)
    
    @param[in] data: ros Int8 message
    
    @note input constraints: 
    @n - none
    @note output constraints: 
    @n - none
    @return none
    """
    # 25.0 : length of cam follower
    # 25mm is highst height
    # -25mm is lowest height
    # height = height-25.0;
    degree = int(180.0 - ((np.arcsin(height/25.0)*180.0)/np.pi+90.0))
    return degree

  def volume_to_height(self, volume):
    """!
    @brief convert actuator degree to position
    @details position = degree*(4,095/360)
    
    @param[in] data: ros Int8 message
    
    @note input constraints: 
    @n - none
    @note output constraints: 
    @n - none
    @return none
    """
    # 25.0 : length of cam follower
    # 25mm is highst height
    # -25mm is lowest height
    # height = height-25.0;
    height = int((0.1*float(volume))- 25.0)
    return height

  def volume_to_position(self, volume):
    """!
    @brief convert actuator degree to position
    @details position = degree*(4,095/360)
    
    @param[in] data: ros Int8 message
    
    @note input constraints: 
    @n - none
    @note output constraints: 
    @n - none
    @return none
    """
    # 25.0 : length of cam follower
    # 25mm is highst height
    # -25mm is lowest height
    # height = height-25.0;
    height = int((0.1*float(volume))- 25.0)
    degree = int(180.0 - ((np.arcsin(height/25.0)*180.0)/np.pi+90.0))
    position = degree*int(4095/360)
    return position

def main(args):
  rospy.init_node('modee_node', anonymous=True)
  modee = ModEE()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)