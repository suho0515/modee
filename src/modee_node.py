#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Int16
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

    # Publisher
    ## it publish state of modee
    self.state_pub = rospy.Publisher("modee_state", String, queue_size=1)    

    # Services
    ## command service which control dynamixels
    # Wait for the service client /dynamixel_workbench/dynamixel_command to be running
    dxl_command_service_name = "/dynamixel_workbench/dynamixel_command"
    rospy.wait_for_service(dxl_command_service_name)
    # Create the connection to the service
    self.dxl_command_service = rospy.ServiceProxy(dxl_command_service_name, DynamixelCommand)


    
    self.ctr_cmd_pub = rospy.Publisher("controller_command", Int16MultiArray, queue_size=10)
    #self.state_pub = rospy.Publisher("controller_state", Int8, queue_size=10)

    # Timer
    ## we would use ros timer for processing tasks.
    self.timer = rospy.Timer(rospy.Duration(0.001), self.timerCallback)

    # Variables
    ## valid volume of micropipette
    self.valid_vol = 0
    ## dynamixel state list
    self.dxl_state_list = []
    ## raw degree value of vertical motion unit actuator
    self.present_position = 0
    ## raw current value of vertical motion unit actuator
    self.present_current = 0
    ## which cmmand modee will do
    self.cmd = 0
    ## goal volume for feedback control
    self.goal_vol = -1
    ## process steps for modee control
    self.process = 0
    ## process flage for modee control
    self.flag = False
    ## process count for modee control
    self.cnt = 0
    




    ## feedback state
    self.feedback_state = 0
    self.feedback_flag = False
    self.goal_position = 0
    ## difference of center percentage
    self.docp = 0
    ## count
    self.cnt = 0
    ## count for detailed control
    self.ctr_cnt = 0
    ## aspirating state
    self.aspirating_state = 0
    ## aspirating flag
    self.aspirating_flag = False
    ## aspirating count
    self.aspirating_cnt = 0
    
    ## pumping flag
    self.pumping_flag = False
    ## pumping count
    self.pumping_cnt = 0


    self.pre_sign = 0

    self.state = 0

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
    self.move_to_origin()
    #self.control_height(1, 25)

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
    @n - the volume should be ranged 0 to 100
    """
    try:
      # input error check
      if(data.data < 0 or data.data > 100):
        raise VolumeError(data.data)

      # do
      self.valid_vol = data.data

    except Exception as e:
      print(e)
    pass
    
  def centerDiffPercentageCallback(self, data):
    """!
    @brief subscribe center different percentage
    @details 
    
    @param[in] data: ros Int8 message
    
    @note input constraints: 
    @n - the percentage should be ranged from -50 to 50
    """
    try:
      # input error check
      if(data.data < -60 or data.data > 60):
        raise CenterDiffPercentageError(data.data)

      # do
      self.docp = data.data

    except Exception as e:
      print(e)
    pass

  def dxlStateCallback(self,data):
    """!
    @brief subscribe dynamixel state
    @details in this function we get information of present position and present current of vertical motion unit
    
    @param[in] data: ros DynamixelStateList message
    
    @note input constraints: 
    @n - none
    """
    try:
      self.dxl_state_list = data
      self.present_position = self.dxl_state_list.dynamixel_state[1].present_position
      self.present_current = self.dxl_state_list.dynamixel_state[1].present_current
    except Exception as e:
      print(e)
    pass

  def cmdCallback(self,data):
    """!
    @brief subscribe command which modee will do
    @details 
    
    @param[in] data: ros Int16MultiArray message
    @n - data[0] is command
    @n  command is listed like below
    @n  0: Waiting, modee will wait for command
    @n  1: Feedback Control, modee will do feedback control with this command
    @n  2: Aspirating, modee will do aspirating with this command
    @n  3: Dispensing, modee will do dispensing with this command
    @n  4: Pumping, modee will do pumping with this command
    @n  5: stop, modee will stop both actuators(vertical motion unit actuator and rotary motion unit actuator) with this command
    
    @n - data[1] is goal volume for feedback control
    
    @note input constraints: 
    @n - command should be ranged from 0 to 5
    @n - goal volume should be ranged from 0 to 100
    """
    try:
      # input error check
      if(data.data[0] < 0 or data.data[0] > 5):
        raise CommandError(data.data[0])
      if(data.data[1] < 0 or data.data[1] > 100):
        raise VolumeError(data.data[1])

      # do
      self.cmd = data.data[0]
      self.goal_vol = data.data[1]

    except Exception as e:
      print(e)
    pass

  def timerCallback(self, *args):
    """!
    @brief this timer callback is processed every 0.001 seconds
    @details
    """

    state = self.process_command(self.cmd, self.goal_vol)
    
    stateMsg = String()
    stateMsg.data = state
    self.state_pub.publish(stateMsg)









    
    # if self.cmd == 0:
    #   print("WATING COMMAND")
    #   if(self.state == 0):
    #     #self.set_operating_mode(1,5)
    #     self.state = 1
    #   elif(self.state == 1):
    #     self.state = 2
    #     #print("state 2")
    #     #self.maintain_volume_with_rotation(10, 1500, 10,10,10, 200)
    #     #self.maintain_volume_with_rotation(50, 0, 10,10,10, -200)
    #     #self.control_current_based_position(1, 100, 0, 60, -60)

    # elif self.cmd == 1:
    #   print("FEEDBACK CONTROL")
      
    # elif self.cmd == 2:
    #   print("ASPIRATING")

    #   if(self.feedback_state == 0):
        
    #     if(self.feedback_flag == False):
    #       self.feedback_flag = True
    #       self.set_operating_mode(1,5)

    #       self.move_to_present_volume()
    #       self.valid_position = self.volume_to_position(self.valid_vol)
    #       self.goal_position = self.volume_to_position(self.goal_vol)
          
    #     print(self.valid_position)
    #     print(self.present_position)

    #     offset = 100
    #     if((self.present_position < self.valid_position + offset) and (self.present_position > self.valid_position - offset) ):
    #       self.feedback_state = 1
    #       self.feedback_flag = False
    #       self.stop()
    #       print("state 1 is complete")

    #   elif(self.feedback_state == 1):
        
    #     # if(self.feedback_flag == False):
    #     #   self.feedback_flag = True

    #     #sign = self.calc_sign(self.goal_vol, self.valid_vol)
    #     diff = self.calc_diff(self.goal_vol, self.valid_vol)
    #     print(diff)
    #     if(diff > 0):
    #       # move up
    #       print("MOVE_UP")
    #       if(abs(diff) < 3):
    #         self.volume_up(10, -20)
    #       else:
    #         self.volume_up(30, -60)
    #     elif(diff < 0):
    #       # move down
    #       print("MOVE_DOWN")
    #       if(abs(diff) < 3):
    #         self.volume_down(10, 20)
    #       else:
    #         self.volume_down(30, 60)
    #     else:
    #       # stop modee
    #       print("STOP")
    #       self.stop()
    #       #self.move_to_origin()
    #       self.feedback_state = 2
    #       #self.cmd = 0

    #   elif(self.feedback_state == 2):
    #     if(self.center_diff_percentage > 10):
    #       self.volume_down(5, 10)
    #       self.stop()
    #       self.ctr_cnt = 0
    #     elif(self.center_diff_percentage < -10):
    #       self.volume_up(5, -10)
    #       self.stop()
    #       self.ctr_cnt = 0
    #     else:
    #       if(self.ctr_cnt > 1000):
    #         # stop modee
    #         print("STOP")
    #         self.stop()
    #         self.move_to_origin()
    #         self.feedback_state = 0
    #         self.cmd = 0
    #         self.ctr_cnt = 0
    #       self.ctr_cnt = self.ctr_cnt + 1

    # elif self.cmd == 3:
    #   #print("ASPIRATING MODE")

      
    #   if(self.aspirating_state == 0):
    #     self.set_operating_mode(1,5)
    #     self.aspirating_state = 1
    #   elif(self.aspirating_state == 1):
    #     offset = 3
    #     goal_cur = 150
    #     if(self.aspirating_flag == False):
          
    #       self.control_current_based_position(1, goal_cur, 1500, 60, 60)
    #       self.aspirating_flag = True
          

    #     if((self.present_current < goal_cur + offset) and (self.present_current > goal_cur - offset)):
    #       self.aspirating_flag = False
    #       self.aspirating_state = 2
      
    #   elif(self.aspirating_state == 2):
    #     if(self.aspirating_cnt > 3000):
    #       self.aspirating_cnt = 0
    #       self.control_current_based_position(1, 1000, 0, 60, 60)
    #       self.cmd = 0
    #       self.aspirating_state = 0
    #     else:
    #       self.aspirating_cnt = self.aspirating_cnt + 1


    #     print(self.present_current, self.present_position)
          
    #     #self.aspirating_state = 1
      
    # elif self.cmd == 4:
    #   print("DISPENSING MODE")
    #   if(self.aspirating_state == 0):
    #     self.set_operating_mode(1,5)
    #     self.aspirating_state = 1
    #   elif(self.aspirating_state == 1):
    #     offset = 3
    #     goal_cur = 360
    #     if(self.aspirating_flag == False):
          
    #       self.control_current_based_position(1, goal_cur, 1900, 60, 60)
    #       self.aspirating_flag = True
          

    #     if((self.present_current < goal_cur + offset) and (self.present_current > goal_cur - offset)):
    #       self.aspirating_flag = False
    #       self.aspirating_state = 2
      
    #   elif(self.aspirating_state == 2):
    #     if(self.aspirating_cnt > 3000):
    #       self.aspirating_cnt = 0
    #       self.control_current_based_position(1, 1000, 0, 60, 60)
    #       self.cmd = 0
    #       self.aspirating_state = 0
    #     else:
    #       self.aspirating_cnt = self.aspirating_cnt + 1


    #     print(self.present_current, self.present_position)
    # elif self.cmd == 5:
    #   print("PUMPING MODE")
    #   if(self.aspirating_state == 0):
    #     self.set_operating_mode(1,5)
    #     self.control_current_based_position(1, 360, 1900, 10, 10)
    #     self.aspirating_state = 1
    #   elif(self.aspirating_state == 1):
        
    #     if(self.pumping_flag == False):
    #       goal_cur = 360
    #       offset = 50
    #       if(self.aspirating_flag == False):
    #         self.control_current_based_position(1, goal_cur, 1900, 300, 300)
    #         self.aspirating_flag = True
    #       if((self.present_current < goal_cur + offset) and (self.present_current > goal_cur - offset)):
    #         self.aspirating_flag = False
    #         #self.aspirating_state = 2
    #         self.pumping_flag = True
    #         self.pumping_cnt = self.pumping_cnt + 1

    #     else:
    #       goal_pos = 1300
    #       offset = 100
    #       if(self.aspirating_flag == False):
    #         self.control_current_based_position(1, 1000, goal_pos, 300, 300)
    #         self.aspirating_flag = True
    #       if((self.present_position < goal_pos + offset) and (self.present_position > goal_pos - offset)):
    #         self.aspirating_flag = False
    #         #self.aspirating_state = 2
    #         self.pumping_flag = False
    #         self.pumping_cnt = self.pumping_cnt + 1

    #       if(self.pumping_cnt > 20):
    #         self.pumping_cnt = 0
    #         self.control_current_based_position(1, 1000, 0, 60, 60)
    #         self.cmd = 0
    #         self.aspirating_state = 0

          
          
      
    #   elif(self.aspirating_state == 2):
    #     if(self.aspirating_cnt > 3000):
    #       self.aspirating_cnt = 0
    #       self.control_current_based_position(1, 1000, 0, 60, 60)
    #       self.cmd = 0
    #       self.aspirating_state = 0
    #     else:
    #       self.aspirating_cnt = self.aspirating_cnt + 1


    #     print(self.present_current, self.present_position)
    # elif self.cmd == 6:
    #   print("STOP MODE")
    #   self.set_operating_mode(1,1)
    #   self.stop()
    #   #self.aspirating_state = 0

    # # stateMsg = Int8()
    # # stateMsg.data = self.state
    # # self.state_pub.publish(stateMsg)
    


  def process_command(self, cmd, goal_vol):
    """!
    @brief process command
    @details
    @n  command is listed like below
    @n  0: Waiting, modee will wait for command
    @n  1: Feedback Control, modee will do feedback control with this command
    @n  2: Aspirating, modee will do aspirating with this command
    @n  3: Dispensing, modee will do dispensing with this command
    @n  4: Pumping, modee will do pumping with this command
    @n  5: stop, modee will stop both actuators(vertical motion unit actuator and rotary motion unit actuator) with this command

    @param[in] cmd: command which modee will do
    @param[in] goal_vol: goal volume for feedback control
    
    @note input constraints: 
    @n - command should be ranged from 0 to 5
    @n - goal volume should be ranged from 0 to 100
    """
    try:
      # input error check
      if(cmd < 0 or cmd > 5):
        raise CommandError(cmd)
      if(goal_vol != -1):
        if(goal_vol < 0 or goal_vol > 100):
          raise VolumeError(goal_vol)

      # do
      result = False

      if self.cmd == 0:
        return "WATING"

      elif self.cmd == 1:
        self.feedback_control_process(self.goal_vol, self.valid_vol)
        return "FEEDBACK_CONTROL"
        
      elif self.cmd == 2:
        self.aspirating()
        return "ASPIRATING"

      elif self.cmd == 3:
        self.dispensing()
        return "DISPENSING"

      elif self.cmd == 4:
        self.pumping()
        return "PUMPING"

      elif self.cmd == 5:

        return "STOP"

    except Exception as e:
      print(e)
    pass

  def feedback_control_process(self, goal_vol, valid_vol):
    """!
    @brief feedback control to adjust micropipette volume
    @details feedback control would be processed like bellow
    @n - process 0: move to present volume
    @n - process 1: feedback control with difference of goal volume and present volume.
    @n if the difference is bigger than 10, it will rotated with XX acceleration and XX velocity
    @n if the difference is bigger than 3, it will rotated with XX acceleration and XX velocity
    @n if the difference is less than 3, it will rotated with XX acceleration and XX velocity
    @n - process 2: feedback control to adjust differnce of center percentage less than 10 percent
    
    @param[in] goal_vol: goal volume
    
    @note input constraints: 
    @n - goal volume should be ranged from 0 to 100

    @return result: result of the process
    """
    try:
      # input error check
      if(goal_vol < 0 or goal_vol > 100):
        raise VolumeError(goal_vol)

      # do
      if(self.process == 0):
          result = self.move_to_origin()
          if(result):
            self.process = 1
          print("process 0", result)
      elif(self.process == 1):
        result = self.move_to_present_volume()
        if(result):
          self.process = 2
        print("process 1", result)
      elif(self.process == 2):
        result = self.feedback_control_with_volume(goal_vol, valid_vol)
        if(result):
          self.process = 3
          #self.cnt = 0
        print("process 2", result)
      elif(self.process == 3):
        result = self.feedback_control_with_docp(self.docp)
        if(result):
          self.process = 4
        print("process 3", result)
      elif(self.process == 4):
        #result = self.stop()
        self.stop()
        #if(result):
        self.process = 5
        print("process 4", result)
      elif(self.process == 5):
        result = self.move_to_origin()
        if(result):
          self.process = 0
          self.cmd = 0
          self.cnt = 0
          self.flag = False
        print("process 5", result)
        

      return True

    except Exception as e:
      print(e)
      return False
    pass

  def feedback_control_with_volume(self, goal_vol, valid_vol):
    diff = self.calc_diff(goal_vol, valid_vol)
    print(diff)
    print(self.cnt)
    if(diff > 0):
      # move up
      print("MOVE_UP")
      if(abs(diff) < 3):
        self.volume_up(5, -10)
      elif(abs(diff) < 20):
        self.volume_up(30, -60)
      else:
        self.volume_up(60, -120)
      return False
    elif(diff < 0):
      # move down
      print("MOVE_DOWN")
      if(abs(diff) < 3):
        self.volume_down(5, 10)
      elif(abs(diff) < 20):
        self.volume_up(30, 60)
      else:
        self.volume_down(60, 120)
      return False
    else:
      self.stop()
      if(self.cnt > 5):
        # stop modee
        print("STOP")
        self.stop()
        self.cnt = 0
        return True
      else:
        self.cnt = self.cnt + 1
        return False
    
    
  def feedback_control_with_docp(self, docp):
    if(docp > 10):
      self.volume_down(5, 10)
      self.stop()
      return False
    elif(docp < -10):
      self.volume_up(5, -10)
      self.stop()
      return False
    else:
      self.stop()
      if(self.cnt > 5):
        # stop modee
        print("STOP")
        self.stop()
        self.cnt = 0
        return True
      else:
        self.cnt = self.cnt + 1
        return False

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
    self.cmd = mode

  def move_to_origin(self):
    """!
    @brief this function move pipette contact to origin position
    @details

    @return result
    @n - if the position is 0, return true.
    @n - if the position is not 0, return false.
    """
    id_2 = 2
    acc_2 = 0
    vel_2 = 0
    self.control_rotation(id_2, acc_2, vel_2)
    
    id_1 = 1
    cur_1 = 180
    #cur_1 = (100 - self.valid_vol) + 100
    pos_1 = 500
    acc_1 = 1
    vel_1 = 20
    self.control_current_based_position(id_1, cur_1, pos_1, acc_1, vel_1)

    offset = 20

    if((self.present_position > (pos_1 - offset)) and (self.present_position < (pos_1 + offset))):
      return True
    else:
      return False
    
  def move_to_present_volume(self):
    """!
    @brief this function move pipette contact to present volume
    @details

    @return result
    @n - if the current is bigger than 10, return true.
    @n - if the position is not bigger than 10, return false.
    """
    self.control_volume(1,self.valid_vol)

    id_1 = 1
    cur_1 = (100 - self.valid_vol) + 0
    #if(cur_1 > 100):
    #  cur_1 = 100
    #print(cur_1)
    cur_1 = 10
    pos_1 = 1500
    acc_1 = 10
    vel_1 = 20
    self.control_current_based_position(id_1, cur_1, pos_1, acc_1, vel_1)

    offset = 5
    if(self.present_current > cur_1 - offset and self.present_current < cur_1 + offset):
      return True
    else:
      return False

  def aspirating(self):
    if(self.process == 0):
      offset = 3
      goal_cur = 150
      if(self.flag == False):
        self.control_current_based_position(1, goal_cur, 1500, 60, 60)
        self.flag = True
          
      if((self.present_current < goal_cur + offset) and (self.present_current > goal_cur - offset)):
        self.flag = False
        self.process = 1
        self.stop()
      
    elif(self.process == 1):
      if(self.cnt > 2000):
        self.control_current_based_position(1, 1000, 0, 60, 60)
        self.cmd = 0
        self.cnt = 0
        self.process = 0
        self.flag = False
      else:
        self.cnt = self.cnt + 1

  def dispensing(self):
    if(self.process == 0):
      offset = 3
      goal_cur = 360
      if(self.flag == False):
        self.control_current_based_position(1, goal_cur, 1900, 60, 60)
        self.flag = True

      if((self.present_current < goal_cur + offset) and (self.present_current > goal_cur - offset)):
        self.flag = False
        self.process = 1
    
    elif(self.process == 1):
      if(self.cnt > 2000):
        self.control_current_based_position(1, 1000, 0, 60, 60)
        self.cmd = 0
        self.cnt = 0
        self.process = 0
        self.flag = False
      else:
        self.cnt = self.cnt + 1

  def pumping(self):
    if(self.process == 0):
      self.stop()
      self.control_current_based_position(1, 150, 1300, 1, 1)
      #self.stop()
      #rospy.sleep(3)
      self.process = 1
    elif(self.process == 1):
      if(self.pumping_flag == False):
        goal_cur = 360
        offset = 50
        if(self.flag == False):
          self.control_current_based_position(1, goal_cur, 1900, 100, 200)
          self.flag = True
        if((self.present_current < goal_cur + offset) and (self.present_current > goal_cur - offset)):
          self.flag = False
          #self.aspirating_state = 2
          self.pumping_flag = True
          self.cnt = self.cnt + 1

      else:
        goal_pos = 1300
        offset = 100
        if(self.flag == False):
          self.control_current_based_position(1, 1000, goal_pos, 100, 200)
          self.flag = True
        if((self.present_position < goal_pos + offset) and (self.present_position > goal_pos - offset)):
          self.flag = False
          self.pumping_flag = False
          self.cnt = self.cnt + 1

        if(self.cnt > 20):
          self.control_current_based_position(1, 1000, 0, 60, 60)
          self.cmd = 0
          self.cnt = 0
          self.process = 0
          self.flag = False
    
    # elif(self.process == 2):
    #   if(self.cnt > 3000):
    #     self.control_current_based_position(1, 1000, 0, 60, 60)
    #     self.cmd = 0
    #     self.cnt = 0
    #     self.process = 0
    #     self.flag = False
    #   else:
    #     self.cnt = self.cnt + 1

  def control_current(self, id, current):
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
    # current: 0~1,193
    dxl_cmd_req.addr_name = "Goal_Current"
    dxl_cmd_req.value = current
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    #print(self.cur_vol)

  def control_current_based_position(self, id, current, position, acc, vel):
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
    dxl_cmd_req.value = acc
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Profile_Velocity"
    dxl_cmd_req.value = vel
    result = self.dxl_command_service(dxl_cmd_req)
    dxl_cmd_req.addr_name = "Goal_Position"
    dxl_cmd_req.value = position
    result = self.dxl_command_service(dxl_cmd_req)
    #rospy.sleep(1)
    #print(result)
    # current: 0~1,193
    dxl_cmd_req.addr_name = "Goal_Current"
    dxl_cmd_req.value = current
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    
    

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

  def control_rotation(self,id=2, acc=10,vel=10):
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

    dxl_cmd_req.addr_name = "Goal_Velocity"
    dxl_cmd_req.value = vel
    result = self.dxl_command_service(dxl_cmd_req)


  def volume_up(self,acc_2=10,vel_2=-30):
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
    dxl_cmd_req.value = acc_2
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    # dxl_cmd_req.addr_name = "Profile_Velocity"
    # dxl_cmd_req.value = -30
    # result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Goal_Velocity"
    dxl_cmd_req.value = vel_2
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)

    
    # dxl_cmd_req.id = 1

    # #self.set_operating_mode(1,1)

    # self.valid_vol

    # dxl_cmd_req.addr_name = "Profile_Acceleration"
    # dxl_cmd_req.value = 1
    # result = self.dxl_command_service(dxl_cmd_req)
    # #print(result)
    # dxl_cmd_req.addr_name = "Profile_Velocity"
    # dxl_cmd_req.value = 1
    # result = self.dxl_command_service(dxl_cmd_req)
    # #print(result)
    # position = self.volume_to_position(0)
    # dxl_cmd_req.addr_name = "Goal_Position"
    # dxl_cmd_req.value = position
    # result = self.dxl_command_service(dxl_cmd_req)
    # #print(result)

    goal_cur = 50
    self.control_current_based_position(1, goal_cur, 1500, 10, 10)



  def volume_down(self,acc_2=10,vel_2=30):
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
    dxl_cmd_req.value = acc_2
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    # dxl_cmd_req.addr_name = "Profile_Velocity"
    # dxl_cmd_req.value = 40
    # result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Goal_Velocity"
    dxl_cmd_req.value = vel_2
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)

    # dxl_cmd_req.id = 1

    # #self.set_operating_mode(1,1)

    # dxl_cmd_req.addr_name = "Profile_Acceleration"
    # dxl_cmd_req.value = 1
    # result = self.dxl_command_service(dxl_cmd_req)
    # #print(result)
    # dxl_cmd_req.addr_name = "Profile_Velocity"
    # dxl_cmd_req.value = 1
    # result = self.dxl_command_service(dxl_cmd_req)
    # #print(result)
    # position = self.volume_to_position(0)
    # dxl_cmd_req.addr_name = "Goal_Position"
    # dxl_cmd_req.value = position
    # result = self.dxl_command_service(dxl_cmd_req)
    # #print(result)
    goal_cur = 10
    self.control_current_based_position(1, goal_cur, 1500, 10, 10)

  def maintain_volume_with_rotation(self,cur_1, pos_1, acc_1=10,vel_1=10, acc_2=10,vel_2=10):
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
    if((self.valid_vol < 20) and (vel_2 > 0)):
      acc_2 = 0
      vel_2 = 0
      print(self.valid_vol)
      print(self.valid_vol == 0)
      print("vel_2 > 0")

    if((self.valid_vol > 80) and (vel_2 < 0)):
      acc_2 = 0
      vel_2 = 0
      print("vel_2 < 0")

    dxl_cmd_req = DynamixelCommandRequest()

    dxl_cmd_req.id = 2

    dxl_cmd_req.addr_name = "Profile_Acceleration"
    dxl_cmd_req.value = acc_2
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Goal_Velocity"
    dxl_cmd_req.value = vel_2
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)

    #goal_cur = (100 - self.valid_vol) + 1
    #print(goal_cur)
    #goal_cur = 100
    goal_cur = 10
    self.control_current_based_position(1, cur_1, pos_1, acc_1, vel_1)



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
    dxl_cmd_req.value = 0
    result = self.dxl_command_service(dxl_cmd_req)
    #print(result)
    dxl_cmd_req.addr_name = "Profile_Velocity"
    dxl_cmd_req.value = 0
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

  def control_current_based_position_test(self):
    """!
    @brief test to identify apropriate current value to control micropipette
    @details it will change position from 0 to 2047 every seconds (or 5 seconds)
    """

    for i in range(0,1800,100):
      print(i)
      self.control_current_based_position(1, 100, i, 60, 60)
      rospy.sleep(1)

    for i in range(1800,0,-100):
      print(i)
      self.control_current_based_position(1, 100, i, 60, 60)
      rospy.sleep(1)

  def control_current_based_position_with_rotation_test(self):
    """!
    @brief test to identify apropriate current value to control micropipette
    @details it will change position from 0 to 2047 every seconds (or 5 seconds)
    """

    for i in range(0,1800,100):
      print(i)
      self.control_rotation(10,30)
      self.control_current_based_position(1, 100, i, 60, 60)
      rospy.sleep(1)

    for i in range(1800,0,-100):
      print(i)
      self.control_rotation(10,-30)
      self.control_current_based_position(1, 100, i, 60, 60)
      rospy.sleep(1)

  def ocr_rotation_speed_test(self):
    """!
    @brief test to identify apropriate speed of rotary motion unit actuator 
    @details it will change position from 0 to 2047 every seconds (or 5 seconds)
    """
    
    #self.move_to_origin()
    #while(True):
    self.maintain_volume_with_rotation(10,10,10, 60)
    

    

class VolumeError(Exception):
  def __init__(self, vol):
    self.msg = "range of volum should be 0~100. but the volum is " + str(vol) + "."
  def __str__(self):
    return self.msg    

class CenterDiffPercentageError(Exception):
  def __init__(self, cdp):
    self.msg = "range of Center Different Percentage should be -50~50. but the CDP is " + str(cdp) + "."
  def __str__(self):
    return self.msg    

class CommandError(Exception):
  def __init__(self, cmd):
    self.msg = "range of command should be ranged from 0 to 5. but the command is " + str(cmd) + "."
  def __str__(self):
    return self.msg    

def main(args):
  rospy.init_node('modee_node', anonymous=True)
  modee = ModEE()
  try:
    #modee.control_current_based_position_test()
    #modee.control_current_based_position_with_rotation_test()
    rospy.spin()
    
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)