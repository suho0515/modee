#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Int16MultiArray

class ModEE:

  def __init__(self):
    # subscribe volume
    self.vol_sub = rospy.Subscriber("volume",Int8,self.volCallback)
    self.cmd_sub = rospy.Subscriber("command",Int16MultiArray,self.cmdCallback)
    self.timer = rospy.Timer(rospy.Duration(0.1), self.timerCallback)
    self.ctr_cmd_pub = rospy.Publisher("controller_command", Int16MultiArray, queue_size=10)
    self.state_pub = rospy.Publisher("controller_state", Int8, queue_size=10)

    self.cur_vol = 0
    
    self.mode = 0
    self.vel_1 = 0
    self.acc_1 = 0
    self.vel_2 = 0
    self.acc_2 = 0
    self.goal_vol = 0

    self.feedback_flag = False
    self.pre_sign = 0

    self.state = -1

  def volCallback(self,data):
    #print(data.data)
    self.cur_vol = data.data
    

  def cmdCallback(self,data):
    #print(data.data)
    self.mode = data.data[0]
    self.vel_1 = data.data[1]
    self.acc_1 = data.data[2]
    self.vel_2 = data.data[3]
    self.acc_2 = data.data[4]
    self.goal_vol = data.data[5]

  def timerCallback(self, *args):
    #print("Test")
    
    if self.mode == 0:
      print("NOTHING MODE")
      ctrCmdMsg = Int16MultiArray()
      ctrCmdMsg.data = [0, 0]
      self.ctr_cmd_pub.publish(ctrCmdMsg)
    elif self.mode == 1:
      print("MANUAL MODE")
    elif self.mode == 2:
      print("FEEDBACK MODE")

      if(self.feedback_flag == False):
        self.feedback_flag = True
        sign = self.calc_sign(self.goal_vol, self.cur_vol)
        if(sign > 0):
          # move up
          print("MOVE_UP")
          self.state = 1
          ctrCmdMsg = Int16MultiArray()
          ctrCmdMsg.data = [1, self.goal_vol]
          self.ctr_cmd_pub.publish(ctrCmdMsg)
        elif(sign < 0):
          # move down
          print("MOVE_DOWN")
          self.state = 2
          ctrCmdMsg = Int16MultiArray()
          ctrCmdMsg.data = [2, self.goal_vol]
          self.ctr_cmd_pub.publish(ctrCmdMsg)
        else:
          # stop modee
          print("STOP")
          self.state = 3
          ctrCmdMsg = Int16MultiArray()
          ctrCmdMsg.data = [3, 0]
          self.ctr_cmd_pub.publish(ctrCmdMsg)
          
          self.feedback_flag = False
          self.set_mode(0)
        self.pre_sign = sign

      if(self.feedback_flag == True):
        sign = self.calc_sign(self.goal_vol, self.cur_vol)
        if(self.pre_sign != sign):
          if(sign > 0):
            # move up
            print("MOVE_UP")
            self.state = 1
            ctrCmdMsg = Int16MultiArray()
            ctrCmdMsg.data = [1, self.goal_vol]
            self.ctr_cmd_pub.publish(ctrCmdMsg)
          elif(sign < 0):
            # move down
            print("MOVE_DOWN")
            self.state = 2
            ctrCmdMsg = Int16MultiArray()
            ctrCmdMsg.data = [2, self.goal_vol]
            self.ctr_cmd_pub.publish(ctrCmdMsg)
          else:
            # stop modee
            print("STOP")
            self.state = 3
            ctrCmdMsg = Int16MultiArray()
            ctrCmdMsg.data = [3, 0]
            self.ctr_cmd_pub.publish(ctrCmdMsg)
            
            self.feedback_flag = False
            self.set_mode(0)
        ctrCmdMsg = Int16MultiArray()
        ctrCmdMsg.data = [4, 0]
        self.ctr_cmd_pub.publish(ctrCmdMsg)
        self.pre_sign = sign
        
    elif self.mode == 3:
      print("ASPIRATING MODE")
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

  def set_mode(self, mode):
    self.mode = mode


    

def main(args):
  rospy.init_node('modee_node', anonymous=True)
  modee = ModEE()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)