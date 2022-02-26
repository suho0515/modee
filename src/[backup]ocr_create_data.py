#!/usr/bin/env python3
from __future__ import print_function

#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import pytesseract
from pytesseract import Output

class image_converter:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv_image = cv2.flip(cv_image,-1)
      cv2.imshow("cv_image", cv_image)
    except CvBridgeError as e:
      print(e)


    try:
      x=290; y=100; w=50; h=170
      roi_img = cv_image[y:y+h, x:x+w]     
      cv2.imshow('roi_img', roi_img)

      # 
      hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)

      rng_1 = cv2.inRange(hsv, (0, 100, 0), (255, 255, 200))
      rng_2 = cv2.inRange(hsv, (0, 0, 0), (255, 255, 100))

      cv2.imshow('rng_1', rng_1)
      cv2.imshow('rng_2', rng_2)

      sum = rng_1 + rng_2
      cv2.imshow('sum', sum)

      # https://stackoverflow.com/questions/9413216/simple-digit-recognition-ocr-in-opencv-python
      # Pre-processing
      #gray = cv2.cvtColor(roi_img,cv2.COLOR_BGR2GRAY)
      #cv2.imshow('gray', gray)
      #blur = cv2.GaussianBlur(gray,(5,5),0)
      #cv2.imshow('blur', blur)
      thresh = cv2.adaptiveThreshold(sum,255,1,1,3,2)
      cv2.imshow('thresh', thresh)

      contours,hierarchy = cv2.findContours(sum,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

      samples =  np.empty((0,100))
      responses = []
      keys = [i for i in range(48,58)]

      for cnt in contours:
        if cv2.contourArea(cnt)>100:
            [x,y,w,h] = cv2.boundingRect(cnt)

            if  h>20:
                #print(cv2.contourArea(cnt))
                #print(w)
                #print(h)
                #print("\n")
                cv2.rectangle(roi_img,(x,y),(x+w,y+h),(0,0,255),2)
                roi = sum[y:y+h,x:x+w]
                roismall = cv2.resize(roi,(10,10))
                #cv2.imshow('norm',roi_img)
                cv2.imshow('roi',roi)
                cv2.imshow('roismall',roismall)
                key = cv2.waitKey(0)

                if key == 27:  # (escape to quit)
                    sys.exit()
                elif key in keys:
                    responses.append(int(chr(key)))
                    sample = roismall.reshape((1,100))
                    samples = np.append(samples,sample,0)

      cv2.imshow('final_image',roi_img)

      responses = np.array(responses,np.float32)
      responses = responses.reshape((responses.size,1))
      print("training complete")

      np.savetxt('generalsamples.data',samples)
      np.savetxt('generalresponses.data',responses)

      cv2.waitKey(3)

    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('ocr_create_data', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)