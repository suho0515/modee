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

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

    # Adding custom options
    self.custom_config = r'--oem 3 --psm 6'

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv_image = cv2.flip(cv_image,-1)
    except CvBridgeError as e:
      print(e)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

      x=280; y=90; w=40; h=160
      roi_img = cv_image[y:y+h, x:x+w]     
      #cv2.imshow('roi_img', roi_img)


      gray_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
      #cv2.imshow('gray_img', gray_img)

      #median_img = cv2.medianBlur(gray_img,3)
      #cv2.imshow('median_img', median_img)

      threshold_img = cv2.threshold(gray_img, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
      cv2.imshow('threshold_img', threshold_img)

      #kernel = np.ones((3,3),np.uint8)
      #morph_image = cv2.morphologyEx(threshold_img, cv2.MORPH_OPEN, kernel)
      #cv2.imshow('morph_image', morph_image)

      #canny edge detection
      #canny_image = cv2.Canny(threshold_img, 100, 200)
      #cv2.imshow('canny_image', canny_image)

      # d = pytesseract.image_to_data(threshold_img, output_type=Output.DICT)
      # print(d.keys())

      # date_pattern = '^[0-9]*$'

      # n_boxes = len(d['text'])
      # for i in range(n_boxes):
      #     if int(d['conf'][i]) > 60:
      #         (x, y, w, h) = (d['left'][i], d['top'][i], d['width'][i], d['height'][i])
      #         img = cv2.rectangle(roi_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
      #         cv2.imshow('img', img)

      cv2.waitKey(0)

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