import numpy as np
import imutils
import cv2

capture = cv2.VideoCapture(0)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while cv2.waitKey(33) < 0:
    ret, frame = capture.read()
    #cv2.imshow("VideoFrame", frame)

    cv_image = cv2.flip(frame,-1)
    #cv2.imshow("cv_image", cv_image)

    x=290; y=100; w=50; h=160
    roi_img = cv_image[y:y+h, x:x+w]     
    #cv2.imshow('roi_img', roi_img)

    # 
    hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)

    rng_1 = cv2.inRange(hsv, (0, 100, 0), (255, 255, 200))
    rng_2 = cv2.inRange(hsv, (0, 0, 0), (255, 255, 100))

    #cv2.imshow('rng_1', rng_1)
    #cv2.imshow('rng_2', rng_2)

    sum = rng_1 + rng_2
    cv2.imshow('sum', sum)

    # Removing noise
    # https://pyimagesearch.com/2015/02/09/removing-contours-image-using-python-opencv/

    contours = cv2.findContours(sum,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)


capture.release()
cv2.destroyAllWindows()