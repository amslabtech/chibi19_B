#!/usr/bin/env python

import rospy
import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Polygon, Rectangle
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


#fourcc = cv2.VideoWriter_fourcc(*'MJPG')
#out = cv2.VideoWriter('/home/amsl/Desktop/output.avi',fourcc, 20.0, (640,480))

def draw_contours(ax, img, contours):
    ax.imshow(img)
    ax.axis('off')
    for i, cnt in enumerate(contours):
        cnt = np.squeeze(cnt, axis=1)  # (NumPoints, 1, 2) -> (NumPoints, 2)
        ax.add_patch(Polygon(cnt, color='b', fill=None, lw=2))
        ax.plot(cnt[:, 0], cnt[:, 1], 'ro', mew=0, ms=4)
        ax.text(cnt[0][0], cnt[0][1], i, color='orange', size='20')

def callback(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.encoding)
    try:
        img = CvBridge().imgmsg_to_cv2(data,"bgr8")
    except CvBridgeError as e:
        print(e)
    #cv2.imshow("original",img)


    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #cv2.imshow('gray', img_gray)

    img_blur = cv2.GaussianBlur(img_gray, (11, 11), 0)
    #cv2.imshow('blur', img_blur)

    # binarization
    ret, img_threshold = cv2.threshold(img_blur, 200, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    cv2.imshow( 'threshold', img_threshold)

    contours = cv2.findContours(img_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
    min_th_area = img.shape[0] * img.shape[1] / 100
    max_th_area = img.shape[0] * img.shape[1] / 4
    contours_lar = list(filter(lambda c:cv2.contourArea(c) > min_th_area, contours))
    contours_large = list(filter(lambda c:cv2.contourArea(c) < max_th_area, contours_lar))

    approx_contours = []
    for i, cnt in enumerate(contours_large):
        arclen = cv2.arcLength(cnt, True)
        approx_cnt = cv2.approxPolyDP(cnt, epsilon=0.005 * arclen, closed=True)
        approx_contours.append(approx_cnt)

    approx_contours_large = []
    for i in range(len(approx_contours)):
        if len(approx_contours[i]) < 10:
            approx_contours_large.append(approx_contours[i])

    try:
        for i, cnt in enumerate(approx_contours_large):
            for j in range(len(cnt)):
                x2, y2 = cnt[j][0]
                if j is 0:
                    num = len(cnt)-1
                    x1, y1 = cnt[num][0]
                    img_approx_contours_large = cv2.line(img, (x1,y1) , (x2,y2), (0,0,255), 3)
                else:
                    img_approx_contours_large = cv2.line(img, (x1,y1) , (x2,y2), (0,0,255), 3)
                x1, y1 = x2, y2
        cv2.imshow("approx_contours_large", img_approx_contours_large)
        cv2.imwrite('/home/amsl/Desktop/white_line.jpg', img_approx_contours_large)
    except:
        cv2.imshow("approx_contours_large", img)
        cv2.imwrite('/home/amsl/Desktop/white_line.jpg', img)
    #out.write(img_approx_contours_large)



    #img_canny = cv2.Canny(img_threshold, 50, 110) # edge detection
    #cv2.imshow( 'canny', img_canny)

    #lines = cv2.HoughLinesP(img_canny, rho=1, theta=np.pi/360, threshold=30, minLineLength=30, maxLineGap=80)
    #for line in lines:
    #    x1, y1, x2, y2 = line[0]
    #    img_red_line = cv2.line(img, (x1,y1), (x2,y2), (0,0,255), 3)
    #cv2.imshow("red_line", img_red_line)

    cv2.imwrite('/home/amsl/Desktop/threshold.jpg', img_threshold)

    if cv2.waitKey(3) == 'q':
        out.release()

def listener():
    rospy.init_node('opencv_subscribe', anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()


