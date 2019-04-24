#!/usr/bin/env python

import rospy
import cv2
import time
import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Circle, Polygon, Rectangle
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

res = False

fourcc_thres = cv2.VideoWriter_fourcc('F','L','V','1')
fourcc_final = cv2.VideoWriter_fourcc(*'MJPG')
#thres_video = cv2.VideoWriter('/home/amsl/Desktop/thres_center.avi',fourcc_thres, 20.0, (640,480), False)
#final_video = cv2.VideoWriter('/home/amsl/Desktop/final_center.avi',fourcc_final, 20.0, (640,480))

def callback(data):
    global res
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.encoding)
    try:
        img = CvBridge().imgmsg_to_cv2(data,"bgr8")
    except CvBridgeError as e:
        print(e)

    #cv2.imshow("original",img)
    #cv2.waitKey(1)
    #cv2.imwrite("/home/amsl/Desktop/img.jpg", img)   
    
    img_gray = Gray(img)

    img_blur = Blur(img_gray)

    img_threshold = Threshold(img_blur)

    contours = cv2.findContours(img_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
    
    contours_large = AreaFilter(contours, img)

    contours_center = CenterFilter(contours_large, img)

    approx_contours = Approximation(contours_center, img)

    approx_contours_large = PointFilter(approx_contours, img)

    approx_contours_rectangle = RectangleFilter(approx_contours_large, img)

    if(approx_contours_rectangle):
        res = True
    else:
        res = False
     
    #thres_video.write(img_threshold)
    ShowContoursImage(approx_contours_rectangle, img, "final_image")
    

def Gray(img):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #cv2.imshow('gray', img_gray)
    #cv2.waitKey(1)
    #cv2.imwrite('/home/amsl/Desktop/gray.jpg', img_gray)

    return img_gray

def Blur(img):
    img_blur = cv2.GaussianBlur(img, (11, 11), 0)

    #cv2.imshow('blur', img_blur)
    #cv2.waitKey(1)
    #cv2.imwrite('/home/amsl/Desktop/blur.jpg', img_blur)

    return img_blur

def RemoveBlack(img):

    height, width = img.shape
    
    for high in range(height):
        for wide in range(width):
            if img[high][wide] < 160:
                img[high][wide] = 160
            else:
                pass

    cv2.imshow('remove_black', img)
    cv2.waitKey(1)
    #cv2.imwrite('/home/amsl/Desktop/blur.jpg', img_blur)

    return img

def Threshold(img):
    ret, img_threshold = cv2.threshold(img, 160, 255, cv2.THRESH_BINARY)

    if not ret:
        rospy.signal_shutdown("error in Threshold")

    #cv2.imshow('threshold', img_threshold)
    #cv2.waitKey(1)
    #cv2.imwrite('/home/amsl/Desktop/threshold.jpg', img_threshold)

    return img_threshold

def AreaFilter(contours, img):
    min_th_area = img.shape[0] * img.shape[1] / 100
    max_th_area = img.shape[0] * img.shape[1] / 3
    contours_large = list(filter(lambda c:cv2.contourArea(c) > min_th_area, contours))
    contours_large = list(filter(lambda c:cv2.contourArea(c) < max_th_area, contours_large))

    #ShowContoursImage(contours_large, img, "AreaFilter")

    return contours_large

def CenterFilter(contours, img):
    contours_center = []
    height = img.shape[0]
    print(img.shape)
    for i, cnt in enumerate(contours):
        M = cv2.moments(cnt)
        cy = int(M['m01']/M['m00'])
        print(cy)
        if cy > (height/2):
            contours_center.append(cnt)

    #ShowContoursImage(contours_center, img, "CenterFilter")

    return contours_center


def Approximation(contours, img):
    approx_contours = []
    for i, cnt in enumerate(contours):
        arclen = cv2.arcLength(cnt, True)
        approx_cnt = cv2.approxPolyDP(cnt, epsilon=0.005 * arclen, closed=True)
        approx_contours.append(approx_cnt)

    #ShowContoursImage(contours_large, img, "Approximation")

    return approx_contours

def PointFilter(contours, img):
    approx_contours_large = []
    for i in range(len(contours)):
        if len(contours[i]) < 10:
            approx_contours_large.append(contours[i])

    #ShowContoursImage(contours_large, img, "PointFilter")

    return approx_contours_large

def calc_rect_area(rect_points):
    x1, y1 = rect_points[0]
    x2, y2 = rect_points[1]
    x3, y3 = rect_points[2]

    w = math.sqrt((x1-x2)**2 + (y1-y2)**2)
    h = math.sqrt((x2-x3)**2 + (y2-y3)**2)

    return w*h

def RectangleFilter(contours, img):
    approx_contours_rectangle = []
    for i, cnt in enumerate(contours):
        rect = cv2.minAreaRect(cnt)
        (cx, cy), (width, height), angle = rect
        rect_points = cv2.boxPoints(rect)
        rect_area = calc_rect_area(rect_points)
        area = cv2.contourArea(cnt)
        if(area > rect_area * 0.80):
            approx_contours_rectangle.append(cnt)

    #ShowContoursImage(contours_large, img, "ReactangFilter")

    return approx_contours_rectangle

def ShowContoursImage(contours, img, name):
    try:
        ret_img = cv2.drawContours(img.copy(), contours, -1, color=(0, 255, 0), thickness=3)
        cv2.imshow(name, ret_img)
        #final_video.write(ret_img)
        #output_place = "/home/amsl/Desktop/" + name + ".jpg"
        #cv2.imwrite(output_place, ret_img)
        
    except:
        cv2.imshow(name,img)
        #final_video.write(img)
        #output_place = "/home/amsl/Desktop/" + name + ".jpg"
        #cv2.imwrite(output_place, img)

    cv2.waitKey(1)



def PublishRes():
    pub = rospy.Publisher('whiteline', Bool, queue_size=1)
    rospy.init_node('opencv_subscribe', anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo(res)
        pub.publish(res)
        r.sleep()

def main():
    try:
        PublishRes()
    except rospy.ROSInterruptException: pass


if __name__ == '__main__':
    main()


