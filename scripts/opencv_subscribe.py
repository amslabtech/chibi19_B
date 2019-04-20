#!/usr/bin/env python
# coding: UTF-8

import rospy
import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Circle, Polygon, Rectangle
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError


#fourcc = cv2.VideoWriter_fourcc(*'MJPG')
#out = cv2.VideoWriter('/home/amsl/Desktop/output.avi',fourcc, 20.0, (640,480))


def listener():
    rospy.init_node('opencv_subscribe', anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    rospy.spin()

def callback(data):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.encoding)
    try:
        img = CvBridge().imgmsg_to_cv2(data,"bgr8")
    except CvBridgeError as e:
        print(e)
    #cv2.imshow("original",img)
    #cv2.imwrite("/home/amsl/Desktop/img.jpg", img)

    # グレー化
    img_gray = Gray(img)

    # ぼかし
    img_blur = Blur(img_gray)

    # 二値化
    img_threshold = Threshold(img_blur)

    # 白と黒の境界を構成している点群の取得
    contours = cv2.findContours(img_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]

    # 面積フィルター
    contours_large = AreaFilter(contours, img)

    # 輪郭を構成する点群の近似化
    approx_contours = Approximation(contours_large, img)

    # 近似化した点群の点の数を対象としたフィルター
    approx_contours_large = PointFilter(approx_contours, img)

    # 点群を矩形で囲み、点群の面積と矩形の面積を比較することで、点群が矩形か判断する
    approx_contours_rectangle = RectangleFilter(approx_contours_large, img)

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

def Threshold(img):
    ret, img_threshold = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

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
        cv2.imshow(ret_img)
        #output_place = "/home/amsl/Desktop/" + name + ".jpg"
        #cv2.imwrite(output_place, ret_img)
    except:
        cv2.imshow(img)
        #output_place = "/home/amsl/Desktop/" + name + ".jpg"
        #cv2.imwrite(output_place, img)



 #   img_contours = img.copy()
 #   for i, cnt in enumerate(contours):
 #       for j in range(len(cnt)):
 #           x2, y2 = cnt[j][0]
 #           if j is 0:
 #               num = len(cnt)-1
 #               x1, y1 = cnt[num][0]
 #               img_contours = cv2.line(img_contours, (x1,y1) , (x2,y2), (0,0,255), 3)
 #           else:
 #               img_contours = cv2.line(img_contours, (x1,y1) , (x2,y2), (0,0,255), 3)
 #           x1, y1 = x2, y2


#    img_contours_large = img.copy()
#    for i, cnt in enumerate(contours_large):
#        for j in range(len(cnt)):
#            x2, y2 = cnt[j][0]
#            if j is 0:
#                num = len(cnt)-1
#                x1, y1 = cnt[num][0]
#                img_contours_large = cv2.line(img_contours_large, (x1,y1) , (x2,y2), (0,0,255), 3)
#            else:
#                img_contours_large = cv2.line(img_contours_large, (x1,y1) , (x2,y2), (0,0,255), 3)
#            x1, y1 = x2, y2


#    img_approx_contours = img.copy()
#    for i, cnt in enumerate(approx_contours):
#        for j in range(len(cnt)):
#            x2, y2 = cnt[j][0]
#            if j is 0:
#                num = len(cnt)-1
#                x1, y1 = cnt[num][0]
#                img_approx_contours = cv2.line(img_approx_contours, (x1,y1) , (x2,y2), (0,0,255), 3)
#            else:
#                img_approx_contours = cv2.line(img_approx_contours, (x1,y1) , (x2,y2), (0,0,255), 3)
#            x1, y1 = x2, y2


#    img_approx_contours_large = img.copy()
#    for i, cnt in enumerate(approx_contours_large):
#        for j in range(len(cnt)):
#            x2, y2 = cnt[j][0]
#            if j is 0:
#                num = len(cnt)-1
#                x1, y1 = cnt[num][0]
#                img_approx_contours_large = cv2.line(img_approx_contours_large, (x1,y1) , (x2,y2), (0,0,255), 3)
#            else:
#                img_approx_contours_large = cv2.line(img_approx_contours_large, (x1,y1) , (x2,y2), (0,0,255), 3)
#            x1, y1 = x2, y2



#    img_final_contours = img.copy()
#    for i, cnt in enumerate(final_contours):
#        for j in range(len(cnt)):
#            x2, y2 = cnt[j][0]
#            if j is 0:
#                num = len(cnt)-1
#                x1, y1 = cnt[num][0]
#                img_final_contours = cv2.line(img_final_contours, (x1,y1) , (x2,y2), (0,0,255), 3)
#            else:
#                img_final_contours = cv2.line(img_final_contours, (x1,y1) , (x2,y2), (0,0,255), 3)
#            x1, y1 = x2, y2


    #cv2.imshow("contours_large", img_contours_large)
    #cv2.imshow("contours", img_approx_contours)
    #cv2.imshow("approx_contours_large", img_approx_contours_large)
    #cv2.imshow("final_contours", img_final_contours)

    #cv2.imshow("contours", img_contours)
    #cv2.imwrite("/home/amsl/Desktop/contours.jpg", img_contours)
    #cv2.imwrite("/home/amsl/Desktop/contours_large.jpg", img_contours_large)
    #cv2.imwrite("/home/amsl/Desktop/approx_contours.jpg", img_approx_contours)
    #cv2.imwrite("/home/amsl/Desktop/approx_contours_large.jpg", img_approx_contours_large)

    #cv2.imwrite(threshold.jpg', img_threshold)

    






if __name__ == '__main__':
    listener()


