#!/usr/bin/env python

import rospy
import cv2
import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Circle, Polygon, Rectangle
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


#fourcc = cv2.VideoWriter_fourcc(*'MJPG')
#out = cv2.VideoWriter('/home/amsl/Desktop/output.avi',fourcc, 20.0, (640,480))

def calc_rect_area(rect_points):
    x1, y1 = rect_points[0]
    x2, y2 = rect_points[1]
    x3, y3 = rect_points[2]

    w = math.sqrt((x1-x2)**2 + (y1-y2)**2)
    h = math.sqrt((x2-x3)**2 + (y2-y3)**2)

    return w*h


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

    img_blur = cv2.GaussianBlur(img_gray, (11, 11), 0)

    # binarization
    ret, img_threshold = cv2.threshold(img_blur, 200, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    contours = cv2.findContours(img_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
    min_th_area = img.shape[0] * img.shape[1] / 100
    max_th_area = img.shape[0] * img.shape[1] / 3
    contours_large = list(filter(lambda c:cv2.contourArea(c) > min_th_area, contours))
    contours_large = list(filter(lambda c:cv2.contourArea(c) < max_th_area, contours_large))

    approx_contours = []
    for i, cnt in enumerate(contours_large):
        arclen = cv2.arcLength(cnt, True)
        approx_cnt = cv2.approxPolyDP(cnt, epsilon=0.005 * arclen, closed=True)
        approx_contours.append(approx_cnt)

    approx_contours_large = []
    for i in range(len(approx_contours)):
        if len(approx_contours[i]) < 10:
            approx_contours_large.append(approx_contours[i])

    final_contours = []
    for i, cnt in enumerate(approx_contours_large):
        rect = cv2.minAreaRect(cnt)
        (cx, cy), (width, height), angle = rect
        rect_points = cv2.boxPoints(rect)
        rect_area = calc_rect_area(rect_points)
        area = cv2.contourArea(cnt)
        if(area > rect_area * 0.80):
            final_contours.append(cnt)

    img_contours = img.copy()
    for i, cnt in enumerate(contours):
        for j in range(len(cnt)):
            x2, y2 = cnt[j][0]
            if j is 0:
                num = len(cnt)-1
                x1, y1 = cnt[num][0]
                img_contours = cv2.line(img_contours, (x1,y1) , (x2,y2), (0,0,255), 3)
            else:
                img_contours = cv2.line(img_contours, (x1,y1) , (x2,y2), (0,0,255), 3)
            x1, y1 = x2, y2


    img_contours_large = img.copy()
    for i, cnt in enumerate(contours_large):
        for j in range(len(cnt)):
            x2, y2 = cnt[j][0]
            if j is 0:
                num = len(cnt)-1
                x1, y1 = cnt[num][0]
                img_contours_large = cv2.line(img_contours_large, (x1,y1) , (x2,y2), (0,0,255), 3)
            else:
                img_contours_large = cv2.line(img_contours_large, (x1,y1) , (x2,y2), (0,0,255), 3)
            x1, y1 = x2, y2


    img_approx_contours = img.copy()
    for i, cnt in enumerate(approx_contours):
        for j in range(len(cnt)):
            x2, y2 = cnt[j][0]
            if j is 0:
                num = len(cnt)-1
                x1, y1 = cnt[num][0]
                img_approx_contours = cv2.line(img_approx_contours, (x1,y1) , (x2,y2), (0,0,255), 3)
            else:
                img_approx_contours = cv2.line(img_approx_contours, (x1,y1) , (x2,y2), (0,0,255), 3)
            x1, y1 = x2, y2


    img_approx_contours_large = img.copy()
    for i, cnt in enumerate(approx_contours_large):
        for j in range(len(cnt)):
            x2, y2 = cnt[j][0]
            if j is 0:
                num = len(cnt)-1
                x1, y1 = cnt[num][0]
                img_approx_contours_large = cv2.line(img_approx_contours_large, (x1,y1) , (x2,y2), (0,0,255), 3)
            else:
                img_approx_contours_large = cv2.line(img_approx_contours_large, (x1,y1) , (x2,y2), (0,0,255), 3)
            x1, y1 = x2, y2
    #cv2.imwrite('/home/amsl/Desktop/white_line.jpg', img_approx_contours_large)
    #out.write(img_approx_contours_large)



    img_final_contours = img.copy()
    for i, cnt in enumerate(final_contours):
        for j in range(len(cnt)):
            x2, y2 = cnt[j][0]
            if j is 0:
                num = len(cnt)-1
                x1, y1 = cnt[num][0]
                img_final_contours = cv2.line(img_final_contours, (x1,y1) , (x2,y2), (0,0,255), 3)
            else:
                img_final_contours = cv2.line(img_final_contours, (x1,y1) , (x2,y2), (0,0,255), 3)
            x1, y1 = x2, y2


    #cv2.imshow("img", img)
    #cv2.imshow('gray', img_gray)
    #cv2.imshow('blur', img_blur)
    #cv2.imshow('threshold', img_threshold)
    #cv2.imshow("contours", img_contours)
    #cv2.imshow("contours_large", img_contours_large)
    #cv2.imshow("contours", img_approx_contours)
    #cv2.imshow("approx_contours_large", img_approx_contours_large)
    #cv2.imshow("final_contours", img_final_contours)

    cv2.imwrite("/home/amsl/Desktop/img.jpg", img)
    cv2.imwrite('/home/amsl/Desktop/gray.jpg', img_gray)
    cv2.imwrite('/home/amsl/Desktop/blur.jpg', img_blur)
    cv2.imwrite('/home/amsl/Desktop/threshold.jpg', img_threshold)
    cv2.imwrite("/home/amsl/Desktop/contours.jpg", img_contours)
    cv2.imwrite("/home/amsl/Desktop/contours_large.jpg", img_contours_large)
    cv2.imwrite("/home/amsl/Desktop/approx_contours.jpg", img_approx_contours)
    cv2.imwrite("/home/amsl/Desktop/approx_contours_large.jpg", img_approx_contours_large)
    cv2.imwrite("/home/amsl/Desktop/final_contours.jpg", img_final_contours)

    #cv2.imwrite(threshold.jpg', img_threshold)

    if cv2.waitKey(3) == 'q':
        out.release()

def listener():
    rospy.init_node('opencv_subscribe', anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()


