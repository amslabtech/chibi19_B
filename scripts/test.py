#!/usr/bin/env python 

import rospy
import numpy as np
import cv2
#from sensor_.msgs import Image

#def callback(data):
#    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.encoding)
#
#    cv2.imshow("input", data.data)
#
#    cv2.waitKey(0)
    #cv2.destroyAllWindows()
    
def listener():
     #cv2.namedWindow('img', cv2.WINDOW_NORMAL)

    img = cv2.imread('white_line.png') 
    cv2.imshow("original", img)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, img_threshold = cv2.threshold(img_gray,200,255,cv2.THRESH_BINARY)
    img_canny = cv2.Canny(img_threshold, 50, 110)


#    lines = cv2.HoughLinesP(img_canny, rho=1, theta=np.pi/360, threshold=30, minLineLength=200, maxLineGap=20)
#    print(lines)
#    for line in lines:
#        x1, y1, x2, y2 = line[0]
#        #redline
#        img_red_line = cv2.line(img, (x1,y1), (x2,y2), (0,0,255), 3)


    cv2.imshow("grayscale", img_gray)
    cv2.imshow("threshold", img_threshold)
    cv2.imshow("canny", img_canny)
#    cv2.imshow("red_line", img_red_line)


    #cv2.imwrite("./output/img_otsu.jpg",img_otsu)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    #print('Hello!!!!!!!!!!!!!')
    
    #rospy.init_node('opencv_subscribe', anonymous=True)
    #rospy.Subscriber("usb_cam/image_raw", Image, callback)
   # rospy.spin()
        
if __name__ == '__main__':
    listener()


