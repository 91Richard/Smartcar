#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import time
import rospy
import cv2
import os
import sys
import glob
import numpy as np
import math
from std_msgs.msg import Bool
from  smartcar.msg import light

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
all_time = time.time()
global all_time
count_light = 0   #判断出现灯的次数
global count_light

def initial_parameters():
    global intrinsicMat
    global distortionCoe
    '''
    intrinsicMat = np.array([[1346, 0.6211 , 968.2755],
                            [0, 1344.7, 507.2217],
                            [0, 0, 1]])

    distortionCoe = np.array([-0.37,0.1097,0.0068,0.0034, 0])
    '''
class trafficLightDetector:                                          #最初使用摄像头直接读入的图像，因此需要去畸变，现在使用另一个节点直接读入去畸变后的图像，这里不再去畸变
    '''
    def initial_parameters(self):
        global intrinsicMat
        global distortionCoe
           #1w摄像头的参数
        intrinsicMat = np.array([[1346, 0.6211 , 968.2755],
                            [0, 1344.7, 507.2217],
                            [0, 0, 1]])

        distortionCoe = np.array([-0.37,0.1097,0.0068,0.0034, 0])
           #普通摄像头的参数
        intrinsicMat = np.array([[428.264533,-0.2570289,295.1081],
                            [0, 427.8575,253.5551],
                            [0, 0, 1]])

        distortionCoe = np.array([-0.38682604,0.13534661,8.18363975e-5,-2.866536872e-4 0])
    '''
    def __init__(self):
        #self.initial_parameters()
        has_red_light = Bool()
        self.rawImg = rospy.Publisher('raw',Image,queue_size=1)       #发布原始图像
        self.msgPub = rospy.Publisher("light_msg",light,queue_size = 1)
        self.cvb = CvBridge()
        self.redlightpub = rospy.Publisher('has_red_light', Bool, queue_size=1)   #判断是否存在红灯
        self.Redlight = rospy.Publisher('redLight',Image,queue_size = 1)     #输出canny算子处理后的图像，用于调试
        rospy.init_node('traffic_light_detection', anonymous=True)    #初始化节点
        print("detect inital success")
        rate = rospy.Rate(20) # 10hz
        rospy.Subscriber('orignial_images', Image, self.callback)       #订阅original_images节点
#================================老版回调函数，新版使用light.msg数据格式进行传递=====================================================
    '''
    def callback(self,imgmsg):
        c1 = cv2.getTickCount()
        img = self.cvb.imgmsg_to_cv2(imgmsg)
        #undstrt = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
        #judge1 = self.line_detection(img)    #判断是否存在停车线
        judge1 =True                          #这里由于场地限制暂不加入停车线检测，在实际场地中你们可以加入这个检测环节
        hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)   #转化为HSV格式图像，更好得处理光线对图像的影响。（由于处理器处理能力限制那个1w的摄像头没法用，读入的图像只有5帧以内，调节不及时）
        red_lower = np.array([0,85,220])        #这两个是阈值，红灯的上界和下界
        red_upper = np.array([20,255,255])      #可以根据现场情况调节
        green_lower = np.array([44,33,220])
        green_upper = np.array([131,255,255])
        red_mask = cv2.inRange(hsv,red_lower,red_upper)   #遮蔽
        red_target = cv2.bitwise_and(img,img,mask = red_mask)    
            #for i in range(red_mask.shape[1]):
            #    for j in range(red_mask.shape[0]):
            #        if red_target[j][i][0] + red_target[j][i][1]+red_target[j][i][2] != 0 :
            #            red_target[j][i] = [10,63,255]
        element = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
        red_target = cv2.erode(red_target,element)       #腐蚀膨胀去噪点，可以百度开闭运算
        red_target = cv2.dilate(red_target,element)
        red_gray = cv2.cvtColor(red_target,cv2.COLOR_BGR2GRAY)
        r_ret,r_binary = cv2.threshold(red_mask,127,255,cv2.THRESH_BINARY) #转化为灰度图像再用canny算子处理，canny算子只能处理灰度图
        r_gray2 = cv2.Canny(r_binary, 100, 200) 

        #====================================最初的想法，使用霍夫圆检测，学弟你们可以试试机器学习的方法====================================
       	#circles_red=cv2.HoughCircles(r_gray2,cv2.HOUGH_GRADIENT,1,600,param1=255,param2=30,minRadius=15,maxRadius=100)     
        #if(circles_red is None):
        #    redLight = 0sedx
        #    has_red_light = False
        #else:
        #    redLight = 1
        #    has_red_light = True
        #===============================================================================================================================
        
        #调试时输出图像
        #self.Redlight.publish(CvBridge().cv2_to_imgmsg(r_gray2)) 

        #=============================================统计二值化之后白点个数，也就是识别出的红像素点个数==================================
        g = r_gray2[:,:] == 255
        count = len(r_gray2[g]) 
        #===============================================================================================================================
        if count>700:                  #!!!!!!!!这个需要你们自己调，注意，动态的点个数和静态的点个数会有一些区别，使用动态点个数作为下界
            has_red_light = True
            print("red_light detected!")
        else:
            has_red_light = False
        print(count)
        stop_judge = has_red_light and judge1  #存在红灯且存在停车线，则停车
        self.redlightpub.publish(stop_judge)
        print(stop_judge)
        c2 = cv2.getTickCount()
        cycle_period = (c2 - c1) / cv2.getTickFrequency()
        print'cycle: %.4f'%(cycle_period)
    '''
#========================================================================================================================================
    def callback(self,imgmsg):
        c1 = cv2.getTickCount()
        img = self.cvb.imgmsg_to_cv2(imgmsg)
        #judge1 = self.line_detection(img)    #判断是否存在停车线
        judge1 =True                          #这里由于场地限制暂不加入停车线检测，在实际场地中你们可以加入这个检测环节
        hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)   #转化为HSV格式图像，更好得处理光线对图像的影响。（由于处理器处理能力限制那个1w的摄像头没法用，读入的图像只有5帧以内，调节不及时）
        hsv1 = hsv.copy()
        element = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
        red_lower = np.array([0,85,220])        #这两个是阈值，红灯的上界和下界
        red_upper = np.array([20,255,255])
        red_mask = cv2.inRange(hsv,red_lower,red_upper)
        red_target = cv2.bitwise_and(hsv,hsv,mask = red_mask)
        red_target = cv2.erode(red_target,element)
        red_target = cv2.dilate(red_target,element)
        red_gray = cv2.cvtColor(red_target,cv2.COLOR_BGR2GRAY)
        r_ret,r_binary = cv2.threshold(red_mask,127,255,cv2.THRESH_BINARY)
        r_gray2 = cv2.Canny(r_binary, 100, 200) 
        r = r_gray2[:,:] == 255
        count_red = len(r_gray2[r])
        #print(count)   #count around 500   distance is approximately 1.6m
                       #unuse of hough circle detection
        if count_red>300:
            redLight = 1
        else:
            redLight = 0
        green_lower = np.array([50,85,220])    #这个阈值有问题,在寝室调不出来
        green_upper = np.array([140,255,255])
        green_mask = cv2.inRange(hsv1,green_lower,green_upper)
        green_target = cv2.bitwise_and(hsv1,hsv1,mask = green_mask)
        green_target = cv2.erode(green_target,element)
        green_target = cv2.dilate(green_target,element)
        green_gray = cv2.cvtColor(green_target,cv2.COLOR_BGR2GRAY)
        g_ret,g_binary = cv2.threshold(green_mask,127,255,cv2.THRESH_BINARY)
        g_gray2 = cv2.Canny(g_binary, 100, 200)       
        g = g_gray2[:,:] == 255
        count_green = len(g_gray2[g])
        #print(count_green)
        if count_green>500:
            greenLight = 1
        else:
            greenLight = 0
        if greenLight + redLight >0 :
            hasLight = 1
            if greenLight == 1:
                Light = 1
            elif redLight == 1:
                Light = 2       
        else:
            hasLight = 0
            Light = 0
        global all_time
        global count_light

        if time.time()-all_time <= 5 and hasLight == 1:
            all_time = time.time()
        if time.time()-all_time > 5 and hasLight ==1:
            count_light = count_light + 1
            all_time = time.time()

        if count_light ==0 and hasLight ==1:
            count_light = 1

        print(time.time()-all_time) 
        print(count_light)                                     
        msg = light()
        msg.exist = hasLight
        msg.red_or_green = Light
        msg.show_time = count_light ##yong yu huitiao han shu 
        self.msgPub.publish(msg)
        print("light",Light)
        
    def line_detection(self,img):          #停车线检测  （这个那个1w的摄像头倾角比较好用，但是用当前摄像头效果不是很好，可以不用直线检测）
        result = img.copy()
        result = result[380:480,:]    #截取小窗去除环境影响
        line_count = 0
        result = cv2.cvtColor(result,cv2.COLOR_BGR2GRAY)
        r_ret,r_binary = cv2.threshold(result,127,255,cv2.THRESH_BINARY)
        #img = self.cvb.imgmsg_to_cv2(imgmsg)
        edges = cv2.Canny(r_binary, 100, 200, apertureSize=3)
        lines = cv2.HoughLines(edges, 1, np.pi/180, 100,200)    #主要修改后面两个参数，其中一个是直线的最短长度，也就是小于该长度的直线都被忽略，能够去环境噪声，具体百度。
        if lines == None:
            self.rawImg.publish(CvBridge().cv2_to_imgmsg(result))              
            return False
        for line in lines:     #这边写的显示有点问题，学弟你们如果想看识别出的直线记得改一下
            rho = line[0][0]
            theta = line[0][1]
            if theta>(np.pi/2-np.pi/6) and theta<(np.pi/6+np.pi/2):
                line_count = line_count +1
                x1 = int(rho*np.cos(theta))
                y1 = int(rho*np.sin(theta))
                if x1 == 0:
                    x2 = x1 +500
                    y2 = y1 
                #x2 = int((rho-result.shape[0]*np.sin(theta))/np.cos(theta))
                else:
                    x2 = int(x1+500)
                    y2 = int(y1+((-x1/y1)*500))
                #if x1<0:
                #    x1 = 0
                # x2>640:
                #    x2 = 640
                pt1 = (x1,y1)
                pt2 = (x2,y2)
                cv2.line(edges, pt1, pt2, (255))
                #print(pt1,pt2)
        self.rawImg.publish(CvBridge().cv2_to_imgmsg(edges))
        if line_count > 0:   #一般不会检测到太多条，1~2条差不多了
            print("line detected")
            return True
        else:
            return False 

if __name__ == "__main__":
    try:
        detector = trafficLightDetector()
        rospy.spin()   #防止程序退出，保证主程序一直循环
    except rospy.ROSInterruptException:
        cv.destroyAllWindows()

#========================================================以下用于没有加入其它节点的调试============================================
#img = cv2.imread('2.jpg')
#msgPub = rospy.Publisher("light_msg",light,queue_size = 1)
'''
def nothing(x):   #这个莫得用
    pass
def main():
    cvb = CvBridge()
    rospy.init_node('traffic_light_detection',anonymous = True)
    rawImg = rospy.Publisher('raw',Image,queue_size=1)
    Redlight = rospy.Publisher('redLight',Image,queue_size = 1)
    element = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))

    cv2.namedWindow('image')
    cv2.createTrackbar('Hlow','image',0,180,nothing)
    cv2.createTrackbar('Slow','image',0,255,nothing)
    cv2.createTrackbar('Vlow','image',0,255,nothing)
    cv2.createTrackbar('Hhigh','image',0,180,nothing)
    cv2.createTrackbar('Shigh','image',0,255,nothing)
    cv2.createTrackbar('Vhigh','image',0,255,nothing)

    video = cv2.VideoCapture(0) 
    while video.isOpened():
        ret, frame = video.read()
        if ret:     
            hl=cv2.getTrackbarPos('Hlow','image')
            sl=cv2.getTrackbarPos('Slow','image')
            vl=cv2.getTrackbarPos('Vlow','image')
            hh=cv2.getTrackbarPos('Hhigh','image')
            sh=cv2.getTrackbarPos('Shigh','image')
            vh=cv2.getTrackbarPos('Vhigh','image')
            hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
            hsv1 = hsv.copy()
            #img = cv2.imread('./catkin_ws/src/smartcar/scripts/red_sample.jpg')
            red_lower = np.array([0,85,220])        #这两个是阈值，红灯的上界和下界
            red_upper = np.array([20,255,255])
            red_mask = cv2.inRange(hsv,red_lower,red_upper)
            red_target = cv2.bitwise_and(hsv,hsv,mask = red_mask)
            #for i in range(red_mask.shape[1]):
            #    for j in range(red_mask.shape[0]):
            #        if red_target[j][i][0] + red_target[j][i][1]+red_target[j][i][2] != 0 :
            #            red_target[j][i] = [10,63,255]
            red_target = cv2.erode(red_target,element)
            red_target = cv2.dilate(red_target,element)
            red_gray = cv2.cvtColor(red_target,cv2.COLOR_BGR2GRAY)
            r_ret,r_binary = cv2.threshold(red_mask,127,255,cv2.THRESH_BINARY)
            r_gray2 = cv2.Canny(r_binary, 100, 200) 
            r = r_gray2[:,:] == 255
            count_red = len(r_gray2[r])
            #print(count)   #count around 500   distance is approximately 1.6m
                           #unuse of hough circle detection
                        
       	    #circles_red=cv2.HoughCircles(r_gray2,cv2.HOUGH_GRADIENT,1,600,param1=255,param2=30,minRadius=15,maxRadius=100)
            #if(circles_red is None):
            #    redLight = 0
            #else:
            #    redLight = 1
            #    print("redlight detected!!")
            rawImg.publish(cvb.cv2_to_imgmsg(frame))
            if count_red>500:
                redLight = 1
            else:
                redLight = 0
            green_lower = np.array([50,85,220])    #这个阈值有问题,在寝室调不出来
            green_upper = np.array([140,255,255])
            green_mask = cv2.inRange(hsv1,green_lower,green_upper)
            green_target = cv2.bitwise_and(hsv1,hsv1,mask = green_mask)
            green_target = cv2.erode(green_target,element)
            green_target = cv2.dilate(green_target,element)
            green_gray = cv2.cvtColor(green_target,cv2.COLOR_BGR2GRAY)
            g_ret,g_binary = cv2.threshold(green_mask,127,255,cv2.THRESH_BINARY)
            g_gray2 = cv2.Canny(g_binary, 100, 200)       
            g = g_gray2[:,:] == 255
            count_green = len(g_gray2[g])
            print(count_green)
            cv2.imshow("fuck",g_gray2)
            cv2.waitKey(1)
            if count_green>500:
                greenLight = 1
            else:
                greenLight = 0
            if greenLight + redLight >0 :
                hasLight = 1
                if greenLight == 1:
                    Light = 1
                else:
                    Light = 0       
            else:
                hasLight = 0
                Light = 0
            global all_time
            global count_light
            if time.time()-all_time > 10 and hasLight ==1:
                count_light = count_light + 1
                all_time = time.time()
            msg = light()
            msg.exist = hasLight
            msg.red_or_green = Light
            msg.show_time = count_light
            msgPub.publish(msg)
            print(Light)
            print()
'''
'''
if __name__ =="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        cv.destroyAllWindows()
'''
#=======================================================================================================================================
def line_detection(img):     #直线检测调试
        result = img.copy()
        result = result[240:480,:]
        line_count = 0
        result = cv2.cvtColor(result,cv2.COLOR_BGR2GRAY)
        #img = self.cvb.imgmsg_to_cv2(imgmsg)
        edges = cv2.Canny(result, 50, 150, apertureSize=3)
        lines = cv2.HoughLines(edges, 1, np.pi/180, 100,200)
        if lines == None:
            rawImg.publish(CvBridge().cv2_to_imgmsg(result))
            return False
        for line in lines:
            rho = line[0][0]
            theta = line[0][1]
            if theta>(np.pi/2-np.pi/18) and theta<(np.pi/18+np.pi/2):
                line_count = line_count +1
                x1 = int(rho*np.cos(theta))
                y1 = int(rho*np.sin(theta))
                if x1 == 0:
                    x2 = x1 +500
                    y2 = y1 
                #x2 = int((rho-result.shape[0]*np.sin(theta))/np.cos(theta))
                else:
                    x2 = int(x1+500)
                    y2 = int(y1+((-x1/y1)*500))
                #if x1<0:
                #    x1 = 0
                #if x2>640:
                #    x2 = 640
                pt1 = (x1,y1)
                pt2 = (x2,y2)
                cv2.line(result, pt1, pt2, (255))
                #print(pt1,pt2)
        rawImg.publish(CvBridge().cv2_to_imgmsg(result))
        if line_count > 1:
            return True
        else:
            return False 
        
'''
if __name__ =="__main__":
    initial_parameters()
    rospy.init_node('traffic_light_detection',anonymous = True)
    global rawImg
    rawImg = rospy.Publisher('raw',Image,queue_size=1)
    Video = cv2.VideoCapture(1)
    while Video.isOpened():
        ret, img = Video.read()
        if ret:
            img =cv2.resize(img,(640,480))
            undstrt = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
            boo = line_detection(undstrt)
            print(boo)
'''
