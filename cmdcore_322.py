#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import cv2
import os
import sys
import glob
import numpy as np
import math


from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from smartcar.msg import light
intrinsicMat = []
distortionCoe = []
prspct_trans_mat = []
perspective_transform_matrix = []
kernel = []

#not detect the green light
starter = True
#ignore the lidar
global lidarLaunch
lidarLaunch = False
##roadWidth = 65.0 /(80/300.0)
class KalmanFilter:

    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    def Estimate(self, coordX, coordY):
        ''' This function estimates the position of the object'''
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        self.kf.correct(measured)
        predicted = self.kf.predict()
        return predicted

def initial_parameters():
    global intrinsicMat
    global distortionCoe
    global perspective_transform_matrix
    global perspective_transform_matrix_slope
    global kernel
    global ramp_control
    global part_A

    ##ramp_control = True
    
    intrinsicMat = np.array([[428.264533, -0.2570289, 295.1081],
                            [0, 427.8575, 253.5551],
                            [0, 0, 1]])

    distortionCoe = np.array([-0.38682604, 0.13534661, 8.18363975e-5, -2.8665336872e-4, 0])
    
    startx = 120
    starty = 50
    length_pers = 400
    width_pers = length_pers
    srcps = np.float32([[(186,354), (85,431), (532,428), (406,347)]])
    dstps = np.float32([[(startx, starty), (startx, starty + width_pers), (startx + length_pers, starty + width_pers), (startx + length_pers, starty)]])
    perspective_transform_matrix = cv2.getPerspectiveTransform(srcps, dstps)
    

    startx = 170
    starty = -600
    length_pers = 300
    width_pers = 1080#length_pers
    srcps = np.float32([[(176,271), (17,472), (534,448), (416,267)]])#([[(27, 349), (177, 207), (452, 207), (599, 349)]])
    dstps = np.float32([[(startx, starty), (startx, starty + width_pers), (startx + length_pers, starty + width_pers), (startx + length_pers, starty)]])
    perspective_transform_matrix_slope = cv2.getPerspectiveTransform(srcps, dstps)              

    

    '''
    intrinsicMat = np.array([[669.0672, -0.2097, 490.6801],
                            [0, 671.0723, 283.2345],
                            [0, 0, 1]])

    distortionCoe = np.array([-0.3739,0.1119,3.5478e-04,0.002, 0])
    

    startx = 280
    starty = 220
    length_pers = 400
    width_pers = length_pers
    srcps = np.float32([[(289,250), (93,415), (870,419), (680,256)]])
    #srcps_ramp = np.float32([[(27, 349), (177, 207), (452, 207), (599, 349)]])
    dstps = np.float32([[(startx, starty), (startx, starty + width_pers), (startx + length_pers, starty + width_pers), (startx + length_pers, starty)]])
    '''

    kernel = np.ones((3,3),np.uint8)



def perspectiveTrans(img):

    global perspective_transform_matrix    

    if perspective_transform_matrix==[]:
        print"Transform failed!"
        return img
    else:
        bird_view_img = cv2.warpPerspective(img, perspective_transform_matrix, img.shape[1::-1], flags=cv2.INTER_LINEAR)
        return bird_view_img

def perspectiveTrans_slope(img):

    global perspective_transform_matrix_slope    

    if perspective_transform_matrix_slope==[]:
        print"Transform failed!"
        return img
    else:
        bird_view_img = cv2.warpPerspective(img, perspective_transform_matrix_slope, img.shape[1::-1], flags=cv2.INTER_LINEAR)
        return bird_view_img

def blind_detection(gray):
    gray_Blur = cv2.GaussianBlur(gray, (5,5),0)
    canny = cv2.Canny(gray_Blur, 100, 200)
    
    lines = cv2.HoughLines(canny, 1, np.pi/180, 100 )
    
    #print lines.shape

    y_nearest = 0

    for line in lines:
        rho, theta = line[0]
        if theta > np.pi/2 + 5*np.pi/180 or theta < np.pi/2 - 5*np.pi/180:
            continue
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = 0
        y1 = int(rho/b)
        x2 = gray.shape[1] - 1
        y2 = int((rho-x2*a) / b)

        if y1 > y_nearest:
            y_nearest = y1
        if y2 > y_nearest:
            y_nearest = y2
                
        if y_nearest > gray.shape[0]*9/10:
            blind_detected = True
            return blind_detected, y_nearest

    blind_detected = False
    return blind_detected, y_nearest

class command:
    def __init__(self):
        count = 0
        #摄像头端口号
        self.cap = cv2.VideoCapture(0)
        self.pubI = rospy.Publisher('images', Image, queue_size=1)
        self.puborignialI = rospy.Publisher('orignial_images', Image, queue_size=1)
        rospy.init_node('command_core', anonymous=True)
        rospy.Subscriber("light_msg", light, RLcallback)
        rospy.Subscriber("laser_cmd", Twist, LScallback)
        self.rate = rospy.Rate(20)

        self.cvb = CvBridge()        

    def spin(self):
        global lidarLaunch
        threshold_value = 50
        global starter,pub,greenLight,aP,lastP, kernel
        global aP_kf
        global last_lane_base, last_LorR
        global final_cmd, cam_cmd, stop_judge_local
        global roadWidth
        global judge_end_tunnel
	last_lane_base = -1
        last_LorR = 1
        y_nearest = 0
        ##ramp_control = True          ################
        while not rospy.is_shutdown():
            ret, img = self.cap.read()
            if ret == True:
                if (not lidarLaunch):  
                    c1 = cv2.getTickCount()
                    kfObj = KalmanFilter()
        	    predictedCoords = np.zeros((2, 1), np.float32)
                    #img = cv2.pyrDown(img)
                    undstrt = cv2.undistort(img, intrinsicMat, distortionCoe, None, intrinsicMat)
                    self.puborignialI.publish(self.cvb.cv2_to_imgmsg(undstrt))                  

        	    #gray0 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    gray = cv2.cvtColor(undstrt, cv2.COLOR_BGR2GRAY)
                        
                    global blind_detected
                    global blind_detection_flag
                    global delay_flag
                    if ramp_control == True:##lv deng liang dier ci 
                        cam_cmd.angular.x = 1
                        print "...........Ramp_Control........."
                        roadWidth = 90 / (80/300.0)
                        if (blind_detected == False):
                            cam_cmd.linear.x = 0.1
                        if (blind_detection_flag == False) and (delay_flag == False):
                            print "!!!!!!!!!!!!!!!!!!!!!!!!!delay_start............................................"
                            beginning_time = cv2.getTickCount()
                            delay_flag = True
                        if delay_flag == True:
                            current_time = cv2.getTickCount()
                            delay_time = (current_time - beginning_time)/cv2.getTickFrequency()
                            print "delay_time:"
                            print delay_time
                            if delay_time >= 1:    #延迟时间,yibaiwan 1s
                                delay_flag = False
                                blind_detection_flag = True
                                print "...................................delay end!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                        if (blind_detected == False) and (blind_detection_flag == True):
                            blind_detected, y_nearest = blind_detection(gray)
                        if (blind_detected == True) and (blind_detection_flag == True):
                            print "__________ramp disappears_______________________"
                            cam_cmd.angular.z = 0
                            pub.publish(cam_cmd)
                            continue 
                        _, binary_gray = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
                        binary_gray[:y_nearest, :] = 0
                        binary_warped = perspectiveTrans_slope(binary_gray)
                        margin = 30
                        histogram = np.sum(binary_warped[binary_warped.shape[0]*4/5:,:], axis=0) #//2
                                            #same
                        lane_base = np.argmax(histogram)


                        nwindows = 10
                        window_height = int(binary_warped.shape[0]/nwindows)
                        nonzero = binary_warped.nonzero()
                        nonzeroy = np.array(nonzero[0])
                        nonzerox = np.array(nonzero[1])
                        lane_current = lane_base
                        #margin = 100
                        minpix = 1
                        lane_inds = []

                        binary_warped = cv2.cvtColor(binary_warped, cv2.COLOR_GRAY2BGR)


                        for window in range(nwindows):
                            win_y_low = binary_warped.shape[0] - (window + 1)*window_height
                            win_y_high = binary_warped.shape[0] - window*window_height 
                            win_x_low = lane_current - margin 
                            win_x_high = lane_current + margin 

                            cv2.rectangle(binary_warped, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0 ,0), 3)
                            good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                            
                            lane_inds.append(good_inds)
                            if len(good_inds) > minpix:
                                lane_current = int(np.mean(nonzerox[good_inds])) ####

                        lane_inds = np.concatenate(lane_inds)#数组拼接

                        pixelX = nonzerox[lane_inds]
                        pixelY = nonzeroy[lane_inds]
                        

                        # calculate the aimPoint
                        
                        if (pixelX.size == 0):
                            print "fffffffffffffffffffffffffff"
                            final_cmd = cam_cmd
                            print cam_cmd.linear.x
                            pub.publish(final_cmd)
                            self.pubI.publish(self.cvb.cv2_to_imgmsg(binary_warped))
                        #print "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"

                            #continue
                        try:
                            a1,a0 = np.polyfit(pixelX, pixelY, 1)
                            ###### to draw the fitted curve
                            
                            for i in range(binary_warped.shape[0]/5):
                                y = 5*i
                                x = int((y - a0)/a1)
                                cv2.circle(binary_warped, (x, y), 3, (0, 0, 255), -1)
                            


                        except:
                            print "xxxxxxxxxxxxxxxxxxxxxxxx"
                            cam_cmd.angular.z = 0;
                            final_cmd = cam_cmd
                            pub.publish(final_cmd)
                            continue
                        
                        
                        aveX = np.average(pixelX)
                        cv2.line(binary_warped, (int(aveX), 0), (int(aveX), 479), (0, 255, 255), 3)

                        


                        if aveX < binary_warped.shape[1]/2:
                            LorR = 1; #LeftLane
                        else:
                            LorR = -1; #RightLane
                        aimLaneP = [0,0]
                        aimLaneP[1] = 450
                        aimLaneP[0] = aveX
                        
                        
                   
                        aP[0] = aimLaneP[0] + LorR*roadWidth/2
                        aP[1] = aimLaneP[1]

                    else:
                        print ".........Track......."
                        gray[:gray.shape[0]/3,:] = 0

                        gray_Blur = cv2.GaussianBlur(gray, (5,5),0)
                   
                        origin_thr = cv2.Canny(gray_Blur, 100, 200) 
                    
                    

                        origin_thr = cv2.dilate(origin_thr, np.ones((10,10), np.uint8))
                        origin_thr = cv2.erode(origin_thr, np.ones((12,12), np.uint8))
                    
                        binary_warped =  perspectiveTrans(origin_thr)
                    
                        histogram = np.sum(binary_warped[binary_warped.shape[0]*2/3:,:], axis=0) #//2
                    
                        margin = 100

                        lane_base = np.argmax(histogram)


                        nwindows = 10
                        window_height = int(binary_warped.shape[0]/nwindows)
                        nonzero = binary_warped.nonzero()
                        nonzeroy = np.array(nonzero[0])
                        nonzerox = np.array(nonzero[1])
                        lane_current = lane_base
                        #margin = 100
                        minpix = 1
                        lane_inds = []

                        binary_warped = cv2.cvtColor(binary_warped, cv2.COLOR_GRAY2BGR)


                        for window in range(nwindows):
                            win_y_low = binary_warped.shape[0] - (window + 1)*window_height
                            win_y_high = binary_warped.shape[0] - window*window_height 
                            win_x_low = lane_current - margin 
                            win_x_high = lane_current + margin 

                            cv2.rectangle(binary_warped, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0 ,0), 3)
                            good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                            
                            lane_inds.append(good_inds)
                            if len(good_inds) > minpix:
                                lane_current = int(np.mean(nonzerox[good_inds])) ####

                        lane_inds = np.concatenate(lane_inds)#数组拼接

                        pixelX = nonzerox[lane_inds]
                        pixelY = nonzeroy[lane_inds]
                        
                        # calculate the aimPoint
                        
                        if (pixelX.size == 0):
                            continue
                        aveX = np.average(pixelX)
                        try:
                            
                            a2,a1,a0 = np.polyfit(pixelX, pixelY, 2);
                                ###### to draw the fitted curve
                            for i in range(binary_warped.shape[1]/5):
                                x = 5*i
                                y = int(a2*x**2 + a1*x + a0)
                                cv2.circle(binary_warped, (x, y), 3, (0, 0, 255), -1)
                           
                        except:
                            cam_cmd.angular.z = 0;
                            final_cmd = cam_cmd
                            pub.publish(final_cmd)
                            print "bbbbbbbbbbbbbbbbbbbbbbbbbbb"
                            continue
                        
                        
                        #aveX = np.average(pixelX)
                                                

                        minY = np.min(pixelY)
                        #通过计算截距区分左右车道线
                        if (2*a2*aveX + a1) > 0 : #斜率大于0
                            if a2 > 0:
                                x_intertcept = (-a1+(abs(a1*a1-4*a2*(a0 - binary_warped.shape[0] - (D - I) / y_cmPerPixel))**0.5))/(2*a2) #求截距
                            else :
                                x_intertcept = (-a1-(abs(a1*a1-4*a2*(a0 - binary_warped.shape[0] - (D - I) / y_cmPerPixel))**0.5))/(2*a2)
                                    
                        else : #斜率小于0
                            if a2 > 0:
                                x_intertcept = (-a1-(abs(a1*a1-4*a2*(a0 - binary_warped.shape[0] -(D - I) / y_cmPerPixel))**0.5))/(2*a2) 
                            else :
                                x_intertcept = (-a1+(abs(a1*a1-4*a2*(a0 - binary_warped.shape[0] -(D - I) / y_cmPerPixel))**0.5))/(2*a2)

                        if (x_intertcept > binary_warped.shape[1]/2.0):
                            LorR = -1; #RightLane
                            print('R')
                            
                        else:
                            LorR = 1; #LeftLane
                            print('L')  

                                         


                        curvatureRadius_aim = 450.0/x_cmPerPixel
                        curvature_aim = 1/curvatureRadius_aim
                            
                        curvature_real = min(curvature_aim, 2*a2 - 0.000001)
                        print 'c: %.4f  2a2: %.4f'%(curvature_real, 2*a2)


                        if aveX < -a1/(2*a2):

                            curvature_x = (-((2*a2/curvature_real)**(2/3.0) - 1)**0.5 - a1)/(2*a2)
                        else:
                            curvature_x = (+((2*a2/curvature_real)**(2/3.0) - 1)**0.5 - a1)/(2*a2)

                        if curvature_x != curvature_x:
                                
                            print 'NaN'
                            pass
                        else:
                            cv2.line(binary_warped, (int(curvature_x), 0), (int(curvature_x) , binary_warped.shape[0]), (0, 0, 255), 3)


                        aimLanePx = curvature_x
                        aimLanePy = a2*(curvature_x**2) + a1*curvature_x + a0
                        aimLaneP = [aimLanePx, aimLanePy]

                        if aimLanePy > binary_warped.shape[0]:
                            aimLanePy = binary_warped.shape[0]

                            delta = a1**2 - 4*a2*(a0 - aimLanePy)
                            if delta > 0:
                                delta = delta**0.5
                                if aveX < -a1/(2*a2):
                                    print "LL"
                                    aimLanePx = (-a1 - delta)/(2*a2)
                                else:
                                    print "RR"
                                    aimLanePx = (-a1 + delta)/(2*a2)
                                aimLaneP = [aimLanePx, aimLanePy]

                        if aimLanePy < minY:
                            aimLanePy = minY

                            delta = a1**2 - 4*a2*(a0 - aimLanePy)
                            if delta > 0:
                                delta = delta**0.5
                                if aveX < -a1/(2*a2):
                                    print "LL"
                                    aimLanePx = (-a1 - delta)/(2*a2)
                                else:
                                    print "RR"
                                    aimLanePx = (-a1 + delta)/(2*a2)
                                aimLaneP = [aimLanePx, aimLanePy]



                                   
                                    
           
                        #计算aimLaneP处斜率，从而得到目标点的像素坐标
                        lanePk = 2*a2*aimLaneP[0] + a1
                        if (lanePk > 500 or lanePk < -500):
                            aP[0] = aimLaneP[0] + LorR*roadWidth/2
                            aP[1] = aimLaneP[1]

                        else:
                            k_ver = -1/lanePk
                            theta = math.atan(k_ver)
                            aP[0] = aimLaneP[0] + math.cos(theta)*(LorR)*roadWidth/2
                            aP[1] = aimLaneP[1] + math.sin(theta)*(LorR)*roadWidth/2


                        
                    predictedCoords = kfObj.Estimate(aP[0], aP[1])
                    aP_kf[0] = predictedCoords[0][0]
                    aP_kf[1] = predictedCoords[1][0]

                    Y_half = np.argsort(pixelY)[int(len(pixelY)/2)]
                    X_half = np.argsort(pixelX)[int(len(pixelY)/2)]
                    
                    aP_kf[0] = pixelX[X_half]+LorR*200
                    aP_kf[1] = pixelY[Y_half]+LorR*50

                    #aP[0] = aP_kf[0]
                    #aP[1] = aP_kf[1]


                    if aP[0] != aP[0] or aP[1] != aP[1]:
                        continue
                   
                    cv2.circle(binary_warped, (int(aP[0]), int(aP[1])), 20, (0, 0, 255), -1)
                    cv2.circle(binary_warped, (int(aP_kf[0]), int(aP_kf[1])), 20, (0, 255, 0), -1)

                    

                    #计算目标点的真实坐标
                    x_cmPerPixel_slope = 80/300.0
                    y_cmPerPixel_slope = 80/1080.0
                    aP[0] = (aP[0] - binary_warped.shape[1]/2.0)*x_cmPerPixel_slope
                    aP[1] = (binary_warped.shape[0] - aP[1])*y_cmPerPixel_slope
                    aP[0]-= 0.1  ##                    

                    '''
                    if(lastP[0] > 0.001 and lastP[1] > 0.001):
                        if(((aP[0]-lastP[0])**2 + (aP[1]-lastP[1])**2 > 1500) and Timer < 4 ): #To avoid the mislead by walkers
                            aP = lastP
                            Timer = Timer + 1
                        else:
                            Timer = 0
                    '''

                    lastP = aP 
                    steerAngle = math.atan(2*I*aP[0]/(aP[0]*aP[0]+(aP[1]+D)*(aP[1]+D)));
                    if steerAngle < 0:
                        cam_cmd.angular.z = k1*steerAngle
                    else:
                        cam_cmd.angular.z = k2*steerAngle

                    print 'steerAngle: %.5f'%(steerAngle)
                    #print(k*steerAngle*180/3.14)
                    
                    if stop_judge_local == True:
                        cam_cmd.linear.x = 0.0
                    else:
                        cam_cmd.linear.x = 0.2
                    
                    print("linear111111111111")
                    #rospy.spinOnce()

                    if judge_end_tunnel == 0 and math.fabs(cam_cmd.angular.z) > 0.13:
                        cam_cmd.angular.z = 0.0
                      
                    final_cmd = cam_cmd
                    pub.publish(final_cmd)
                    self.pubI.publish(self.cvb.cv2_to_imgmsg(binary_warped))
                    c2 = cv2.getTickCount()
                    print 'time'
                    print (c2 - c1)/cv2.getTickFrequency()
                    
                else:
                    pub.publish(final_cmd)
                        
            self.rate.sleep()

        self.cap.release()

def RLcallback(light_msg):
    global stop_judge_local
    global ramp_control
    global part_A
    global lidarLaunch
    if light_msg.show_time >1 and light_msg.red_or_green ==1:
        ramp_control = True

##    if light_msg.red_or_green ==1 and light_msg.show_time ==1:
##        part_A = 1

    if light_msg.red_or_green ==2:
        stop_judge_local = True
    else:
        stop_judge_local = False



def LScallback(laser_cmd):
    global stop_judge_local, lidarLaunch
    global final_cmd
    global judge_end_tunnel
    judge_end_tunnel =  laser_cmd.linear.y
    if stop_judge_local == True:
        laser_cmd.linear.x = 0
        final_cmd = laser_cmd

    if laser_cmd.linear.z >= 0:  #laser control steer 
        lidarLaunch = True 
        final_cmd = laser_cmd
          
    else:
        lidarLaunch = False
        
        pass
    
        

if __name__ == '__main__':
    #device = rospy.get_param('device', 1)
    #width = rospy.get_param('width', 1280)
    #height = rospy.get_param('height', 720)
    #rates = rospy.get_param('rates', 10)
    global blind_detected
    blind_detected = False
    global blind_detection_flag
    blind_detection_flag = False
    global delay_flag
    delay_flag = False
    global ramp_control
    ramp_control = False
    global part_A
    part_A = 0
    global judge_end_tunnel 
    judge_end_tunnel = 0
    
    initial_parameters()
    #距离映射
    x_cmPerPixel = 80/400.0
    y_cmPerPixel = 80/400.0
    roadWidth = 65.0 / x_cmPerPixel #80.0 
    


    aP = [0.0, 0.0]
    lastP = [0.0, 0.0]
    aP_kf=[0.0,0.0]
    Timer = 0

    #轴间距
    I = 61.5
    #图像坐标系底部与车后轮轴心间距 #cm
    D = 126
    #计算cmdSteer的系数，舵机转向与之前相反，此处用正数
    k1 = 3.6		 # 3.6
    k2 = 3.6
    #steerAngle, cmdSteer;
    global final_cmd, cam_cmd #laser_cmd = Twist()
    final_cmd = Twist()
    
    cam_cmd = Twist()
    ##cam_cmd.linear.x = 0.2
    
    global stop_judge_local
    stop_judge_local = False
    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    try:
        cmd = command()
        cmd.spin()
        rospy.spin()
        final_cmd = cam_cmd
        

    except rospy.ROSInterruptException:
        pass
