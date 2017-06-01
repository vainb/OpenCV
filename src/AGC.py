# -*- coding:utf-8 -*-

# ***********pyserial********
# https://pyserial.readthedocs.io/en/latest/pyserial_api.html
# https://pyserial.readthedocs.io/en/latest/shortintro.html
# ***************************
import cv2
import numpy as np
import math
import copy
import serial
import time

# ser1:connect to ARM   ->ARM
# ser2:BT->
# ser1 = serial.Serial("/dev/ttyUSB1", 115200)
# ser2 = serial.Serial("/dev/ttyUSB0", 115200)
ser1 = serial.Serial('COM3', 115200, timeout=0.5)
ser2 = serial.Serial('COM5', 115200, timeout=0.5)

def AGC_Control(x,y):
    keepDistance = 30
    correct = 3
    # x /= 10
    # print('startControl')
    tmp = int((x/correct)-5)
    if tmp > 0 and tmp < 10:
        ser1.write("0000"+str(tmp))
        print(tmp)
    else :
        ser1.write("00000")

def insert_sort(lst):
    n=len(lst)
    if n==1: return lst
    for i in range(1,n):
        for j in range(i,0,-1):
            if lst[j]<lst[j-1]: lst[j],lst[j-1]=lst[j-1],lst[j]
    return lst

def pic_y2dist(pic_y):
    dist=math.tan((camera_y-pic_y)/(camera_y-opsite0_y)*opsite0_theta)*high
    return dist

def dist2pic_y(dist):
    pic_y=camera_y-(math.atan(dist/high)/opsite0_theta*(camera_y-opsite0_y))
    return pic_y

# cv2.namedWindow('result')

cap = cv2.VideoCapture(0)
cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,  640)
cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 480)

opsite0, opsite1, high=40, 25, 26.5
opsite0_y, opsite1_y=139, 320
opsite0_x, opsite1_x=149, 96
refer_long=10
adjust_mul=1
adjust_add=0
opsite0_theta= math.atan(opsite0/high)
opsite1_theta= math.atan(opsite1/high)
camera_y=opsite1_y-(-opsite1_theta*(opsite1_y-opsite0_y))/(opsite0_theta-opsite1_theta)
camera_x=320
maxdist=pic_y2dist(0)
mindisk=pic_y2dist(480)
maxline=11
minline=3
maxline_pic=480-dist2pic_y(pic_y2dist(480)+maxline)
minline_pic=dist2pic_y(pic_y2dist(0)-minline)


def OpenCV():
    print "start"

    # Creating a window for later use
    while True:
        count = ser2.inWaiting()
        if count != 0:
            # 读取内容并回显
            recv = ser2.read(4)
            ser1.write(recv+'5')
            print recv
        # 清空接收缓冲区
        ser1.flushInput()
        ser2.flushInput()



        (_, frame) = cap.read()
        # Converting to Gray
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Do gaussian blur
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        gray_show =copy.copy(gray)

        # # Do Threshold-----------------------------corridor-------------------------------------
        # ret,th1 = cv2.threshold(gray,80,255,cv2.THRESH_BINARY_INV)
        # Do adaptiveThreshold----------------------classroom-------------------------------------
        th1 = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
        # Do erode if needed
        th1 = cv2.erode(th1, None, iterations=1)
        # Do dilate if needed
        # th1 = cv2.dilate(th1, None, iterations=1)
        th2 = copy.copy(th1)
        # th3 = copy.copy(th1)
        mask = np.ones(gray.shape,np.uint8)
        mean = cv2.mean(gray,mask = mask)
        # print 'mean=',mean[0]
        # find contours
        (contours, _) = cv2.findContours(th1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # catch the contour of line
        a = int(len(contours))
        for i in range(a):
            cnt = contours[i]
            hull= cv2.convexHull(cnt)
            approx = cv2.approxPolyDP(hull,5,True)
            if int(len(approx)) == 4:
                rect = cv2.minAreaRect(approx)
                box = np.int0(cv2.cv.BoxPoints(rect))
                boxy = insert_sort([box[0][1],box[1][1],box[2][1],box[3][1]])
                if boxy[0] > 10 and boxy[3] < 470:
                    wh = insert_sort([rect[1][0],rect[1][1]])
                    if wh[1] > 600:
                        if wh[0]>minline_pic and wh[0]<maxline_pic:
                            mask_t = np.zeros(gray.shape,np.uint8)
                            cv2.drawContours(mask_t,[cnt],0,255,-1)
                            mean_t = cv2.mean(gray,mask = mask_t)
                            if mean_t > mean:
                                area = cv2.contourArea(approx)
                                if area > wh[1]*wh[0]*0.8:
                                    cv2.drawContours(frame,[cnt],0,(0,255,0),-1)
                                    cv2.drawContours(gray_show,[cnt],0,(0,255,0),3)
                                    cv2.drawContours(th2,[cnt],0,(255,255,255),-1)
                                    # cv2.drawContours(th2,[approx],0,(255,255,255),-1)
                            else :
                                cv2.drawContours(th2,[cnt],0,(0,0,0),-1)
                        else :
                            cv2.drawContours(th2,[cnt],0,(0,0,0),-1)
                    else :
                        cv2.drawContours(th2,[cnt],0,(0,0,0),-1)
                else :
                    cv2.drawContours(th2,[cnt],0,(0,0,0),-1)
            else :
                cv2.drawContours(th2,[cnt],0,(0,0,0),-1)

        # Do erode if needed
        th2 = cv2.erode(th2, None, iterations=1)
        # Do dilate if needed
        th2 = cv2.dilate(th2, None, iterations=1)
        # Do canny
        edges = cv2.Canny(th2,50,150,apertureSize = 3)
        lines = cv2.HoughLines(image=edges,rho=1,theta=np.pi/180,threshold=80)
        # print(lines)


        if np.all(lines):
        # if lines!=None:
            # print(lines.shape[1])
            rightline_theta, rightline_rho, rightlines, rightline_y, rightline_dist= 0, 0, 0, 0, 0
            a = lines.shape[1]
            for i in range(a):
                rho=lines[0][i][0]
                theta=lines[0][i][1]
                #leftline
                if (theta > 60*np.pi/180) and (theta < 120*np.pi/180):
                    if (rho>rightline_rho):
                        rightline_theta=rightline_theta+theta
                        rightline_rho=copy.copy(rho)
                        rightlines=rightlines+1
            if rightlines>0:
                rho=copy.copy(rightline_rho)
                rightline_theta=rightline_theta/rightlines
                theta=copy.copy(rightline_theta)
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                cv2.line(frame,(x1,y1),(x2,y2),(0,0,255),3)
                cv2.line(gray_show,(x1,y1),(x2,y2),(255,255,255),3)
                # print(x0, y0, x1, y1, x2, y2)
                if theta<np.pi/2:
                    rightline_dist=high*math.tan((camera_y-(y0-(320- x0)*math.tan(np.pi/2-theta)))/(opsite1_y-opsite0_y)*(opsite0_theta-opsite1_theta))
                elif theta==np.pi/2:
                    rightline_dist=high*math.tan((camera_y-y0)/(opsite1_y-opsite0_y)*(opsite0_theta-opsite1_theta))
                else:
                    rightline_dist=high*math.tan((camera_y-(y0+(320- x0)*math.tan(theta-np.pi/2)))/(opsite1_y-opsite0_y)*(opsite0_theta-opsite1_theta))

                print 'rightline_dist=',int(rightline_dist),
                print ' rightline_theta=',int(rightline_theta*100)
                AGC_Control(int(rightline_dist),int(rightline_theta*100))
                # print("rightline_dist=", rightline_dist,"rightline_theta=", rightline_theta)
                # print 'rightline_dist=%d ,rightline_theta=%5.3f' %rightline_dist%rightline_theta
            else:
                rightline_dist=None
                rightline_theta=None
                ser1.write("00000")
                # print('rightline_dist=None_dist',rightline_dist)
                # print('rightline_theta=None',rightline_theta)
        else:
            rightline_dist=None
            rightline_theta=None
            ser1.write("00000")
            # ser.write("f0")
            # print('rightline_dist=None_dist',rightline_dist)
            # print('rightline_theta=None',rightline_theta)

        # result = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow("AGC", gray_show)

        if cv2.waitKey(30) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        OpenCV()
    except KeyboardInterrupt:
        if ser != None:
            ser1.close()
            ser2.close()
            # sys.exit(app.exec_())