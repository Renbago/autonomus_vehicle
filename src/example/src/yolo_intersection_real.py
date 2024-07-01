#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
#import supervision as sv
import time
from ultralytics import YOLO
import torch
import serial
#import pyzed.sl as sl
import sys

from example.msg import MekatronomYolo


class yolo():
    def __init__(self):
        
        self.stop_flag = False
        self.stop_flag2 = False
        self.mesafe = 15

        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.image_callback)

        #Subscribers
        self.image_sub = rospy.Subscriber("/zed2i/zed_node/rgb/image_rect_color",Image,self.image_callback)
        self.depth_sub = rospy.Subscriber("/zed2i/zed_node/depth/depth_registered",Image,self.depth_callback)
        #Publishers
        self.yoloPub = rospy.Publisher("/automobile/yolo", MekatronomYolo, queue_size=1)
        
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print("Using Device: ", self.device)
    
        self.model  = YOLO("/home/mekatronom/boschmekatronom_ws/BoschMekatronom/src/weights/boschM100Epoch.pt")
        self.image_callback_flag = False
        rospy.Timer(rospy.Duration(0.5), self.set_callback)
        
        self.control=0
        self.class_name = ""
        
        #self.intersection_callback(self.frame.copy())


    def plot_detections(self,frame, class_name, top, left, bottom, right):
        # Draw rectangle around the detection
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
        
        # Prepare label text with class name and confidence
        label_text = f'{class_name}'
        
        # Put label text on the image
        cv2.putText(frame, label_text, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return frame

    def yolo_callback(self):
        
        self.yolomessage = MekatronomYolo()

        model_input_size = (360, 640)  # Example: (height, width) that model expects
        
        scale_x = self.frame.shape[1] / model_input_size[1]
        scale_y = self.frame.shape[0] / model_input_size[0]
        # print("Scaling factors:", scale_x, scale_y)
        # Initialize frame_with_detections with the original frame
        # Ensures there is always a frame to display even if no detections occur
        frame_with_detections = self.frame.copy()
        # print("test")
        results = self.model(self.frame)
        # print("test")
        for r in results:
            boxes = r.boxes
            # print("test")
            for box in boxes:
                # print("test")
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                self.class_name = self.model.names[int(c)]
                top = int(b[1] * scale_y)
                left = int(b[0] * scale_x)
                bottom = int(b[3] * scale_y)
                right = int(b[2] * scale_x)
                label_text = f'{self.class_name}'

                self.yolomessage.object.append(label_text)
                rospy.loginfo(f"Class: {self.class_name} Top: {top} Left: {left} Bottom: {bottom} Right: {right}")
                # Update frame_with_detections with each detection
                frame_with_detections = self.plot_detections(frame_with_detections, self.class_name, top, left, bottom, right)

        self.intersection_callback(self.frame.copy())
        
        self.yolomessage.distance = [float(self.mesafe)]

        self.yoloPub.publish(self.yolomessage)

        
        
        #cv2.rectangle(frame_with_detections, (right_top_x, right_top_y), (left_top_x, left_top_y),(255,0,255), 2)

        cv2.imshow("frame_with_detections", frame_with_detections)
        cv2.waitKey(1)
    
                                
    def set_callback(self, event):
        if self.image_callback_flag:
            self.yolo_callback()
        

    def image_callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            print("frame shape:", self.frame.shape)
            
            #self.frame = data#pyzedden data resim olarak geliyo  mesaj olarak değil
            self.image_callback_flag = True
            
        except CvBridgeError as e:
            print(e)
            pass

    def depth_callback(self, data):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(data, "32FC1")
            depths = np.frombuffer(data.data, dtype=np.float32).reshape(data.height, data.width)            
            self.depth_callback_flag = True
            # Image coordinates of the center pixel
            u = data.width // 2
            v = data.height // 2
            right_top_x, right_top_y = 125, 50
            left_top_x, left_top_y = 225, 150
            scan_res = 5

            points_x = np.linspace(right_top_x, left_top_x, scan_res)
            points_y = np.linspace(right_top_y, left_top_y, scan_res)
            depth_values = []
            for x in points_x:
                for y in points_y:
                    depth = depths[int(y), int(x)]
                    if 0.3 < depth:
                        depth_values.append(depth)
            
            # Access the depth value of the center pixel
            # center_depth = depths[v, u]

        except CvBridgeError as e:
            # print(e)
            pass


    def intersection_callback(self, frame):
        # print("girdi")
        img = cv2.resize(frame, (640,480))
        gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        hue_min = 0#0
        hue_max = 60#60
        sat_min = 0#0
        sat_max = 255#255
        val_min = 70#70
        val_max = 255#255

        # HSV değerlerine dayalı maske oluşturun
        lower_bound = np.array([hue_min, sat_min, val_min])
        upper_bound = np.array([hue_max, sat_max, val_max])
        # mask = cv2.inRange(hsv_img, lower_bound, upper_bound)

        imgBlur= cv2.GaussianBlur(img, (7, 7), 1)

        imgHSV= cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)

        imgMask = cv2.inRange(gray_image, 200, 255)

        kernel = np.ones((5, 5))
        imgDil = cv2.dilate(imgMask, kernel, iterations=2)

        imgContour = imgDil.copy()
        
        width, height = 640, 480

        bottom_right = [990, 480] # 640, 480
        bottom_left = [-350, 480] # 0, 480
        top_right = [600, 240]
        top_left = [40, 240]

        src = np.float32([top_left, top_right, bottom_left, bottom_right])

        # Dönüşüm sonucu alınacak köşe noktalarını belirleme
        dst = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
        matrix = cv2.getPerspectiveTransform(src, dst)
        imgWarp = cv2.warpPerspective(img, matrix, (width, height))

        imgContourPers = imgWarp.copy()
        gray_image = cv2.cvtColor(imgContourPers, cv2.COLOR_BGR2GRAY)
        imgMaskGray = cv2.inRange(gray_image, 230, 255)
        imgHSVPers = cv2.cvtColor(imgWarp, cv2.COLOR_BGR2HSV)
        imgMaskPers = cv2.inRange(imgHSVPers, lower_bound, upper_bound)

        imgDilPers = cv2.dilate(imgMaskGray, kernel, iterations=2)

        #cv2.imshow("perspective:",imgDilPers)
        kernel2= np.ones((1,20),np.uint8)
        y=10
        x=200
        h=460
        w=240
        crop = imgDilPers[y:y+h, x:x+w]# bu crop u sıfır matrisinde yerine geri koymam lazım 
        # cv2.imshow("croped",crop)
        #bos = np.zeros((480, 640))# bunu içine cropu konumuna insert edicem 
        bos=np.zeros((480, 640, 3), dtype = np.uint8)
        bosx= cv2.cvtColor(bos, cv2.COLOR_BGR2GRAY)
        
        bosx[10:10+460, 200:200+240] = crop
        # cv2.imshow("vvvvvvvvvvvvvv",bosx)
        #roi nin  ayarlanıp kırpılıp yenşiden genişletilmiş hali

        yatayline= cv2.erode(bosx,kernel2,iterations=3)#erode çakıştırma and alarak filitreleme
        
        self.bosluk=np.all(yatayline==0) #bosluk = 0 gelirse duz cizgi detected
        if not self.bosluk:
            
            # print("düz çizgi var ")
            backtorgb = cv2.cvtColor(yatayline,cv2.COLOR_GRAY2RGB)
            backtorgb2 = cv2.cvtColor(imgDilPers,cv2.COLOR_GRAY2RGB)
            # cv2.imshow("bactorgb",backtorgb2)
            lower= np.array([200, 200, 200], dtype = "uint8")
            upper= np.array([255, 255, 255], dtype = "uint8")
            mask = cv2.inRange(backtorgb,lower,upper)
            backtorgb[np.where((backtorgb==[255,255,255]).all(axis=2))] = [0,0,255]
            boyali =cv2.addWeighted(backtorgb2, 0.5, backtorgb, 0.7, 0)
            #255 128 128 BOYA
            #128 128 128 YOL
            # cv2.imshow("rgb  :",backtorgb)
            # cv2.imshow("hedef  :",boyali)

            gren=[0,255,0]
            renk= np.array([0,0,255])
            ust=0
            alt=0

            for i in np.arange(480):
                a=backtorgb[i][325]
                if  np.array_equal(a, renk):
                    backtorgb = cv2.circle(backtorgb, (325,i), 2, gren, -1)
                    #print("buldum ")
                    ust=i
                    break
            for i in np.arange(480):
                reverse=479-i
                #print("i:",reverse)
                a=backtorgb[reverse][325]
                if  np.array_equal(a, renk):
                    backtorgb = cv2.circle(backtorgb, (325,reverse), 2, gren, -1)
                    #print("buldum ")
                    alt=reverse
                    break
            #print("ust :",ust,"alt",alt)
                if alt==ust:
                    ust=ust+5
                self.mesafe=((480-alt)/abs(alt-ust))*4
            # print("mesafe:",self.mesafe)#
            
            # cv2.imshow("rgb  :",backtorgb)
            # cv2.imshow("hedef  :",boyali)
                    


        # cv2.imshow("original:",imgDil)
        # cv2.imshow("kontrol",yatayline)
        #cv2.waitKey(1)
            
def main():
    rospy.init_node('yolo', anonymous=False)
    
    yl = yolo()
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            # Sürekli çalışacak kodlar burada
            rate.sleep()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
