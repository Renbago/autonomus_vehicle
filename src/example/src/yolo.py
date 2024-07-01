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
import supervision as sv
from time import time
from ultralytics import YOLO
import torch

from example.msg import MekatronomYolo

class yolo():
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/automobile/image_raw",Image,self.image_callback)
        self.yoloPub = rospy.Publisher("/automobile/yolo", MekatronomYolo, queue_size=10)

        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print("Using Device: ", self.device)

        self.model  = YOLO("/home/mekatronom/boschmekatronom_ws/BoschMekatronom/src/weights/topluLevhaM100Epoch.pt")
        self.image_callback_flag = False
        rospy.Timer(rospy.Duration(0.5), self.set_callback)


    def plot_detections(self,frame, class_name, top, left, bottom, right):
        # Draw rectangle around the detection
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
        
        # Prepare label text with class name and confidence
        label_text = f'{class_name}'
        
        # Put label text on the image
        cv2.putText(frame, label_text, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return frame

    def yolo_callback(self):
        model_input_size = (480, 640)  # Example: (height, width) that model expects
        scale_x = self.frame.shape[1] / model_input_size[1]
        scale_y = self.frame.shape[0] / model_input_size[0]
        print("Scaling factors:", scale_x, scale_y)

        # Initialize frame_with_detections with the original frame
        # Ensures there is always a frame to display even if no detections occur
        frame_with_detections = self.frame.copy()

        results = self.model(self.frame)

        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                class_name = self.model.names[int(c)]
                top = int(b[1] * scale_y)
                left = int(b[0] * scale_x)
                bottom = int(b[3] * scale_y)
                right = int(b[2] * scale_x)

                rospy.loginfo(f"Class: {class_name} Top: {top} Left: {left} Bottom: {bottom} Right: {right}")
                # Update frame_with_detections with each detection
                frame_with_detections = self.plot_detections(frame_with_detections, class_name, top, left, bottom, right)

        cv2.imshow('color', frame_with_detections)
        cv2.waitKey(1)

    def set_callback(self, event):

        self.yolo_callback()


    def image_callback(self, data):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_callback_flag = True
            
        except CvBridgeError as e:
            print(e)



def main():
    rospy.init_node('yolo', anonymous=False)
    
    yl = yolo()
    rate = rospy.Rate(3)
    try:
        while not rospy.is_shutdown():
            # Sürekli çalışacak kodlar burada
            rate.sleep()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()