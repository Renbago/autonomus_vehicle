#!/usr/bin/env python3

# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import json
from pynput import keyboard
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError


class CarControl():
    # ===================================== INIT==========================================
    def __init__(self):
        """Run on the PC. It forwards the commans from the user via KeboardListenerThread to the RcBrainThread. 
        The RcBrainThread converts them into actual commands and sends them to the remote via a socket connection.
        
        """
        self.bridge = CvBridge()

        self.dirKeys   = ['w', 'a', 's', 'd']
        self.paramKeys = ['t','g','y','h','u','j','i','k', 'r', 'p']
        self.pidKeys = ['z','x','v','b','n','m']

        self.allKeys = self.dirKeys + self.paramKeys + self.pidKeys
        

        rospy.init_node('EXAMPLEnode', anonymous=False)     
        #self.carControl = rospy.Publisher(steeringAngle, String, queue_size=1)
        self.imageTopic = rospy.Subscriber('/automobile/camera_output', Image, self.imageTopicCallback)

        
        self.increment = 0
        #self.twist_pub_timer = rospy.Timer(rospy.Duration(1), self.twist_pub_timer_callback)     

    def imageTopicCallback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("Image window", cv_image)
        print(cv_image.shape)
    # def twist_pub_timer_callback(self,event):
        
    #     # YOLO topic publish deneme
    #     # self.yolo.object = ["Cenk","Muhammed"]
    #     # self.yolo_pub.publish(self.yolo)
        
    #     data = {}
    #     data = {
    #         'action': '2',
    #         'steerAngle': float(30.0)   
    #     }
    #     self.data = json.dumps(data)
    #     self.carControl.publish(self.data)
    #     rospy.sleep(0.1)

    #     # İkinci mesaj için veriler
    #     data = {
    #         'action': '1',
    #         'speed': float(0.3)  
    #     }   
    #     self.data = json.dumps(data)
    #     self.carControl.publish(self.data)

    #     # self._send_command(self.array[self.increment])
    #     # self.increment += 1
    #     # if self.increment == 4:
    #     #     self.increment = 0

    # # ===================================== RUN ==========================================
    # def run(self):
    #     """Apply initializing methods and start the threads. 
    #     """
    #     with keyboard.Listener(on_press = self.keyPress, on_release = self.keyRelease) as listener: 
    #         listener.join()
	
    # # ===================================== KEY PRESS ====================================
    # def keyPress(self,key):
    #     pass
        
    # # ===================================== KEY RELEASE ==================================
    # def keyRelease(self, key):
    #     pass                                                             
                 
    # ===================================== SEND COMMAND =================================
    # def _send_command(self, key):
    #     """Transmite the command to the remotecontrol receiver. 
        
    #     Parameters
    #     ----------
    #     inP : Pipe
    #         Input pipe. 
    #     """
    #     command = self.rcBrain.getMessage(key)
    #     if command is not None:
	
    #         #command = json.dumps(command)
    #         #self.publisher.publish(command)  


if __name__ == '__main__':
    try:
        nod = CarControl()
        rospy.spin()
    except rospy.ROSInterruptException or KeyboardInterrupt:
        rospy.signal_shutdown
        
        pass


# def main():
#     rospy.init_node('lane_tracker', anonymous=True)
#     lane_tracker = CarControl(steeringAngle='/automobile/command',
#                                 outputimage_topic_name = '/automobile/camera_output',)

#     rate = rospy.Rate(30) # 10Hz
#     try:
#         while not rospy.is_shutdown():
#             # Sürekli çalışacak kodlar burada
#             rate.sleep()
#     except KeyboardInterrupt:

#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()
