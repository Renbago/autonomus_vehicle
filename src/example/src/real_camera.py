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

import rospy
import cv2
import numpy as np
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
from geometry_msgs.msg import PoseStamped
import serial
import time
#from ultralytics import YOLO


class realCamera():
    # ===================================== INIT==========================================
    def __init__(self,camera_topic_name,pose_topic_name):
        """
        Creates a bridge for converting the image from Gazebo image intro OpenCv image
        """
        
        #bölgelere ilk değeri 0 veriyorum. baslangic
        self.on = 0
        self.onsag = 0
        self.onsol = 0 
        self.sag = 0
        self.sol = 0
        self.onsolengel = 0
        self.onsagengel = 0
        self.speed = 0.2
        self.sure = 0
        self.derece_verisi_counter = 0
        self.engel = 0
        self.orta_cizgi_yakinlik = 0
        self.serit_degisti = 0
        self.carside_cb = True  # Normalde false simdilik True
        self.serit_kontrol = 0
        self.konum = 0
        self.arac_eylem = "yol_temiz"
        self.yol_kapali = 0
        self.lane_type = "sheesh"
        self.eylem = "yol_temiz"

        self.degree_veri = 0
        #self.ser =serial.Serial(baudrate=115200,port='/dev/ttyACM0')
        #bölge ayarlaması bitti
        
        #arac boyut tanımlamaları:
        self.vehicleWidth = 0.6
        self.vehicleLength = 0.28

        # isik tanımlamaları
        self.kirmizi_isik = 0
        self.yesil_isik = 0
        self.durak_giriliyor = 0

        self.iterasyon_degeri=0
        self.yaw_degree = 0
        self.mpc_iter = 0

        #
        self.y_offset = 0
        self.x_offset = 0
        self.depth_x = 0
        self.depth_y = 0
        self.depth_cb_done = False
        self.steering_angle = 0
        self.mavi_cizgi_sol_üst = 0

        self.theta = 0

        self.not_shifted_array = []
        self.shifted_array = []
        self.shifted_array_aviable = False
        self.odometry_cb = False
        self.hedef_x = 0

        self.twist = Twist()     # Gazeboda arabayi sürmek için 
        self.msg = String()

        self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 480))
        self.rgb=rospy.Subscriber(camera_topic_name,Image,self.image_callback)
        self.pose=rospy.Subscriber(pose_topic_name,PoseStamped,self.pose_callback)
        print("realCamera node")
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print(f"Device: {self.device}")


    def image_callback(self, data):

        try:
            start_time = time.time()
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img_width = 640
            img_height = 480
            #img = cv2.resize(img, (img_width, img_height))
            cv2.imshow('color', img)
            # cv2.waitKey(3)
            # cv2.imwrite('imgContourPers.jpg', img)

            # c_thresh1 = 0
            # c_thresh2 = 70
            hue_min = 0
            hue_max = 60
            sat_min = 0
            sat_max = 255
            val_min = 70
            val_max = 255

            # Görüntüyü HSV renk uzayına dönüştürün
            # hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # HSV değerlerine dayalı maske oluşturun
            lower_bound = np.array([hue_min, sat_min, val_min])
            upper_bound = np.array([hue_max, sat_max, val_max])
            # mask = cv2.inRange(hsv_img, lower_bound, upper_bound)

            imgBlur= cv2.GaussianBlur(img, (7, 7), 1)

            imgHSV= cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)

            imgMask = cv2.inRange(gray_image, 200, 255)
            #cv2.imshow('mask', imgMask)

            kernel = np.ones((5, 5))
            imgDil = cv2.dilate(imgMask, kernel, iterations=2)

            imgContour = imgDil.copy()

            contours, hierarchy = cv2.findContours(imgDil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            cv2.drawContours(imgContour, contours, -1, (255, 0, 255), 3)

            # cv2.imshow("imgcounterLeft", imgContourLeft)
            # cv2.imshow("imgcounterRight", imgContourRight)

            # for cntLeft in contoursLeft:
            #     areaLeft = cv2.contourArea(cntLeft)
            #     if areaLeft > 1000:  # DÜZENLENEBİLİR - Burada sadece alanı 1000 birim
            #         periLeft = cv2.arcLength(cntLeft, True)
            #         approxLeft = cv2.approxPolyDP(cntLeft, 0.02 * periLeft, True)
            #         xLeft, yLeft, wLeft, hLeft = cv2.boundingRect(approxLeft)
            #         aLeft = int(wLeft / 2 + xLeft)
            #         bLeft = int(hLeft / 2 + yLeft)
            #         cv2.rectangle(imgContourLeft, (xLeft, yLeft), (xLeft + wLeft, yLeft + hLeft), (0, 255, 0), 5)
            #         cv2.rectangle(imgContourLeft, (aLeft, yLeft), (aLeft, yLeft + hLeft), (255, 0, 0), 2)
            #         cv2.line(imgContourLeft, (aLeft, bLeft), (640, 27), (255, 255, 0), 1)
            #         # distanceLeft = int(640 - aLeft)
            #         # solSeritDurum = "OK"
            #     # else: solSeritDurum = "KAYIP"

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
            contoursPers, hierarchyPers = cv2.findContours(imgDilPers, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cv2.drawContours(imgContourPers, contoursPers, -1, (255, 0, 255), 1)


            # 640 x 360 için
            pixel_val = np.arange(25, (img_height - 25), 25)

            # 1280 x 720 için
            # pixel_val = np.arange(25, (img_height - 25), 25)
            def calculate_steering_angle_new(L, R, T):
                #L arac uzunlugu
                #R curve yaricapi
                #T tekerler arasi mesafe

                steering_angle_in = math.atan(L/(R-(0.5*T)))
                steering_angle_out = math.atan(L/(R+(0.5*T)))

                print(f"Steering angle in {math.degrees(steering_angle_in)}")
                print(f"Steering angle out {steering_angle_out}\n")

                return steering_angle_in, steering_angle_out

            def calculate_steering_angle_new_oldstyle(lx, rx, ly, ry):
                mid_y_left = sum(ly)/len(ly)
                mid_x_left = sum(lx)/len(lx)

                mid_y_right = sum(ry) / len(ry)
                mid_x_right = sum(rx) / len(rx)

                mid_steering_y = (mid_y_left + mid_y_right) / 2
                mid_steering_x = (mid_x_left + mid_x_right) / 2
                steering_point = [mid_steering_x, mid_steering_y]
                print(f"Steering Point {steering_point}")
                bottom_mid = [320, 480]
                # cv2.line(merged_img, steering_point, bottom_mid, color=(0, 255, 0), thickness=2)
                steering_angle = math.atan((steering_point[0] - bottom_mid[0]) / (steering_point[1] - bottom_mid[1]))
                steering_angle = 90 - (90 - math.degrees(steering_angle))
                self.steering_angle = steering_angle
                green_text = "\033[92m"  # green
                reset_text = "\033[0m"  # reset to default color
                #print(f"{green_text}Steering Angle {self.steering_angle}{reset_text}")
                # 180 - (90 - math.degrees(steering_angle))

                steering_angle_limit = 20
                if self.steering_angle > steering_angle_limit:
                    self.steering_angle = steering_angle_limit
                elif self.steering_angle < -steering_angle_limit:
                    self.steering_angle = -steering_angle_limit

                #print(f"Steering angle old style {self.steering_angle}")

                return steering_point
            
            def get_poly_points(left_fit, right_fit):
                '''
                Get the points for the left lane/ right lane defined by the polynomial coeff's 'left_fit'
                and 'right_fit'
                :param left_fit (ndarray): Coefficients for the polynomial that defines the left lane line
                :param right_fit (ndarray): Coefficients for the polynomial that defines the right lane line
                : return (Tuple(ndarray, ndarray, ndarray, ndarray)): x-y coordinates for the left and right lane lines
                '''
                ysize, xsize = 480, 640

                # Get the points for the entire height of the image
                plot_y = np.linspace(0, ysize - 1, ysize)
                plot_xleft = left_fit[0] * plot_y ** 2 + left_fit[1] * plot_y + left_fit[2]
                plot_xright = right_fit[0] * plot_y ** 2 + right_fit[1] * plot_y + right_fit[2]

                # But keep only those points that lie within the image
                plot_xleft = plot_xleft[(plot_xleft >= 0) & (plot_xleft <= xsize - 1)]
                plot_xright = plot_xright[(plot_xright >= 0) & (plot_xright <= xsize - 1)]
                plot_yleft = np.linspace(ysize - len(plot_xleft), ysize - 1, len(plot_xleft))
                plot_yright = np.linspace(ysize - len(plot_xright), ysize - 1, len(plot_xright))

                return plot_xleft.astype(int), plot_yleft.astype(int), plot_xright.astype(int), plot_yright.astype(
                    int)

            def compute_offset_from_center(poly_param, x_mppx):
                '''
                Computes the offset of the car from the center of the detected lane lines
                :param poly_param (ndarray): Set of 2nd order polynomial coefficients that represent the detected lane lines
                :param x_mppx (float32): metres/pixel in the x-direction
                :return (float32): Offset
                '''
                plot_xleft, plot_yleft, plot_xright, plot_yright = get_poly_points(poly_param[0], poly_param[1])

                lane_center = (plot_xright[-1] + plot_xleft[-1]) / 2
                car_center = 640 / 2

                offset = (lane_center - car_center) * x_mppx
                return offset

            def compute_curvature(poly_param, y_mppx, x_mppx):
                '''
                Computes the curvature of the lane lines (in metres)
                :param poly_param (ndarray): Set of 2nd order polynomial coefficients that represent the detected lane lines
                :param y_mppx (float32): metres/pixel in the y-direction
                :param x_mppx (float32): metres/pixel in the x-direction
                :return (float32): Curvature (in metres)
                '''
                plot_xleft, plot_yleft, plot_xright, plot_yright = get_poly_points(poly_param[0], poly_param[1])

                y_eval = np.max(plot_yleft)

                left_fit_cr = np.polyfit(plot_yleft * y_mppx, plot_xleft * x_mppx, 2)
                right_fit_cr = np.polyfit(plot_yright * y_mppx, plot_xright * x_mppx, 2)

                left_curverad = ((1 + (2 * left_fit_cr[0] * y_eval * y_mppx + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
                    2 * left_fit_cr[0])
                right_curverad = ((1 + (
                            2 * right_fit_cr[0] * y_eval * y_mppx + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
                    2 * right_fit_cr[0])

                return left_curverad, right_curverad


            def fit_polynom(mask, lx, rx, ly, ry):
                # Sol ve sağ noktaları al
                points_left = [(lx[i], ly[i]) for i in range(min(len(lx), len(ly)))]
                points_right = [(rx[i], ry[i]) for i in range(min(len(rx), len(ry)))]

                # Sol taraf için polinom uydur
                left_x = np.array([point[0] for point in points_left])
                left_y = np.array([point[1] for point in points_left])
                left_poly = np.polyfit(left_y, left_x, 2)

                # Sağ taraf için polinom uydur
                right_x = np.array([point[0] for point in points_right])
                right_y = np.array([point[1] for point in points_right])
                right_poly = np.polyfit(right_y, right_x, 2)

                # Türev hesapla
                left_turev = np.polyder(left_poly)
                right_turev = np.polyder(right_poly)

                x_point = 200  # Örneğin x = 2 noktasındaki eğimi bulalım
                slope_at_x_left = np.polyval(left_turev, x_point)
                slope_at_x_right = np.polyval(right_turev, x_point)

                #print("Yolun Eğimi\n")
                #print(f"sol_türev {left_turev}")
                #print(f"sag_türev {right_turev}\n")
                #print(f"x = {x_point} noktasında eğim sol {slope_at_x_left}")
                #print(f"x = {x_point} noktasında eğim sag {slope_at_x_right}\n")

                left_curverad, right_curverad = compute_curvature([left_poly, right_poly], y_mppx=0.002, x_mppx=0.002)
                #print(f"Left Curve {left_curverad}")
                #print(f"Right Curve {right_curverad}")


                # Mask üzerine eğrileri çiz
                for y in range(mask.shape[0]):
                    left_x = int(np.polyval(left_poly, y))
                    right_x = int(np.polyval(right_poly, y))
                    cv2.circle(mask, (left_x, y), 2, (255, 0, 0), -1)  # Sol şerit
                    cv2.circle(mask, (right_x, y), 2, (0, 0, 255), -1)  # Sağ şerit

                    # # Sol köşenin eğimini göster
                    # if 0 < y < mask.shape[0] - 1:
                    #     left_gradient = left_turev[y]
                    #     if abs(left_gradient) > 0.1:  # Eğim eşik değeri
                    #         cv2.putText(mask, f'{left_gradient:.2f}', (left_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    #                     (0, 255, 0), 1, cv2.LINE_AA)
                    #
                    #     # Sağ köşenin eğimini göster
                    #     right_gradient = right_turev[y]
                    #     if abs(right_gradient) > 0.1:  # Eğim eşik değeri
                    #         cv2.putText(mask, f'{right_gradient:.2f}', (right_x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    #                     (0, 255, 0), 1, cv2.LINE_AA)


                return mask, left_poly, right_poly, left_curverad, right_curverad

            def draw(mask, poly_param, curve_rad, offset):
                '''
                Utility function to draw the lane boundaries and numerical estimation of lane curvature and vehicle position.
                :param mask (ndarray): Original image
                :param poly_param (ndarray): Set of 2nd order polynomial coefficients that represent the detected lane lines
                :param curve_rad (float32): Lane line curvature
                :param offset (float32): Car offset
                :return (ndarray): Image with visual display
                '''

                left_fit = poly_param[0]
                right_fit = poly_param[1]
                plot_xleft, plot_yleft, plot_xright, plot_yright = get_poly_points(left_fit, right_fit)

                pts_left = np.array([np.transpose(np.vstack([plot_xleft, plot_yleft]))])
                pts_right = np.array([np.flipud(np.transpose(np.vstack([plot_xright, plot_yright])))])

                # Write data on the image
                if (left_fit[1] + right_fit[1]) / 2 > 0.05:
                    text = 'Left turn, curve radius: {:04.2f} m'.format(curve_rad)
                elif (left_fit[1] + right_fit[1]) / 2 < -0.05:
                    text = 'Right turn, curve radius: {:04.2f} m'.format(curve_rad)
                else:
                    text = 'Straight'

                cv2.putText(mask, text, (50, 60), cv2.FONT_HERSHEY_DUPLEX, 1.2, (255, 255, 255), 2, cv2.LINE_AA)

                direction = ''
                if offset > 0:
                    direction = 'left'
                elif offset < 0:
                    direction = 'right'

                text = '{:0.1f} cm {} of center'.format(abs(offset) * 100, direction)
                cv2.putText(mask, text, (50, 110), cv2.FONT_HERSHEY_DUPLEX, 1.2, (255, 255, 255), 2, cv2.LINE_AA)

                return mask


            def sliding_windows(mask, image_org):
                # Histogram
                histogram = np.sum(mask[mask.shape[0] // 2:, :], axis=0)
                midpoint = int(histogram.shape[0] / 2)
                left_base = np.argmax(histogram[:midpoint])
                right_base = np.argmax(histogram[midpoint:]) + midpoint

                # Sliding Window
                y = 472
                lx = []
                rx = []
                ly = []
                ry = []

                msk = mask.copy()

                while y > 0:
                    ## Left threshold
                    img = mask[y - 40:y, left_base - 50:left_base + 50]
                    contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    for contour in contours:
                        M = cv2.moments(contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            lx.append(left_base - 50 + cx)
                            ly.append(y)
                            left_base = left_base - 50 + cx

                    ## Right threshold
                    img = mask[y - 40:y, right_base - 50:right_base + 50]
                    contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    for contour in contours:
                        M = cv2.moments(contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            rx.append(right_base - 50 + cx)
                            ry.append(y)
                            right_base = right_base - 50 + cx

                    cv2.rectangle(image_org, (left_base - 50, y), (left_base + 50, y - 40), (0, 255, 255), 2)
                    cv2.rectangle(image_org, (right_base - 50, y), (right_base + 50, y - 40), (0, 255, 255), 2)
                    y -= 40
                return msk, lx, rx, ly, ry

            def calculate_steering_angle(con_right,
                                        con_left):  # Sag Taraf Conturlerini ve sol taraf conturlerini alarak steering angle hesaplar
                steering_line_threshold = 250  # Yeşil nokta sınır
                #print(f"Con Right {con_right}\n Con Left {con_left}")
                filt_con_right = {key: value for key, value in con_right.items() if
                                key > steering_line_threshold}  # Piksel değeri y'de 250 (260-270-280 gibi) altında olanları almak
                filt_con_left = {key: value for key, value in con_left.items() if key > steering_line_threshold}

                min_x_right = {}
                max_x_left = {}

                for key in filt_con_right.keys():
                    if len(filt_con_right[key]) > 0:
                        min_x_right[key] = min(filt_con_right[key])

                # filt_min_x_right = {key: value for key, value in min_x_right.items() if value < img_width/2 + x_axis_offset} # X ekseninde offset ile tam ortaya steering line çekilmesini engelliyorum

                for key in filt_con_left.keys():
                    if len(filt_con_left[key]) > 0:
                        max_x_left[key] = max(filt_con_left[key])

                # filt_max_x_left = {key: value for key, value in max_x_left.items() if value > img_width/2 - x_axis_offset}

                #print(f"Min X Right {min_x_right}\n Max X Left {max_x_left}")

                no_lines_left = False
                no_lines_right = False

                if len(list(max_x_left.keys())) > 0:
                    sol_taraf_max_y = min(list(max_x_left.keys()))
                    sol_taraf_max_x = max_x_left[sol_taraf_max_y]
                else:
                   # print("Sol Taraf Max X Yok")
                    no_lines_left = True
                    sol_taraf_max_y = 300
                    sol_taraf_max_x = 0
                if len(list(min_x_right.keys())) > 0:
                    sag_taraf_max_y = min(list(min_x_right.keys()))
                    left_offset = img_width / 2
                    sag_taraf_max_x = min_x_right[sag_taraf_max_y] + left_offset
                else:
                    #print("Sağ Taraf Max X Yok")
                    no_lines_right = True
                    sag_taraf_max_y = 300
                    sag_taraf_max_x = 640

                if sol_taraf_max_y > sag_taraf_max_y or no_lines_left:
                    if no_lines_left:
                        steering_point = [0, steering_line_threshold]
                    else:
                        steering_point = [int((sol_taraf_max_x + sag_taraf_max_x) / 2),
                                        int((sol_taraf_max_y + sag_taraf_max_y) / 2)]
                    #print(f"Sola dön - Sol taraf max_y = {sol_taraf_max_y} - Sağ Taraf max_y = {sag_taraf_max_y}")
                    #print(f"Sola dön - Sol taraf max_x = {sol_taraf_max_x} - Sağ Taraf max_x = {sag_taraf_max_x}")
                elif sol_taraf_max_y < sag_taraf_max_y or no_lines_right:
                    if no_lines_right:
                        steering_point = [640, steering_line_threshold]
                    else:
                        steering_point = [int((sol_taraf_max_x + sag_taraf_max_x) / 2),
                                        int((sol_taraf_max_y + sag_taraf_max_y) / 2)]
                    # print(f"Sağa dön - Sol taraf max_y = {sol_taraf_max_y} - Sağ Taraf max_y = {sag_taraf_max_y}")
                    # print(f"Sağa dön - Sol taraf max_x = {sol_taraf_max_x} - Sağ Taraf max_x = {sag_taraf_max_x}")
                else:
                    # print(f"Düz git - Sol taraf max_y = {sol_taraf_max_y} - Sağ Taraf max_y = {sag_taraf_max_y}")
                    # print(f"Düz git - Sol taraf max_x = {sol_taraf_max_x} - Sağ Taraf max_x = {sag_taraf_max_x}")
                    steering_point = [int((sol_taraf_max_x + sag_taraf_max_x) / 2),
                                    int((sol_taraf_max_y + sag_taraf_max_y) / 2)]

                print(f"Steering Point {steering_point}")
                bottom_mid = [320, 480]
                #cv2.line(merged_img, steering_point, bottom_mid, color=(0, 255, 0), thickness=2)
                steering_angle = math.atan((steering_point[0] - bottom_mid[0]) / (steering_point[1] - bottom_mid[1]))
                steering_angle = 90 - (90 - math.degrees(steering_angle))
                self.steering_angle = steering_angle
                green_text = "\033[92m"  # green
                reset_text = "\033[0m"  # reset to default color
                print(f"{green_text}Steering Angle {self.steering_angle}{reset_text}")
                # 180 - (90 - math.degrees(steering_angle))

                steering_angle_limit = 20
                if self.steering_angle > steering_angle_limit:
                    self.steering_angle = steering_angle_limit
                elif self.steering_angle < -steering_angle_limit:
                    self.steering_angle = -steering_angle_limit

                steering_angle_error_th = 3
                for prev_steer_angle in self.collective_steering_angle:
                    if abs(prev_steer_angle - self.steering_angle) < steering_angle_error_th:
                        steering_angle = self.collective_steering_angle[-1]

                # 30 kere aynı değer geldiyse dön artık yeter be adam
                for prev_steer_angle in self.collective_steering_angle:
                    if prev_steer_angle != steering_angle:
                        break

                if len(self.collective_steering_angle) < 30:
                    self.collective_steering_angle.append(self.steering_angle)
                else:
                    collective_steering_angle = [0]

                # self.twist.angular.z = float(math.degrees(self.steering_angle))
                # self.twist.linear.x = float(0.2)

            def draw_mean_circle(grup_x_given, img):
                points_green = []
                for key in grup_x_given.keys():
                    if grup_x_given[key] == []:
                        continue
                    # print(grup_x_given[key])
                    x1x2 = grup_x_given[key]
                    # print(f"x1x2 Tipi = {type(x1x2)}")
                    x_mean = int((x1x2[0] + x1x2[1]) / 2)
                    cv2.circle(img, (x_mean, key), 1, (0, 255, 0), thickness=2)
                    points_green.append((x_mean, key))
                return points_green

            def draw_lines_with_pixels(points, img):
                # En az iki nokta gereklidir
                if len(points) < 2:
                    print("En az iki nokta gereklidir.")
                    return [], img

                line_pixels = []  # Tüm çizgi pikselleri burada saklanacak

                # İlk iki noktayı al
                p1 = points[0]
                p2 = points[1]

                # İlk iki noktadan bir çizgi çiz
                cv2.line(img, p1, p2, (255, 0, 0), thickness=2)

                # İlk iki nokta arasındaki pikselleri ekle
                line_pixels.extend(get_line_pixels(p1, p2))

                # Kalan noktaları işleme
                for i in range(2, len(points)):
                    # Geçerli noktayı al
                    p = points[i]

                    # Son noktadan çizgi çek
                    cv2.line(img, p2, p, (255, 0, 0), thickness=2)

                    # Çizginin piksellerini ekle
                    line_pixels.extend(get_line_pixels(p2, p))

                    # Sonraki çizgi için p2'yi güncelle
                    p2 = p

                # İşlenmiş görüntüyü ve çizgi piksellerini döndür
                return line_pixels

            def get_line_pixels(p1, p2):
                line_pixels = []  # Çizginin piksellerini burada saklanacak
                x1, y1 = p1
                x2, y2 = p2
                dx = abs(x2 - x1)
                dy = abs(y2 - y1)
                x, y = x1, y1
                sx = -1 if x1 > x2 else 1
                sy = -1 if y1 > y2 else 1

                if dx > dy:
                    err = dx / 2.0
                    while x != x2:
                        line_pixels.append((x, y))
                        err -= dy
                        if err < 0:
                            y += sy
                            err += dx
                        x += sx
                else:
                    err = dy / 2.0
                    while y != y2:
                        line_pixels.append((x, y))
                        err -= dx
                        if err < 0:
                            x += sx
                            err += dy
                        y += sy
                line_pixels.append((x, y))
                return line_pixels

            img_sliding_window, lx, rx, ly, ry = sliding_windows(imgDilPers, imgWarp)
            mask_with_curve, left_poly, right_poly, left_curverad, right_curverad = fit_polynom(imgWarp, lx=lx, rx=rx, ly=ly, ry=ry)
            curvature = (left_curverad + right_curverad) / 2
            offset_calc = compute_offset_from_center([left_poly, right_poly], x_mppx=0.002)
            draw(mask=mask_with_curve, poly_param=[left_poly, right_poly], curve_rad=curvature, offset=offset_calc)
            steering_angle_in, steering_angle_out = calculate_steering_angle_new(L=0.6, R=curvature, T=0.2)
            self.steering_angle = math.degrees(steering_angle_in)
            #steering_point = calculate_steering_angle_new_oldstyle(lx, rx, ly, ry)
            #cv2.circle(mask_with_curve, (int(steering_point[0]), int(steering_point[1])), 5, (255 ,0, 255), -1)

            # print(f"lx :\n {lx}")
            # print(f"rx :\n {rx}")
            # print(f"ly :\n {ly}")
            # print(f"ry :\n {ry}")

            # cv2.circle(img, bottom_left, 5, (0, 0, 255), -1)
            # cv2.circle(img, bottom_right, 5, (0, 0, 255), -1)
            # cv2.circle(img, top_left, 5, (0, 0, 255), -1)
            # cv2.circle(img, top_right, 5, (0, 0, 255), -1)
            end_time = time.time()
            duration = end_time - start_time  # İşlemin sürdüğü süreyi hesapla
            print(f"CallbackImage süresi: {duration} saniye.")
            cv2.imshow('Contour', imgContourPers)
            cv2.imshow("img-given", img)
            cv2.imshow('Perspective', mask_with_curve)
            cv2.imshow("SlidingWindows", img_sliding_window)
            print("steering_angle",self.steering_angle)
            #self.steerLateral = 0.2
            self.carControlPublisher()


            cv2.waitKey(3)
            # image_message = self.bridge.cv2_to_imgmsg(img, "bgr8")

            # print(len(image_message.data))

            # cv2.imshow("IMG Wrap Right", imgWarpRight)
            # cv2.imshow("IMG Wrap Left", imgWarpLeft)

            # world_coords = calculate_real_world_coor(matrix, contoursPers)
            #
            # print("World Coordinates: ", world_coords)
            # print("World Coordinates Shape: ", world_coords.shape)
            #
            # # İşaretlenmiş dünya koordinatlarını normal görüntüde işaretleme
            # img_world_coor = img.copy()
            # cv2.circle(img_world_coor, (int(world_coords[0]), int(world_coords[1])), 5, (0, 255, 0), -1)
            #
            # cv2.imshow("imgWorldCoords", img_world_coor)

        except CvBridgeError as e:
            print(e)

    def pose_callback(self, data):
        pass
        # start_time = time.time()
        # print("Position: ", data.pose.position)
        # print("Orientation: ", data.pose.orientation)
        # end_time = time.time()
        # duration = end_time - start_time  # İşlemin sürdüğü süreyi hesapla
        # print(f"CallbackPosition süresi: {duration} saniye.")

        # ser = serial.Serial(port='/dev/ttyACM0',baudrate=19200)
        # #switchOn=input('Hiz Degerini Giriniz')
        # switchOn="#1:20;;\r\n"
        # ser.write(bytes(switchOn,'utf-8'))
        # time.sleep(1)
        # value=ser.readline()
        # valueInString=str(value,'UTF-8')
        # print(valueInString)
        
    def carControlPublisher(self):

        ser = serial.Serial(port='/dev/ttyACM0',baudrate=19200)
        #switchOn=input('Hiz Degerini Giriniz')
        switchOn="#1:0;;\r\n"
        ser.write(bytes(switchOn,'utf-8'))
        #time.sleep(1)
        value=ser.readline()
        valueInString=str(value,'UTF-8')
        print(valueInString)

        if self.steering_angle > 20:
            self.steering_angle = 20
        if self.steering_angle < -20:
            self.steering_angle = -20
        
        ser = serial.Serial(port='/dev/ttyACM0',baudrate=19200)
        #switchOn=input('Hiz Degerini Giriniz')
        switchOn = "#2:{};;\r\n".format(self.steering_angle)
        ser.write(bytes(switchOn,'utf-8'))
        #time.sleep(1)
        value=ser.readline()
        valueInString=str(value,'UTF-8')
        #print(valueInString)

        # carData = {}
        # carData = {
        #     'action': '2',
        #     'steerAngle': self.steering_angle 
        #     # 'steerAngle': 30.0
        # }
        # # print("data",carData)
        # # print("math.degrees(float(self.DM2Arr(self.u[0, 1])))  ",math.degrees(float(self.DM2Arr(self.u[0, 1])))  )
        # self.carData = json.dumps(carData)
        # self.carControl.publish(self.carData)
        # #rospy.sleep(0.01)
        # car2Data = {}
        # # İkinci mesaj için veriler
        # car2Data = {
        #     'action': '1',
        #     'speed': self.steerLateral  
        #     #'speed': 0.0
        # }   
        # #print("data",car2Data)
        # self.car2Data = json.dumps(car2Data)
        # self.carControl.publish(self.car2Data)
        

def main():
    rospy.init_node('cameratracker', anonymous=False)
    cameratracker = realCamera(camera_topic_name = "/zed2i/zed_node/stereo/image_rect_color",
                                 pose_topic_name = "/zed2i/zed_node/pose")

    rate = rospy.Rate(30) # 10Hz
    try:
        while not rospy.is_shutdown():
            # Sürekli çalışacak kodlar burada
            rate.sleep()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        

if __name__ == '__main__':
    main()

