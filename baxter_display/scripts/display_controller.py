#!/usr/bin/python

"""
Display Controller.

File name: display_controller.py
Author: Nicolas Tobis
Date last modified: 14/05/2019
Python Version: 3.6
"""

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import sys
import getpass
import time

import rospy

import cv2
import cv_bridge
from PIL import (Image as IMG,
                ImageDraw,
                ImageFont)


from sensor_msgs.msg import (
    Image,
)

USER = getpass.getuser()
BASE_PATH = "/home/" + USER + "/brain-cluster-ws/src/baxter_display/"
USER_INPUT_SPEECH_PATH =    BASE_PATH + "share/images/output.png"
MOTION_WAVE_PATH =          BASE_PATH + "share/images/Motion_Audio_Wave.mov"
STILL_WAVE_PATH =           BASE_PATH + "share/images/Still_Audio_Wave.mov"
FONT_PATH =                 BASE_PATH + "scripts/resources/OpenSans-Regular.ttf"

# Defining ROS parameters
STOP_THREAD = 'stop_thread'
DISPLAY_INFORMATION = 'display_information'
ACTIVE_CAMERA = 'active_camera'
RECOG_UPPER_X = 'recog_upper_x'
RECOG_UPPER_Y = 'recog_upper_y'
RECOG_LOWER_X = 'recog_lower_x'
RECOG_LOWER_Y = 'recog_lower_y'
RECOG_LABEL = 'recog_label'
RECOG_CAMERA = 'recog_camera'
FACE_TRAINING_ACTIVE = 'face_training_active'


class DisplayController:
    """
    Display controller class

    Keyword arguments:
    width -- int
    height -- int
    play_time -- int
    """
    def __init__(self, width=1024, height=600, play_time=None):
        self.width = width
        self.height = height
        self.pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)

    def generate_png(self, output_string):
        """
        Generate a png from the speech recognition output

        Keyword arguments:
        output_string -- string
        """

        img = IMG.new('RGB', (self.width, self.height), (22, 22, 33))
        d = ImageDraw.Draw(img)

        f_size = 36
        fnt = ImageFont.truetype(FONT_PATH, f_size)
        d.text((self.width/2 - len(output_string) * f_size /4, self.height / 2 - f_size), output_string, font=fnt, fill=(235, 235, 235))
        img.save(USER_INPUT_SPEECH_PATH, 'png')

    def send_image(self):
        """
        Send the image located at the specified path to the head
        display on Baxter.

        @param path: path to the image file to load and send
        """

        img = cv2.imread(USER_INPUT_SPEECH_PATH)

        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        self.pub.publish(msg)

    def send_boxed_image(self, camera = 1, upper_x = None, upper_y = None, lower_x = None, lower_y = None, labels=None, cameras=None):
        """
        Send the image located at the specified path to the head
        display on Baxter.

        @param path: path to the image file to load and send
        """

        try:
            if camera == 0:
                msg = rospy.wait_for_message("/usb_cam_left/image_raw", Image, timeout=1)

            elif camera == 2:
                msg = rospy.wait_for_message("/usb_cam_right/image_raw", Image, timeout=1)

            else:
                msg = rospy.wait_for_message("/usb_cam_middle/image_raw", Image, timeout=1)

            img = cv_bridge.CvBridge().imgmsg_to_cv2(msg)
            # cv2.imshow("name",img)
            # cv2.waitKey()
            for counter in range(len(upper_x)):
                if cameras[counter] == camera:
                    img = cv2.rectangle(img,(upper_x[counter],1080 - upper_y[counter]),(lower_x[counter],1080 - lower_y[counter]),0,thickness=5)
                    cv2.putText(img, labels[counter], (upper_x[counter] + 3, 1080 - upper_y[counter]-10+3), cv2.FONT_HERSHEY_SIMPLEX, 2, (1,1,1), 2)
                    cv2.putText(img, labels[counter], (upper_x[counter], 1080 - upper_y[counter]-10), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2)
            img = cv2.resize(img,(1024,600))

            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="rgb8")
            self.pub.publish(msg)
        except:
            pass


    def play_video(self, duration=None, idle=False, boxed_output=None):
        """
        Send images to the screen

        Keyword arguments:
        duration -- float
        idle -- boolean
        boxed_output -- boolean
        """

        if rospy.get_param(DISPLAY_INFORMATION):
            self.send_boxed_image(
                rospy.get_param(ACTIVE_CAMERA),
                rospy.get_param(RECOG_UPPER_X),
                rospy.get_param(RECOG_UPPER_Y),
                rospy.get_param(RECOG_LOWER_X),
                rospy.get_param(RECOG_LOWER_Y),
                rospy.get_param(RECOG_LABEL),
                rospy.get_param(RECOG_CAMERA)
            )

        elif rospy.get_param(FACE_TRAINING_ACTIVE):
            FPS = 7
            while rospy.get_param(FACE_TRAINING_ACTIVE):
                try:
                    msg = rospy.wait_for_message("/usb_cam_middle/image_raw", Image, timeout=1)
                    img = cv_bridge.CvBridge().imgmsg_to_cv2(msg)
                    img = cv2.resize(img,(1024,600))
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="rgb8")
                    self.pub.publish(msg)
                    rospy.sleep(1.0/FPS)
                except:
                    pass

        else:
            global stop_thread
            # Create a VideoCapture object and read from input file
            if idle:
                cap = cv2.VideoCapture(STILL_WAVE_PATH)
            else:
                cap = cv2.VideoCapture(MOTION_WAVE_PATH)

            # Check if camera opened successfully
            if not cap.isOpened():
                print("Error opening video  file")

            # Get FPS
            fps = int(cap.get(cv2.CAP_PROP_FPS))

            # Get publishing rate
            # pub_rate = rospy.Rate(1) # 10hz

            # Read until video is completed
            start_time = time.time()
            while cap.isOpened():

                # Capture frame-by-frame
                ret, frame = cap.read()
                if ret:
                    msg = cv_bridge.CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")
                    self.pub.publish(msg)
                    rospy.sleep(1.0/fps)
                    if duration:
                        if time.time() - start_time > duration:
                            break
                    if rospy.get_param(STOP_THREAD):
                        break

                # Break the loop
                else:
                    break

            # When everything done, release
            # the video capture object
            cap.release()

            # Closes all the frames
            cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('rsdk_xdisplay_image', anonymous=True)
