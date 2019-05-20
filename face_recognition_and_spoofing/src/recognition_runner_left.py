#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Run the service for face recognition and spoofing in python

File name: recognition_runner_left.py
Author: Tolga Dur, Jiacheng Wang
Date Created: 16/03/2019
Data last modified: 14/05/2019
Python Version: 2.7
"""

from ModelTrainer import ModelTrainer
from VideoRecognition import VideoRecognition
import rospy
import argparse


def main():
    videoRecognition = VideoRecognition("/usb_cam_left/image_raw", 2)
    rospy.init_node("face_spoofing_left", anonymous=True)
    try:
        rospy.spin()
        print("success")
    except KeyboardINterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
