#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ROS service for live face training in Python

File Name: face_training_service.py
Author: Tolga Dur, Jiacheng Wang
Date Created: 15/03/2019
Data last modified: 14/05/2019
Python Version: 2.7
"""

import sys
import os

import rospy
from ModelTrainer import ModelTrainer

from face_spoofing.srv import GetName

class FaceTrainingService:

    def __init__(self, name=None):
        self.name = name

    def train_face(self, request):
        """
        Passes name to the ModelTrainer class

        Keyword arguments:
        request -- name
        """
        modelTrainer = ModelTrainer("/usb_cam_left/image_raw")
        modelTrainer.train_with_camera(request.name)


if __name__ == "__main__": # pragma: no cover
    rospy.init_node('face_training_server')
    face_training_server = FaceTrainingService()
    s = rospy.Service('face_training', GetName, face_training_server.train_face)
    rospy.spin()
