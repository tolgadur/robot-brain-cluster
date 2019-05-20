#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Superclass specifying useful parameters and paths to the dataset and the encoding file.

File name: FaceRecognition.py
Author: Tolga Dur, Jiacheng Wang
Date Created: 14/03/2019
Data last modified: 14/05/2019
Python Version: 2.7
"""

from imutils.video import VideoStream
import cv2
import time


class FaceRecognition(object):
    def __init__(self):
        self.dataset = "./dataset"
        self.encoding_file = "./output/encoding.pickle"
        self.method = "hog"  # Can be CNN aternatively
        self.minConfidence = 50.0
