#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Class responsible for performing face recognition and spoofing detection.

File name: VideoRecognition.py
Author: Tolga Dur, Jiacheng Wang
Date Created: 16/03/2019
Data last modified: 14/05/2019
Python Version: 2.7
"""

from scipy.spatial import distance as dist
from FaceRecognition import FaceRecognition
from face_spoofing.msg import ObjectArray, Obj
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from imutils.video import FPS
from imutils import face_utils
import dlib
import face_recognition
import numpy as np
import imutils
import pickle
import cv2
import rospy
import time
import os


class VideoRecognition(FaceRecognition):
    def __init__(self, camera_name, id):
        super(VideoRecognition, self).__init__()

        # load the actual face recognition model along with the label encoder
        self.encoding_data = pickle.loads(open(self.encoding_file, "rb").read())

        # variable for spoofing
        self.rect_len_start = 0
        self.rect_len_next = 0

        # Facial Landmark predictor for spoofing detection
        self.landmark_predictor = dlib.shape_predictor("./shape_predictor_68_face_landmarks.dat")

        # Grab the indexes of the facial landmarks for the left and right eye, respectively
        (self.lStart, self.lEnd) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]
        (self.rStart, self.rEnd) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]

        # Spoofing parameters
        self.TOTAL = []
        self.RECT = []
        self.FAKE_FACE = []
        coordinates = []

        # Initialising ROS Publisher
        self.id = id
        if id == 1:
            self.pub = rospy.Publisher('face', ObjectArray, queue_size=100)
        elif id == 2 :
            self.pub = rospy.Publisher('face2', ObjectArray, queue_size=100)
        elif id == 3:
            self.pub = rospy.Publisher('face3', ObjectArray, queue_size=100)
        elif id == 4:
            self.pub = rospy.Publisher('face4', ObjectArray, queue_size=100)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(camera_name, Image, self.callback)


    def callback(self, data):
        """
        Detects faces from frames received from the camera streams. Then sets up the
        parameters and calls the functions doing the face recognition function and spoofing.

        Keyword arguments:
        data -- image frames from cameras
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # face_recognition and spoofing should use the same frame
        frame = imutils.resize(cv_image, width=1920, height=1080)
        detector = dlib.get_frontal_face_detector()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # detect faces in the grayscale frame
        coordinates = detector(gray, 0)
        names, confidences = self.__face_recognition(frame)
        obj_array = self.__convert_to_obj_array(names, coordinates, confidences)
        self.rect_len_next = len(names)

        # when number of faces changes, initiate spoofing_detection variables
        if self.rect_len_start == 0:
            self.TOTAL = []
            self.RECT = []
            self.FAKE_FACE = []

        if self.rect_len_next != self.rect_len_start:
            self.TOTAL = []
            self.RECT = []
            self.FAKE_FACE = []
            for i in range(len(names)):
                self.RECT.append(0)
                self.TOTAL.append(0)
                self.FAKE_FACE.append(True)

        obj_array = self.__spoofing(names, obj_array, frame, coordinates)
        self.pub.publish(obj_array)

    def __face_recognition(self, frame):
        """
        Recognizes faces from frame and return the names and confidences of faces.

        Keyword arguments:
        frame -- image from cameras
        """
        print("Detecting...")
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        boxes = face_recognition.face_locations(rgb, model=self.method)
        print("printing boxes", boxes)
        names, confidences = self.__get_recognised_names(boxes,rgb)

        return names, confidences

    def __get_recognised_names(self, boxes, rgb):
        """
        Compares faces given by boxes with dataset and returns the result.

        Keyword arguments:
        boxes -- faces extracted from frame
        rgb -- different version of frame
        """
        encodings = face_recognition.face_encodings(rgb, boxes)
        names = []
        confidences = []

        for encoding in encodings:
            matches = face_recognition.compare_faces(self.encoding_data["encodings"], encoding)
            name = "Unknown"
            confidence = 100.0

            if True in matches:
                # count how many times each face was matched
                matchCount = [i for (i, b) in enumerate(matches) if b]
                counts = {}

                # loop over the matched indexes and maintain a count for each recognized face
                for i in matchCount:
                    name = self.encoding_data["names"][i]
                    counts[name] = counts.get(name, 0) + 1

                name = max(counts, key=counts.get)
                picture_count = sum([1 for d in self.encoding_data["names"] if name in d])
                confidence = (float(counts[name])/picture_count) * 100
                if confidence < self.minConfidence:
                    name = "Unknown"
                    confidence = 100.0 - confidence

            names.append(name)
            confidences.append(confidence)
        return names, confidences

    def __convert_to_obj_array(self, names, coordinates, confidences):
        """
        Creates the ROS message that is later passed to the environment model.

        Keyword arguments:
        names -- names of faces
        coordinates -- coordinates of faces on the image frame
        confidences -- the confidence of the recogintion
        """
        object_message = ObjectArray()
        for name in names:
            object = Obj()
            object.label = name
            object.boolean = True
            object_message.obj_list.append(object)

        for i, d in enumerate(coordinates):
            object_message.obj_list[i].upper_x = d.left() if d.left() > 0 else 0
            object_message.obj_list[i].upper_y = d.top() if d.top() > 0 else 0
            object_message.obj_list[i].lower_x = d.right() if d.right() > 0 else 0
            object_message.obj_list[i].lower_y = d.bottom() if d.bottom() > 0 else 0
            object_message.obj_list[i].confidence = confidences[i]
            object_message.id = self.id
        return object_message

    @staticmethod
    def __eye_aspect_ratio(eye):
        """
        Returns the eye aspect ratio.

        Keyword arguments:
        eye -- eye landmarks
        """
        A = dist.euclidean(eye[1], eye[5])
        B = dist.euclidean(eye[2], eye[4])
        C = dist.euclidean(eye[0], eye[3])

        # compute the eye aspect ratio
        return (A + B) / (2.0 * C)

    def __spoofing(self, names, obj_array, frame, coordinates):
        """
        Detects whether faces are real or fake.

        Keyword arguments:
        names -- names of the faces
        obj_array -- contain all the data need to be sent to environment model
        frame -- raw image from cameras
        coordinates -- coordinates of faces
        """
        # spoofing detection: grab the indexes of the facial landmarks for the left and right eye, respectively
        EYE_AR_THRESH = 0.2
        EYE_AR_CONSEC_FRAMES = 2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if len(names) < len(coordinates):
            for i in range(len(names),len(coordinates)):
                self.RECT.append(0)
                self.TOTAL.append(0)
                self.FAKE_FACE.append(True)

        for (i, rect) in enumerate(coordinates):
            self.RECT[i] += 1
            shape = self.landmark_predictor(gray, rect)
            shape = face_utils.shape_to_np(shape)

            (x, y, w, h) = face_utils.rect_to_bb(rect)
            leftEye = shape[self.lStart:self.lEnd]
            rightEye = shape[self.rStart:self.rEnd]
            leftEAR = self.__eye_aspect_ratio(leftEye)
            rightEAR = self.__eye_aspect_ratio(rightEye)
            leftEyeHull = cv2.convexHull(leftEye)
            rightEyeHull = cv2.convexHull(rightEye)
            cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
            cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)
            ear = (leftEAR + rightEAR) / 2.0

            if ear < EYE_AR_THRESH:
                self.TOTAL[i] += 2
                self.FAKE_FACE[i] = False


            if self.RECT[i] > 15 and self.TOTAL[i] < EYE_AR_CONSEC_FRAMES:
                obj_array.obj_list[i].boolean = False

        self.rect_len_start = len(coordinates)
        return obj_array
