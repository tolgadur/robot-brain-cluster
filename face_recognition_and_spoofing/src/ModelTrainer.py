#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Class responsible for creating the econding model for face recognition. This class
also contains the functions for live training of faces.

File name: ModelTrainer.py
Author: Tolga Dur, Jiacheng Wang
Date Created: 15/03/2019
Data last modified: 14/05/2019
Python Version: 2.7
"""

from sklearn.preprocessing import LabelEncoder
from FaceRecognition import FaceRecognition
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
import face_recognition
from sklearn.svm import SVC  # Can use other classification methods
import pickle
import cv2
import os
import numpy as np
import uuid
from imutils import paths
import imutils


class ModelTrainer(FaceRecognition):
    def __init__(self, camera_name):
        super(ModelTrainer, self).__init__()

        self.image_paths = list(paths.list_images(self.dataset))

        self.known_encoding = []
        self.known_names = []

        self.num_faces_processed = 0

        self.camera_name = camera_name
        self.picture_count = 0
        self.name = "Unknown"


    def __deleting_encoding(self, name):
        """
        Deletes old encodings if the face to be training is already in the dataset.
        This is done so that the encodings of this face can be recomputed and appended
        to the model.

        Keyword arguments:
        name -- name of the new face
        """
        data = pickle.loads(open(self.encoding_file, "rb").read())
        print("deleting {} ...".format(name))
        for value in data["names"][:]:
            if value == name:
                index = data["names"].index(value)
                del data["encodings"][index]
                del data["names"][index]

        f = open(self.encoding_file, "wb")
        f.write(pickle.dumps(data, protocol=2))
        f.close()

    def train_with_camera(self, name):
        self.__deleting_encoding(name)
        self.bridge = CvBridge()
        self.name = name
        self.image_sub = rospy.Subscriber(self.camera_name, Image, self.train_with_camera_callback)

    def train_with_camera_callback(self, data):
        """
        Extracts faces from frames received from the camera streams.

        Keyword arguments:
        data -- image frames from cameras.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        frame = imutils.resize(cv_image, width=1920, height=1080)
        path = "./dataset/" + self.name + "/"

        if not os.path.exists(os.path.dirname(path)):
            try:
                os.makedirs(os.path.dirname(path))
            except OSError as exc: # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        if self.picture_count % 5 == 0:
            filename = str(uuid.uuid4()) + ".jpg"
            print("collecting {}'s face...".format(self.name))
            cv2.imwrite(os.path.join(path, filename), frame)

        self.picture_count += 1
        if self.picture_count > 125:
            self.image_sub.unregister()
            print("training face..")
            data = pickle.loads(open(self.encoding_file, "rb").read())
            self.known_encoding = data.get("encodings")
            self.known_names = data.get("names")
            self.train_model()
            rospy.signal_shutdown("successful shutdown :-)")

    def train_model(self):
        """
        Encodes new faces and appends to the model file: encodings.pickle
        """
        # These lists will later contain the face encodings and corresponding names for each person in the dataset
        encodingResult = []
        nameResult = []

        for (i, imagePath) in enumerate(self.image_paths):
            # Get each persons name from imagePaths and put them in a python list
            name = imagePath.split("/")[2]
            if name not in self.known_names:
                print("Encoding " + name + "'s face...")

                image = cv2.imread(imagePath)
                rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

                # Detect cordinates of faces
                boxes = face_recognition.face_locations(rgb, model=self.method)

                # compute the encoding for the face
                encodings = face_recognition.face_encodings(rgb, boxes)

                # loop over the encodings
                for encoding in encodings:
                # add each encoding + name to our set of known names and encodings
                    encodingResult.append(encoding)
                    nameResult.append(name)

        encodingResult += self.known_encoding
        nameResult += self.known_names
        # Writing the encodings onto the 'encodings.pickle file'
        print("Almost done...")
        data = {"encodings": encodingResult, "names": nameResult}
        print
        file = open(self.encoding_file, "wb")
        file.write(pickle.dumps(data, protocol=2))
        file.close()
        print("done!")
