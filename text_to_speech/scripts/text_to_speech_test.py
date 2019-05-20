#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Text To Speech Tests.

File name: text_to_speech_test.py
Author: Nicolas Tobis
Date last modified: 14/05/2019
Python Version: 3.6
"""

# Import necessary packages
import unittest
import rospy
import time
import subprocess
import shlex
import sys
import os
import signal
import psutil
import wave
import getpass
import pandas as pd

from text_to_speech import TextToSpeech

# Rospy Parameter for Robot Orientation and pointing
ACTIVE_CAMERA = 'active_camera'
DETECTED = 'detected'

USER = getpass.getuser()
base_path = "/home/" + USER + "/brain-cluster-ws/src/text_to_speech/scripts/"

TTS = pd.read_csv(base_path + "resources/tts.csv", delimiter='\t')

################################ SET UP ######################################

# Create ROS Functionality for Python
def kill_child_processes(parent_pid, sig=signal.SIGTERM):
    try:
        parent = psutil.Process(parent_pid)
        print(parent)
    except psutil.NoSuchProcess:
        print("parent process not existing")
        return
    children = parent.children(recursive=True)
    print(children)
    for process in children:
        print("try to kill child: " + str(process))
        process.send_signal(sig)


class Roscore(object):
    """
    roscore wrapped into a subprocess.
    Singleton implementation prevents from creating more than one instance.
    """
    __initialized = False
    def __init__(self):
        if Roscore.__initialized:
            raise Exception("You can't create more than 1 instance of Roscore.")
        Roscore.__initialized = True
    def run(self):
        try:
            self.roscore_process = subprocess.Popen(['roscore'])
            self.roscore_pid = self.roscore_process.pid  # pid of the roscore process (which has child processes)
        except OSError as e:
            sys.stderr.write('roscore could not be run')
            raise e
    def terminate(self):
        print("try to kill child pids of roscore pid: " + str(self.roscore_pid))
        kill_child_processes(self.roscore_pid)
        self.roscore_process.terminate()
        self.roscore_process.wait()  # important to prevent from zombie process
        Roscore.__initialized = False

# Mock Datasets
class EmptyData:
    def __init__(self):
        self.instruction_id = 1
        self.type = []
        self.label = []
        self.owner = []
        self.real = []
        self.confidence = []
        self.upper_x = []
        self.upper_y = []
        self.lower_x = []
        self.lower_y = []
        self.camera = []

class OneObject:
    def __init__(self):
        self.instruction_id = 1
        self.type = ['object']
        self.label = ['snowboard']
        self.owner = ['Nico']
        self.real = [True]
        self.confidence = [1.0]
        self.upper_x = [435]
        self.upper_y = [692]
        self.lower_x = [1103]
        self.lower_y = [18]
        self.camera = [0]

class OneFaceReal:
    def __init__(self):
        self.instruction_id = 1
        self.type = ['face']
        self.label = ['Will']
        self.owner = ['empty']
        self.real = [True]
        self.confidence = [1.0]
        self.upper_x = [435]
        self.upper_y = [692]
        self.lower_x = [1103]
        self.lower_y = [18]
        self.camera = [0]

class OneFaceFake:
    def __init__(self):
        self.instruction_id = 1
        self.type = ['face']
        self.label = ['Will']
        self.owner = ['empty']
        self.real = [False]
        self.confidence = [1.0]
        self.upper_x = [435]
        self.upper_y = [692]
        self.lower_x = [1103]
        self.lower_y = [18]
        self.camera = [0]


class OneFaceOneObjectReal:
    def __init__(self):
        self.instruction_id = 1
        self.type = ['object', 'face']
        self.label = ['snowboard', 'Will']
        self.owner = ['Will', 'empty']
        self.real = [True, True]
        self.confidence = [1.0, 0.8445714712142944]
        self.upper_x = [435, 737]
        self.upper_y = [692, 528]
        self.lower_x = [1103, 923]
        self.lower_y = [18, 342]
        self.camera = [0, 0]


class OneFaceOneObjectFake:
    def __init__(self):
        self.instruction_id = 1
        self.type = ['object', 'face']
        self.label = ['snowboard', 'Will']
        self.owner = ['Will', 'empty']
        self.real = [True, False]
        self.confidence = [1.0, 0.8445714712142944]
        self.upper_x = [435, 737]
        self.upper_y = [692, 528]
        self.lower_x = [1103, 923]
        self.lower_y = [18, 342]
        self.camera = [0, 0]


class OneFaceTwoObjectsSame:
    def __init__(self):
        self.instruction_id = 1
        self.type = ['object', 'object', 'face']
        self.label = ['snowboard', 'snowboard', 'Will']
        self.owner = ['Will', 'Will', 'empty']
        self.real = [True, True, True]
        self.confidence = [1.0, 1.0, 0.8445714712142944]
        self.upper_x = [435, 435, 737]
        self.upper_y = [692, 692, 528]
        self.lower_x = [1103, 1103, 923]
        self.lower_y = [18, 18, 342]
        self.camera = [0, 0, 0]


class TwoFacesTwoObjectsSame:
    def __init__(self):
        self.instruction_id = 1
        self.type = ['object', 'object', 'face', 'face']
        self.label = ['snowboard', 'snowboard', 'Will', 'Nico']
        self.owner = ['Will', 'Nico', 'empty', 'empty']
        self.real = [True, True, True, True]
        self.confidence = [1.0, 1.0, 0.8445714712142944, 0.8445714712142944]
        self.upper_x = [435, 435, 737, 737]
        self.upper_y = [692, 692, 528, 528]
        self.lower_x = [1103, 1103, 923, 923]
        self.lower_y = [18, 18, 342, 342]
        self.camera = [0, 0, 0, 0]


class TwoFacesTwoObjectsDifferent:
    def __init__(self):
        self.instruction_id = 1
        self.type = ['object', 'object', 'face', 'face']
        self.label = ['snowboard', 'sports ball', 'Will', 'Nico']
        self.owner = ['Will', 'Nico', 'empty', 'empty']
        self.real = [True, True, True, True]
        self.confidence = [1.0, 1.0, 0.8445714712142944, 0.8445714712142944]
        self.upper_x = [435, 435, 737, 737]
        self.upper_y = [692, 692, 528, 528]
        self.lower_x = [1103, 1103, 923, 923]
        self.lower_y = [18, 18, 342, 342]
        self.camera = [0, 0, 0, 0]


class ThreeFacesThreeObjectsTwoSameOneDifferent:
    def __init__(self):
        self.instruction_id = 1
        self.type = ['object', 'object', 'object', 'face', 'face', 'face']
        self.label = ['snowboard', 'snowboard', 'sports ball', 'Will', 'Nico', 'Eivinas']
        self.owner = ['Will', 'Eivinas', 'Nico', 'empty', 'empty', 'empty']
        self.real = [True, True, True, True, True, True]
        self.confidence = [1.0, 1.0, 1.0, 0.8445714712142944, 0.8445714712142944, 0.8445714712142944]
        self.upper_x = [435, 435, 435, 737, 737, 737]
        self.upper_y = [692, 692, 692, 528, 528, 528]
        self.lower_x = [1103, 1103, 1103, 923, 923, 923]
        self.lower_y = [18, 18, 18, 342, 342, 342]
        self.camera = [0, 0, 0, 0, 0, 0]


class ThreeFacesThreeObjectsThreeDifferent:
    def __init__(self):
        self.instruction_id = 1
        self.type = ['object', 'object', 'object', 'face', 'face', 'face']
        self.label = ['snowboard', 'tennis racket', 'sports ball', 'Will', 'Nico', 'Eivinas']
        self.owner = ['Will', 'Eivinas', 'Nico', 'empty', 'empty', 'empty']
        self.real = [True, True, True, True, True, True]
        self.confidence = [1.0, 1.0, 1.0, 0.8445714712142944, 0.8445714712142944, 0.8445714712142944]
        self.upper_x = [435, 435, 435, 737, 737, 737]
        self.upper_y = [692, 692, 692, 528, 528, 528]
        self.lower_x = [1103, 1103, 1103, 923, 923, 923]
        self.lower_y = [18, 18, 18, 342, 342, 342]
        self.camera = [0, 0, 0, 0, 0, 0]


class TwoFacesOneFakeOneReal:
    def __init__(self):
        self.instruction_id = 1
        self.type = ['face', 'face']
        self.label = ['Nico', 'Will']
        self.owner = ['empty', 'empty']
        self.real = [True, False]
        self.confidence = [1.0, 0.8445714712142944]
        self.upper_x = [435, 737]
        self.upper_y = [692, 528]
        self.lower_x = [1103, 923]
        self.lower_y = [18, 342]
        self.camera = [0, 0]

class TwoFacesSame:
    def __init__(self):
        self.instruction_id = 1
        self.type = ['face', 'face']
        self.label = ['Will', 'Will']
        self.owner = ['empty', 'empty']
        self.real = [True, False]
        self.confidence = [1.0, 0.8445714712142944]
        self.upper_x = [435, 737]
        self.upper_y = [692, 528]
        self.lower_x = [1103, 923]
        self.lower_y = [18, 342]
        self.camera = [0, 0]


class TwoFacesSameReal:
    def __init__(self):
        self.instruction_id = 1
        self.type = ['face', 'face']
        self.label = ['Will', 'Will']
        self.owner = ['empty', 'empty']
        self.real = [True, True]
        self.confidence = [1.0, 0.8445714712142944]
        self.upper_x = [435, 737]
        self.upper_y = [692, 528]
        self.lower_x = [1103, 923]
        self.lower_y = [18, 342]
        self.camera = [0, 0]

################################ END SET UP ###################################

############################### START TESTS ###################################

class TestHotword(unittest.TestCase):
    def setUp(self):
        pass


class TestEmptyObject(TestHotword):
    def test_makeRequest(self):
        face = ['will']
        object = ['snowboard']
        data = EmptyData()

        results = [
            "",
            "I can’t see any objects.",
            "No, I cannot see any snowboard.",
            "I can’t see anybody.",
            "No, I cannot see will.",
            "I don’t think anybody has a snowboard.",
            "",
            "I cannot see any snowboard.",
            "I cannot see will."
        ]

        for id in range(1, 9):
            if id == 6:
                continue
            data.instruction_id = id
            text_to_speech = TextToSpeech(data, 'en-EN', input_faces=face, input_objects=object)
            out = text_to_speech.callback()
            self.assertEqual(out, results[id])


class TestOneObject(TestHotword):
    def test_makeRequest(self):
        face = ['will']
        object = ['snowboard']
        data = OneObject()

        results = [
            "",
            "I am 100 percent sure that I can see a snowboard  behind me on my left.",
            "Yes, I am 100 percent sure that I can see exactly one snowboard  behind me on my left.",
            "I can’t see anybody.",
            "No, I cannot see will.",
            "I am 100 percent confident that the snowboard belongs to Nico. behind me on my left.",
            "",
            "I am 100 percent sure that the snowboard is here  behind me on my left.",
            "I cannot see will."
        ]

        for id in range(1, 9):
            if id == 6:
                continue
            data.instruction_id = id
            text_to_speech = TextToSpeech(data, 'en-EN', input_faces=face, input_objects=object)
            out = text_to_speech.callback()
            self.assertEqual(out, results[id])


class TestOneFaceReal(TestHotword):
    def test_makeRequest(self):
        face = ['will']
        object = ['snowboard']
        data = OneFaceReal()

        results = [
            "",
            "I can’t see any objects.",
            "No, I cannot see any snowboard.",
            "I am 100 percent sure that I can see will  behind me on my left.",
            "Yes, I am 100 percent sure that I can see will  behind me on my left.",
            "I don’t think anybody has a snowboard.",
            "",
            "I cannot see any snowboard.",
            "I am 100 percent sure that will is here  behind me on my left."
        ]

        for id in range(1, 9):
            if id == 6:
                continue
            data.instruction_id = id
            text_to_speech = TextToSpeech(data, 'en-EN', input_faces=face, input_objects=object)
            out = text_to_speech.callback()
            self.assertEqual(out, results[id])
            # print(out)


class TestOneFaceFake(TestHotword):
    def test_makeRequest(self):
        face = ['will']
        object = ['snowboard']
        data = OneFaceFake()

        results = [
            "",
            "I can’t see any objects.",
            "No, I cannot see any snowboard.",
            "I am 100 percent sure that I can see will. Not the real one though. behind me on my left.",
            "Yes, I am 100 percent sure that I can see will. Not the real one though. behind me on my left.",
            "I don’t think anybody has a snowboard.",
            "",
            "I cannot see any snowboard.",
            "Yes, I am 100 percent sure that a fake will is here,  behind me on my left."
        ]

        for id in range(1, 9):
            if id == 6:
                continue
            data.instruction_id = id
            text_to_speech = TextToSpeech(data, 'en-EN', input_faces=face, input_objects=object)
            out = text_to_speech.callback()
            self.assertEqual(out, results[id])


class TestOneFaceOneObjectReal(TestHotword):
    def test_makeRequest(self):
        face = ['will']
        object = ['snowboard']
        data = OneFaceOneObjectReal()

        results = [
            "",
            "I am 100 percent sure that I can see a snowboard  behind me on my left.",
            "Yes, I am 100 percent sure that I can see exactly one snowboard  behind me on my left.",
            "I am 84 percent sure that I can see will  behind me on my left.",
            "Yes, I am 84 percent sure that I can see will  behind me on my left.",
            "I am 100 percent confident that the snowboard belongs to Will. behind me on my left.",
            "",
            "I am 100 percent sure that the snowboard is here  behind me on my left.",
            "I am 84 percent sure that will is here  behind me on my left."
        ]

        for id in range(1, 9):
            if id == 6:
                continue
            data.instruction_id = id
            text_to_speech = TextToSpeech(data, 'en-EN', input_faces=face, input_objects=object)
            out = text_to_speech.callback()
            self.assertEqual(out, results[id])


class TestOneFaceOneObjectFake(TestHotword):
    def test_makeRequest(self):
        face = ['will']
        object = ['snowboard']
        data = OneFaceOneObjectFake()

        results = [
            "",
            "I am 100 percent sure that I can see a snowboard  behind me on my left.",
            "Yes, I am 100 percent sure that I can see exactly one snowboard  behind me on my left.",
            "I am 84 percent sure that I can see will. Not the real one though. behind me on my left.",
            "Yes, I am 84 percent sure that I can see will. Not the real one though. behind me on my left.",
            "I am 100 percent confident that the snowboard belongs to Will. behind me on my left.",
            "",
            "I am 100 percent sure that the snowboard is here  behind me on my left.",
            "Yes, I am 84 percent sure that a fake will is here,  behind me on my left."
        ]

        for id in range(1, 9):
            if id == 6:
                continue
            data.instruction_id = id
            text_to_speech = TextToSpeech(data, 'en-EN', input_faces=face, input_objects=object)
            out = text_to_speech.callback()
            self.assertEqual(out, results[id])


class TestOneFaceTwoObjectsSame(TestHotword):
    def test_makeRequest(self):
        face = ['will']
        object = ['snowboard']
        data = OneFaceTwoObjectsSame()

        results = [
            "",
            "I can see 2 snowboards, ",
            "Yes, I can actually see more than one snowboard. I think there are 2.  ",
            "I am 84 percent sure that I can see will  behind me on my left.",
            "Yes, I am 84 percent sure that I can see will  behind me on my left.",
            "I think that the following people have a snowboard: Will and Will.",
            "",
            "I can actually see more than one snowboard. I think there are 2. One of them is here.  ",
            "I am 84 percent sure that will is here  behind me on my left."
        ]

        for id in range(1, 9):
            if id == 6:
                continue
            data.instruction_id = id
            text_to_speech = TextToSpeech(data, 'en-EN', input_faces=face, input_objects=object)
            out = text_to_speech.callback()
            self.assertEqual(out, results[id])


class TestTwoFacesTwoObjectsSame(TestHotword):
    def test_makeRequest(self):
        face = ['will']
        object = ['snowboard']
        data = TwoFacesTwoObjectsSame()

        results = [
            "",
            "I can see 2 snowboards, ",
            "Yes, I can actually see more than one snowboard. I think there are 2.  ",
            "I can see will and nico.",
            "Yes, I am 84 percent sure that I can see will  behind me on my left.",
            "I think that the following people have a snowboard: Will and Nico.",
            "",
            "I can actually see more than one snowboard. I think there are 2. One of them is here.  ",
            "I am 84 percent sure that will is here  behind me on my left."
        ]

        for id in range(1, 9):
            if id == 6:
                continue
            data.instruction_id = id
            text_to_speech = TextToSpeech(data, 'en-EN', input_faces=face, input_objects=object)
            out = text_to_speech.callback()
            self.assertEqual(out, results[id])


class TestTwoFacesTwoObjectsDifferent(TestHotword):
    def test_makeRequest(self):
        face = ['will']
        object = ['snowboard']
        data = TwoFacesTwoObjectsDifferent()

        results = [
            "",
            "I can see 1 sports ball, and 1 snowboard.",
            "Yes, I am 100 percent sure that I can see exactly one snowboard  behind me on my left.",
            "I can see will and nico.",
            "Yes, I am 84 percent sure that I can see will  behind me on my left.",
            "I am 100 percent confident that the snowboard belongs to Will. behind me on my left.",
            "",
            "I am 100 percent sure that the snowboard is here  behind me on my left.",
            "I am 84 percent sure that will is here  behind me on my left."
        ]

        for id in range(1, 9):
            if id == 6:
                continue
            data.instruction_id = id
            text_to_speech = TextToSpeech(data, 'en-EN', input_faces=face, input_objects=object)
            out = text_to_speech.callback()
            self.assertEqual(out, results[id])


class TestThreeFacesThreeObjectsTwoSameOneDifferent(TestHotword):
    def test_makeRequest(self):
        face = ['will']
        object = ['snowboard']
        data = ThreeFacesThreeObjectsTwoSameOneDifferent()

        results = [
            "",
            "I can see 2 snowboards, and 1 sports ball.",
            "Yes, I can actually see more than one snowboard. I think there are 2.  ",
            "I can see will nico and eivinas.",
            "Yes, I am 84 percent sure that I can see will  behind me on my left.",
            "I think that the following people have a snowboard: Will and Eivinas.",
            "",
            "I can actually see more than one snowboard. I think there are 2. One of them is here.  ",
            "I am 84 percent sure that will is here  behind me on my left."
        ]

        for id in range(1, 9):
            if id == 6:
                continue
            data.instruction_id = id
            text_to_speech = TextToSpeech(data, 'en-EN', input_faces=face, input_objects=object)
            out = text_to_speech.callback()
            self.assertEqual(out, results[id])


class TestThreeFacesThreeObjectsThreeDifferent(TestHotword):
    def test_makeRequest(self):
        face = ['will']
        object = ['snowboard']
        data = ThreeFacesThreeObjectsThreeDifferent()

        results = [
            "",
            "I can see 1 sports ball, 1 snowboard, and 1 tennis racket.",
            "Yes, I am 100 percent sure that I can see exactly one snowboard  behind me on my left.",
            "I can see will nico and eivinas.",
            "Yes, I am 84 percent sure that I can see will  behind me on my left.",
            "I am 100 percent confident that the snowboard belongs to Will. behind me on my left.",
            "",
            "I am 100 percent sure that the snowboard is here  behind me on my left.",
            "I am 84 percent sure that will is here  behind me on my left."
        ]

        for id in range(1, 9):
            if id == 6:
                continue
            data.instruction_id = id
            text_to_speech = TextToSpeech(data, 'en-EN', input_faces=face, input_objects=object)
            out = text_to_speech.callback()
            self.assertEqual(out, results[id])


class TestTwoFacesOneFakeOneReal(TestHotword):
    def test_makeRequest(self):
        face = ['will']
        object = ['snowboard']
        data = TwoFacesOneFakeOneReal()

        results = [
            "",
            "I can’t see any objects.",
            "No, I cannot see any snowboard.",
            "I can see nico and will. But at least one of them is fake.",
            "Yes, I am 84 percent sure that I can see will. Not the real one though. behind me on my left.",
            "I don’t think anybody has a snowboard.",
            "",
            "I cannot see any snowboard.",
            "Yes, I am 84 percent sure that a fake will is here,  behind me on my left."
        ]

        for id in range(1, 9):
            if id == 6:
                continue
            data.instruction_id = id
            text_to_speech = TextToSpeech(data, 'en-EN', input_faces=face, input_objects=object)
            out = text_to_speech.callback()
            self.assertEqual(out, results[id])


class TestTwoFacesSameOneFakeOneReal(TestHotword):
    def test_makeRequest(self):
        face = ['will']
        object = ['snowboard']
        data = TwoFacesSame()

        results = [
            "",
            "I can’t see any objects.",
            "No, I cannot see any snowboard.",
            "I can see will and will. But at least one of them is fake.",
            "I can actually see will more than once. But at least one of them is fake.  ",
            "I don’t think anybody has a snowboard.",
            "",
            "I cannot see any snowboard.",
            "Yes, I can actually see will more than once. At least one of them is fake.  "
        ]

        for id in range(1, 9):
            if id == 6:
                continue
            data.instruction_id = id
            text_to_speech = TextToSpeech(data, 'en-EN', input_faces=face, input_objects=object)
            out = text_to_speech.callback()
            self.assertEqual(out, results[id])


class TestTwoFacesSameBothReal(TestHotword):
    def test_makeRequest(self):
        face = ['will']
        object = ['snowboard']
        data = TwoFacesSameReal()

        results = [
            "",
            "I can’t see any objects.",
            "No, I cannot see any snowboard.",
            "I can see will and will.",
            "Yes, I can actually see will more than once. I knew I shouldn’t have had that last pint last night.  ",
            "I don’t think anybody has a snowboard.",
            "",
            "I cannot see any snowboard.",
            "Yes, I can actually see will more than once. I knew I shouldn’t have had that last pint last night. One of them is here.  "
        ]

        for id in range(1, 9):
            if id == 6:
                continue
            data.instruction_id = id
            text_to_speech = TextToSpeech(data, 'en-EN', input_faces=face, input_objects=object)
            out = text_to_speech.callback()
            self.assertEqual(out, results[id])

############################### END TESTS #####################################


############################### MAIN ##########################################

if __name__=="__main__":
    roscore = Roscore()
    roscore.run()
    time.sleep(1)  # wait a bit to be sure the roscore is really launched

    ####################### INITIALIZE YOUR NODES HERE ########################

    rospy.init_node('SpeechClient')

    ####################### END OF SECTION ####################################

    try:
        unittest.main()
    finally:
        roscore.terminate()
