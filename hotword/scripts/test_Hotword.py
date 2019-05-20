#!/usr/bin/env python

"""
Hotword Tests.

File name: test_Hotword.py
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

# Import Ros Scripts
from hotword_client import HotwordClient
from hotword_service import HotwordService
from hotword.srv import LangCode

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

################################ END SET UP ###################################

############################### START TESTS ###################################

class TestHotword(unittest.TestCase):
    def setUp(self):
        self.hotwordClient = HotwordClient()


class TestMakeRequestInEnglish(TestHotword):
    def test_makeRequest(self):
        f = wave.open("resources/HeyDeNiro.wav")
        data = f.readframes(f.getnframes())
        hotword_server = HotwordService(data)
        s = rospy.Service('hotword_recognition',
                          LangCode,
                          hotword_server.detect_hotword)
        language_code = self.hotwordClient.make_request()
        self.assertEqual(language_code, "en-EN")
        s.shutdown()


class TestMakeRequestInSpanish(TestHotword):
    def test_makeRequest(self):
        f = wave.open("resources/HolaDeNiro.wav")
        data = f.readframes(f.getnframes())
        hotword_server = HotwordService(data)
        s = rospy.Service('hotword_recognition',
                          LangCode,
                          hotword_server.detect_hotword)
        language_code = self.hotwordClient.make_request()
        self.assertEqual(language_code, "es-ES")
        s.shutdown()


class TestMakeRequestInItalian(TestHotword):
    def test_makeRequest(self):
        f = wave.open("resources/CiaoDeNiro.wav")
        data = f.readframes(f.getnframes())
        hotword_server = HotwordService(data)
        s = rospy.Service('hotword_recognition',
                          LangCode,
                          hotword_server.detect_hotword)
        language_code = self.hotwordClient.make_request()
        self.assertEqual(language_code, "it-IT")
        s.shutdown()


class TestMakeRequestInGerman(TestHotword):
    def test_makeRequest(self):
        f = wave.open("resources/HalloDeNiro.wav")
        data = f.readframes(f.getnframes())
        hotword_server = HotwordService(data)
        s = rospy.Service('hotword_recognition',
                          LangCode,
                          hotword_server.detect_hotword)
        language_code = self.hotwordClient.make_request()
        self.assertEqual(language_code, "de-DE")
        s.shutdown()


class TestMakeRequestInDutch(TestHotword):
    def test_makeRequest(self):
        f = wave.open("resources/HoiDeNiro.wav")
        data = f.readframes(f.getnframes())
        hotword_server = HotwordService(data)
        s = rospy.Service('hotword_recognition',
                          LangCode,
                          hotword_server.detect_hotword)
        language_code = self.hotwordClient.make_request()
        self.assertEqual(language_code, "nl-NL")
        s.shutdown()

class TestMakeRequestGeneratesServiceException(TestHotword):
    def test_makeRequest(self):
        try:
            sentence = self.hotwordClient.make_request()
        except rospy.ROSException:
            # Test Passed
            pass


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
