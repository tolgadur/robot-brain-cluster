#!/usr/bin/env python
"""
Speech Rec Tests.

File name: test_speechRecClient.py
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

# Import Ros Scripts
from speech_rec_client import SpeechRecClient
from google_speech_rec_service import SpeechRecService
from speech_rec.srv import GetSentence

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

class TestSpeechRecClient(unittest.TestCase):
    def setUp(self):
        self.speechRecClient = SpeechRecClient()


class TestMakeRequestInEnglish(TestSpeechRecClient):
    def test_makeRequest(self):
        speech_server = SpeechRecService("resources/Hello.wav")
        s = rospy.Service('speech_recognition',
                          GetSentence,
                          speech_server.translate_speech)
        sentence = self.speechRecClient.make_request('en-EN')
        self.assertEqual(sentence, "hello")
        s.shutdown()


class TestMakeRequestInSpanish(TestSpeechRecClient):
    def test_makeRequest(self):
        speech_server = SpeechRecService("resources/Hola.wav")
        s = rospy.Service('speech_recognition',
                          GetSentence,
                          speech_server.translate_speech)
        sentence = self.speechRecClient.make_request('es-ES')
        self.assertEqual(sentence, "hola")
        s.shutdown()


class TestMakeRequestGeneratesServiceException(TestSpeechRecClient):
    def test_makeRequest(self):
        try:
            sentence = self.speechRecClient.make_request('Random')
        except rospy.ROSException:
            # Test Passed
            pass

class TestMakeRequestForRandomSound(TestSpeechRecClient):
    def test_makeRequest(self):
        speech_server = SpeechRecService("resources/random_sound.wav")
        s = rospy.Service('speech_recognition',
                          GetSentence,
                          speech_server.translate_speech)
        sentence = self.speechRecClient.make_request('es-ES')
        self.assertEqual(sentence, None)
        s.shutdown()


class TestMakeRequestWithoutNetworkConnection(TestSpeechRecClient):
    def test_makeRequest(self):
        speech_server = SpeechRecService("resources/random_sound.wav")
        s = rospy.Service('speech_recognition',
                          GetSentence,
                          speech_server.translate_speech)
        os.system("nmcli radio wifi off")
        sentence = self.speechRecClient.make_request('es-ES')
        os.system("nmcli radio wifi on")
        self.assertEqual(sentence, None)
        s.shutdown()


class TestMakeRequestInEnglishWithMicrophone(TestSpeechRecClient):
    def test_makeRequest(self):
        speech_server = SpeechRecService()
        s = rospy.Service('speech_recognition',
                          GetSentence,
                          speech_server.translate_speech)
        sentence = self.speechRecClient.make_request('es-ES')
        self.assertEqual(sentence, "hola")
        s.shutdown()


class TestRunServiceMainAndTestMakeRequest(TestSpeechRecClient):
    def test_makeRequest(self):
        print("Launching python speech rec")
        subprocess.Popen(args=["gnome-terminal", "--command=python speech_rec_service.py"])
        time.sleep(5)
        print("Waking up")
        sentence = self.speechRecClient.make_request('en-EN')
        assert isinstance(sentence, unicode)


############################### END TESTS #####################################


############################### MAIN ##########################################

if __name__=="__main__":
    roscore = Roscore()
    roscore.run()
    time.sleep(3)  # wait a bit to be sure the roscore is really launched

    ####################### INITIALIZE YOUR NODES HERE ########################

    rospy.init_node('SpeechClient')

    ####################### END OF SECTION ####################################

    try:
        unittest.main()
    finally:
        roscore.terminate()
