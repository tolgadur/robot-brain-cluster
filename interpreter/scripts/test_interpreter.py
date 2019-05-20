#!/usr/bin/env python

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
from interpreter_client import InterpreterClient
from interpreter_service import InterpreterService
from interpreter.srv import Interpreter

################################ SET UP ######################################

# Create ROS Functionality for Python
def kill_child_processes(parent_pid, sig=signal.SIGTERM): # pragma: no cover
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

class Roscore(object): # pragma: no cover
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

class TestInterpreterClient(unittest.TestCase):
    def setUp(self):
        self.interpreterClient = InterpreterClient()

class TestMakeNoRequest(TestInterpreterClient):
    def test_makeRequest(self):
        interpreter_server = InterpreterService()
        s = rospy.Service('interpreter',
                         Interpreter,
                         interpreter_server.translate_string)
        returned = self.interpreterClient.make_request("")
        empty = []
        self.assertEqual(returned,empty)
        s.shutdown()

<<<<<<< HEAD
class TestEmptyRequest(TestInterpreterClient):
    def test_makeRequest(self):
        interpreter_server = InterpreterService()
        s = rospy.Service('interpreter',
                         Interpreter,
                         interpreter_server.translate_string)
        returned = self.interpreterClient.make_request("who has the ball")
        expected = 5,[],[],['baseball bat','sports ball','baseball glove']
        self.assertEqual(returned,expected)
        s.shutdown()

class TestMakeRequestForObject(TestInterpreterClient):
     def test_makeRequest(self):
         interpreter_server = InterpreterService()
         s = rospy.Service('interpreter',
                           Interpreter,
                           interpreter_server.translate_string)
         returned = self.interpreterClient.make_request("Can you see a plane")
         expected = ['2','-1','aeroplane']
         self.assertEqual(returned,expected)
         s.shutdown()

class TestMakeRequestForFace(TestInterpreterClient):
     def test_makeRequest(self):
         interpreter_server = InterpreterService()
         s = rospy.Service('interpreter',
                           Interpreter,
                           interpreter_server.translate_string)
         returned = self.interpreterClient.make_request("Can you see Tolga")
         expected = ['4','tolga','-1']
         self.assertEqual(returned,expected)
         s.shutdown()

class TestTextStripping(TestInterpreterClient):
     def test_makeRequest(self):
         interpreter_server = InterpreterService()
         s = rospy.Service('interpreter',
                           Interpreter,
                           interpreter_server.translate_string)
         returned = self.interpreterClient.make_request("can you see a plane the ")
         expected = ['2','-1','aeroplane']
         self.assertEqual(returned,expected)
         s.shutdown()

class TestMakeRequestWithoutNetworkConnection(TestInterpreterClient):
    def test_makeRequest(self):
        interpreter_server = InterpreterService()
        s = rospy.Service('interpreter',
                          Interpreter,
                          interpreter_server.translate_string)
        os.system("nmcli radio wifi off")
        returned = self.interpreterClient.make_request('test')
        os.system("nmcli radio wifi on")
        self.assertEqual(returned, [])
        s.shutdown()
=======
# class TestConflcit(TestInterpreterClient):
#     def test_makeRequest(self):
#         interpreter_server = InterpreterService()
#         s = rospy.Service('interpreter',
#                          Interpreter,
#                          interpreter_server.translate_string)
#         returned = self.interpreterClient.make_request("who has the ball")
#         expected = 5,[],[],['baseball bat','sports ball','baseball glove']
#         self.assertEqual(returned,expected)
#         s.shutdown()
#
# class TestMakeRequestForObject(TestInterpreterClient):
#      def test_makeRequest(self):
#          interpreter_server = InterpreterService()
#          s = rospy.Service('interpreter',
#                            Interpreter,
#                            interpreter_server.translate_string)
#          returned = self.interpreterClient.make_request("Can you see a plane")
#          expected = 2,[],['aeroplane'],[]
#          self.assertEqual(returned,expected)
#          s.shutdown()
#
# class TestMakeRequestForFace(TestInterpreterClient):
#      def test_makeRequest(self):
#          interpreter_server = InterpreterService()
#          s = rospy.Service('interpreter',
#                            Interpreter,
#                            interpreter_server.translate_string)
#          returned = self.interpreterClient.make_request("Can you see Tolga")
#          expected = 4,['tolga'],[],[]
#          self.assertEqual(returned,expected)
#          s.shutdown()
#
# class TestMakeRequestForConflictingObject(TestInterpreterClient):
#      def test_makeRequest(self):
#          interpreter_server = InterpreterService()
#          s = rospy.Service('interpreter',
#                            Interpreter,
#                            interpreter_server.translate_string)
#          returned = self.interpreterClient.make_request("Can you see tolga or josh")
#          expected = 4,['josh','tolga'],[],[]
#          self.assertEqual(returned,expected)
#          s.shutdown()
#
# class TestMakeRequestForLocation(TestInterpreterClient):
#      def test_makeRequest(self):
#          interpreter_server = InterpreterService()
#          s = rospy.Service('interpreter',
#                            Interpreter,
#                            interpreter_server.translate_string)
#          returned = self.interpreterClient.make_request("where is josh")
#          expected = 8,['josh',],[],[]
#          self.assertEqual(returned,expected)
#          s.shutdown()
#
# class TestMakeRequestForActivation(TestInterpreterClient):
#      def test_makeRequest(self):
#          interpreter_server = InterpreterService()
#          s = rospy.Service('interpreter',
#                            Interpreter,
#                            interpreter_server.translate_string)
#          returned = self.interpreterClient.make_request("activate")
#          expected = -8,[],[],[]
#          self.assertEqual(returned,expected)
#          s.shutdown()
#
# class TestMakeRequestForChangeLanguage(TestInterpreterClient):
#      def test_makeRequest(self):
#          interpreter_server = InterpreterService()
#          s = rospy.Service('interpreter',
#                            Interpreter,
#                            interpreter_server.translate_string)
#          returned = self.interpreterClient.make_request("change language")
#          expected = -6,[],[],[]
#          self.assertEqual(returned,expected)
#          s.shutdown()


# class TestTextStripping(TestInterpreterClient):
#      def test_makeRequest(self):
#          interpreter_server = InterpreterService()
#          s = rospy.Service('interpreter',
#                            Interpreter,
#                            interpreter_server.translate_string)
#          returned = self.interpreterClient.make_request("can you see a plane the ")
#          expected = ['2','-1','aeroplane']
#          self.assertEqual(returned,expected)
#          s.shutdown()
#
# class TestMakeRequestWithoutNetworkConnection(TestInterpreterClient):
#     def test_makeRequest(self):
#         interpreter_server = InterpreterService()
#         s = rospy.Service('interpreter',
#                           Interpreter,
#                           interpreter_server.translate_string)
#         os.system("nmcli radio wifi off")
#         returned = self.interpreterClient.make_request('test')
#         os.system("nmcli radio wifi on")
#         self.assertEqual(returned, [])
#         s.shutdown()
>>>>>>> 0f6361b671a9374ff349b50abead5caf4d02fe25


############################### END TESTS #####################################

############################### MAIN ##########################################

if __name__=="__main__":
    roscore = Roscore()
    roscore.run()
    time.sleep(3)  # wait a bit to be sure the roscore is really launched

    ####################### INITIALIZE YOUR NODES HERE ########################

    rospy.init_node('InterpreterClient')

    ####################### END OF SECTION ####################################

    try:
        unittest.main()
    finally:
        roscore.terminate()
