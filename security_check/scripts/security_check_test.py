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
import pandas as pd
import time

from security_check import SecurityCheck

################################ SET UP ######################################
class RealParticipant:
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
        self.camera = [1]

class FakeParticipant:
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
        self.camera = [1]

class RealParticipantAndID:
    def __init__(self):
        self.instruction_id = 1
        self.type = ['face', 'particular']
        self.label = ['Will', 'Imperial College London ID']
        self.owner = ['empty', 'Will']
        self.real = [True, True]
        self.confidence = [1.0, 1.0]
        self.upper_x = [435, 435]
        self.upper_y = [692, 435]
        self.lower_x = [1103, 435]
        self.lower_y = [18, 435]
        self.camera = [1, 1]


################################ END SET UP ###################################

############################### START TESTS ###################################

class TestHotword(unittest.TestCase):
    def setUp(self):
        pass

class TestRealParticipant(TestHotword):
    def test_makeRequest(self):
        real_p = RealParticipant()
        security_check = SecurityCheck('Will', real_p)
        result = security_check.can_see_person()
        self.assertEqual(result, True)

class TestFakeParticipant(TestHotword):
    def test_makeRequest(self):
        fake_p = FakeParticipant()
        security_check = SecurityCheck('Will', fake_p)
        security_check.can_see_person()
        result = security_check.person_is_real()
        self.assertEqual(result, False)

class IDCheck(TestHotword):
    def test_makeRequest(self):
        real_p = RealParticipantAndID()
        security_check = SecurityCheck('Will', real_p)
        result = security_check.id_check()
        self.assertEqual(result, True)


############################### END TESTS #####################################


############################### MAIN ##########################################

if __name__=="__main__":

    ####################### INITIALIZE YOUR NODES HERE ########################


    ####################### END OF SECTION ####################################

    unittest.main()
