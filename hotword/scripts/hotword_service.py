#!/usr/bin/env python

"""
Hotword Service.

File name: hotword_service.py
Author: Nicolas Tobis
Date last modified: 14/05/2019
Python Version: 3.6
"""

import sys
import os
import getpass

import rospy
try:
    import snowboydecoder
except:
    from snowboy import snowboydecoder

from hotword.srv import LangCode

USER = getpass.getuser()
base_path = "/home/" + USER + "/brain-cluster-ws/"
model_path = base_path + "src/hotword/scripts/resources/models/"

class HotwordService:
    """
    Hotword Recognition Service

    Keyword arguments:
    """


    def __init__(self, file=None):
        self.file = file


    def detect_hotword(self, request = None):
        """
        Detecting a hotword from a model

        Keyword arguments:
        request = ROS Service Message
        """
        # Setting parameters
        models = {
            'en-EN' : model_path + "HeyDeNiro.pmdl",
            'de-DE' : model_path + "HalloDeNiro.pmdl",
            'es-ES' : model_path + "HolaDeNiro.pmdl",
            'it-IT' : model_path + "CiaoDeNiro.pmdl",
            'nl-NL' : model_path + "HoiDeNiro.pmdl"
        }
        sensitivity = [0.5]

        # Initializing Snowboy (Hotword Detector)
        detector = snowboydecoder.HotwordDetector(models[request.lang_in], sensitivity=sensitivity)

        try:
            detector.start(sleep_time=0.03, file=self.file)
        except KeyboardInterrupt: # pragma: no cover
            pass
        detector.terminate()
        print "Returning: " + str(request.lang_in)
        return request.lang_in

    def detect_language(self, request = None):
        """
        Detecting a hotword from a model, returning a language code

        Keyword arguments:
        request = ROS Service Message
        """
        # Setting parameters
        languages = ['en-EN', 'de-DE', 'es-ES', 'it-IT', 'nl-NL']
        models = [model_path + "HeyDeNiro.pmdl",
                  model_path + "HalloDeNiro.pmdl",
                  model_path + "HolaDeNiro.pmdl",
                  model_path + "CiaoDeNiro.pmdl",
                  model_path + "HoiDeNiro.pmdl"]

        sensitivity = [0.5]*len(models)

        # Initializing Snowboy (Hotword Detector)
        detector = snowboydecoder.HotwordDetector(models, sensitivity=sensitivity)

        try:
            lang = [None]
            callbacks = [lambda i=i: _return_language(languages[i], lang) for i, val in enumerate(models)]
            detector.start(detected_callback=callbacks, sleep_time=0.03, file=self.file)
        except KeyboardInterrupt: # pragma: no cover
            pass
        detector.terminate()
        print "Returning: " + str(lang[0])
        return lang[0]

def _return_language(in_lang, out_lang):
    """
    Preparing ROS message for service call

    Keyword arguments:
    in_lang = string
    out_lang = string
    """

    out_lang[0] = in_lang

if __name__ == "__main__": # pragma: no cover
    print "Running Hotword Recognition"
    rospy.init_node('hotword_recognition_server')
    hotword_server = HotwordService()
    s = rospy.Service('hotword_recognition', LangCode, hotword_server.detect_hotword)
    rospy.spin()
