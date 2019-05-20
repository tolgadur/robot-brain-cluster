#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Speech Rec Service.

File name: speech_rec_service.py
Author: Nicolas Tobis
Date last modified: 14/05/2019
Python Version: 3.6
"""

import sys
import os

import rospy
import speech_recognition as sr

from speech_rec.srv import GetSentence

class SpeechRecService:

    def __init__(self, file=None):
        self.file = None
        if file is not None:
            self.file = file


    def translate_speech(self, request):
        
        # return 'can you see a person'
        # Start the recognizer
        r = sr.Recognizer()

        # Select microphone
        mic = sr.Microphone()


        # Adjusting for ambient noise
        with mic as source:
            r.adjust_for_ambient_noise(source)
        print "Set minimum energy threshold to {}".format(r.energy_threshold)

        try:
            # Capture audio and interpret
            connection_error = 0
            while connection_error < 5:
                try:

                    print("Say something!")
                    if self.file is not None:
                        mic = sr.AudioFile(self.file)
                        with mic as source:
                            audio = r.record(source)
                    else:
                        with mic as source:
                            audio = r.listen(source)
                    print("Got it! Now to recognize it...")

                    # Recognize speech using Google Speech Recognition
                    value = r.recognize_google(audio, language=request.language_code)
                    print value
                    # value ="Hello"
                    print "You said {}".format(value)
                    return value

                except sr.UnknownValueError:
                    print("Oops! Didn't catch that")
                    if self.file is not None:
                        break
                except sr.RequestError as e:
                    print("Uh oh! Couldn't request results from Google Speech Recognition service; {0}".format(e))
                    connection_error += 1
                    # time.sleep(3)
        except KeyboardInterrupt: # pragma: no cover
            print("\nExiting program.")
            sys.exit(-1)


if __name__ == "__main__": # pragma: no cover
    rospy.init_node('speech_recognition_server')
    speech_server = SpeechRecService()
    s = rospy.Service('speech_recognition', GetSentence, speech_server.translate_speech)
    rospy.spin()
