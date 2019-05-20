#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Audio Out Service.

File name: audio_out_service.py
Author: Nicolas Tobis
Date last modified: 14/05/2019
Python Version: 3.6
"""

# Import Packages
from google.cloud import texttospeech
import os
import sys
import rospy
import pyaudio
import wave
import getpass
import numpy as np
from threading import Thread

# Rospy Parameter
TRAINING_DONE = 'training_done'
FACE_TRAINING_ACTIVE = 'face_training_active'

# Getting the username of a given pc user
USER = getpass.getuser()

# Google credentials
root_path = "/home/" + USER + "/brain-cluster-ws/"
sys.path.append(os.path.join(root_path, "src/baxter_display/scripts"))
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = root_path + "src/audio_out/GoogleAuthentication.json"

# Import ROS components
from audio_out.srv import PlaySentence
from display_controller import DisplayController

class AudioOutService(object):
    """
    AudioOut Service responsible for turning text into audio-waves
    """

    def __init__(self, text=None, lang_code=None, in_file='output.wav'):
        self.text = text
        self.lang_code = lang_code
        self.file = in_file


    def text_to_speech_driver(self, request):
        """
        Service Call, also signaling a information to the display

        Keyword arguments:
        request -- ROS Message
        """

        # Launch Display controller
        display_controller = DisplayController()

        # Convert string to audio file
        self.text_to_speech(request)

        # Get duration of audio out
        duration = self.get_duration()

        # List for Multi-Threading
        threads = []

        # Start Video Thread
        if not rospy.get_param(FACE_TRAINING_ACTIVE):
            video_thread = Thread(target=display_controller.play_video, args=[duration])
            video_thread.start()
            threads.append(video_thread)

        # Start Audio Thread
        audio_thread = Thread(target=self.play_audio)
        audio_thread.start()
        threads.append(audio_thread)

        for process in threads:
            process.join()

        return True

    def text_to_speech(self, request):
        """
        API Call

        Keyword arguments:
        request -- ROS Message
        """

        # Instantiates a client
        client = texttospeech.TextToSpeechClient()
        # Set the text input to be synthesized
        synthesis_input = texttospeech.types.SynthesisInput(text=request.text)

        # Build the voice request, select the language code ("en-US") and the ssml
        # voice gender ("neutral")
        voice = texttospeech.types.VoiceSelectionParams(
            language_code=request.language_code,
            ssml_gender=texttospeech.enums.SsmlVoiceGender.MALE)

        # Select the type of audio file you want returned
        audio_config = texttospeech.types.AudioConfig(
            audio_encoding=texttospeech.enums.AudioEncoding.LINEAR16)

        # Perform the text-to-speech request on the text input with the selected
        # voice parameters and audio file type
        response = client.synthesize_speech(synthesis_input, voice, audio_config)

        # The response's audio_content is binary.
        with open(self.file, 'wb') as out:
            # Write the response to the output file.
            out.write(response.audio_content)
            print('Audio content written to file "output.wav"')


    def play_audio(self):
        """
        Play Audio

        Keyword arguments:
        """

        # length of data to read.
        chunk = 1024

        '''
        ************************************************************************
              This is the start of the "minimum needed to read a wave"
        ************************************************************************
        '''
        # open the file for reading.
        wf = wave.open(self.file, 'rb')

        # create an audio object
        p = pyaudio.PyAudio()

        # open stream based on the wave object which has been input.
        stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                        channels=wf.getnchannels(),
                        rate=wf.getframerate(),
                        output=True)

        # read data (based on the chunk size)
        data = wf.readframes(chunk)

        # play stream (looping from beginning of file to the end)
        count = 0
        while data:
            # writing to the stream is what *actually* plays the sound.
            stream.write(data)
            data = wf.readframes(chunk)
            count += 1
            # Only checking every 10 frames to ensure audio quality
            if not count % 10:
                if rospy.get_param(TRAINING_DONE):
                    break

        # cleanup stuff.
        stream.close()
        p.terminate()
        wf.close()

    def get_duration(self):
        wf = wave.open(self.file, 'rb')
        frame_rate = wf.getframerate()
        frames = wf.getnframes()
        wf.close()
        return frames / float(frame_rate)



if __name__ == "__main__": # pragma: no cover
    rospy.init_node('audio_out_server')
    audio_out_server = AudioOutService()
    s = rospy.Service('audio_out', PlaySentence, audio_out_server.text_to_speech_driver)
    rospy.spin()
