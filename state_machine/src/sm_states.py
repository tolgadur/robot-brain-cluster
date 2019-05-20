#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
State Machine.

File name: state_machine.py
Author: Nicolas Tobis
Date last modified: 14/05/2019
Python Version: 3.6
"""

import getpass
import os
# Imports
import sys
from threading import Thread

# Import additional packages
import pandas as pd
# Import ROS packages
import rospy
import smach

# Set Path Variables
USER = getpass.getuser()
root_path = "/home/" + USER + "/brain-cluster-ws/"
sys.path.append(os.path.join(root_path, "src/hotword/scripts"))
sys.path.append(os.path.join(root_path, "src/speech_rec/scripts"))
sys.path.append(os.path.join(root_path, "src/interpreter/scripts"))
sys.path.append(os.path.join(root_path, "src/environment/scripts"))
sys.path.append(os.path.join(root_path, "src/audio_out/scripts"))
sys.path.append(os.path.join(root_path, "src/text_to_speech/scripts"))
sys.path.append(os.path.join(root_path, "src/face_training/scripts"))
sys.path.append(os.path.join(root_path, "src/pointing/scripts"))
sys.path.append(os.path.join(root_path, "src/baxter_display/scripts"))
sys.path.append(os.path.join(root_path, "src/deniro-mobile-base/Emilk_Project/bosch_imu_emilk/scripts"))
sys.path.append(os.path.join(root_path, "src/security_check/scripts"))

# Client classes
from hotword_client import HotwordClient
from hotword_service import HotwordService
from speech_rec_client import SpeechRecClient
from interpreter_client import InterpreterClient
from environment_client import EnvironmentClient
from audio_out_client import AudioOutClient
from audio_out_service import AudioOutService
from text_to_speech import TextToSpeech
from face_training_client import FaceTrainingClient
from pointing_client import PointingClient
from display_controller import DisplayController
from rotation_client import RotationClient
from security_check import SecurityCheck

# In try block so it doesn't need to import on a personal computer
try:
    from tuck_arms import Tuck
except:
    pass

# Defining parameter keys
DEFAULT_LANG = 'en-EN'
TRANSITION = 'transition'
STOP_THREAD = 'stop_thread'
ACTIVE_CAMERA = 'active_camera'
FACE_TRAINING_ACTIVE = 'face_training_active'
TRAINING_DONE = 'training_done'
POINTING_COMPLETE = 'pointing_complete'
DISPLAY_INFORMATION = 'display_information'
DETECTED = 'detected'
RECOG_UPPER_X = 'recog_upper_x'
RECOG_UPPER_Y = 'recog_upper_y'
RECOG_LOWER_X = 'recog_lower_x'
RECOG_LOWER_Y = 'recog_lower_y'
RECOG_LABEL = 'recog_label'
PERSON_IDENTIFIED = 'person_identified'
RECOG_CAMERA = 'recog_camera'

# Getting correct routing based on special_code
INSTRUCTIONS = {
    -1: "INVALID_INSTRUCTION_ERROR",
    -2: "NO_OBJECT_SPECIFIED",
    -3: "NO_NAME_SPECIFIED",
    -4: "INVALID_OBJECT",
    -5: "INVALID_NAME",
    -6: "CHANGE_LANGUAGE",
    -7: "FACE_TRAINING",
    -8: "ACTIVATE",
    -9: "PASS_MESSAGE",
    -10: "DELIVER_MESSAGE"
}

# Instantiating Audio Out Client
audio_out_client = AudioOutClient()

# For display sending
rospy.set_param(STOP_THREAD, False)

# Setting the default camera
rospy.set_param(ACTIVE_CAMERA, 1)

# Setting Training to False
rospy.set_param(TRAINING_DONE, False)
rospy.set_param(FACE_TRAINING_ACTIVE, False)

# Pointing status
rospy.set_param(POINTING_COMPLETE, False)

# Detected something
rospy.set_param(DETECTED, False)

# Setting initial recog_elements to Empty Array
rospy.set_param(RECOG_UPPER_X, [])
rospy.set_param(RECOG_UPPER_Y, [])
rospy.set_param(RECOG_LOWER_X, [])
rospy.set_param(RECOG_LOWER_Y, [])
rospy.set_param(RECOG_LABEL, [])
rospy.set_param(RECOG_CAMERA, [])

# Setting initial display information to false
rospy.set_param(DISPLAY_INFORMATION, False)

# Setting global check for identifying a person in the security check to false
rospy.set_param(PERSON_IDENTIFIED, False)


# Load Dictionary
df = pd.read_csv(root_path + "src/state_machine/src/resources/speech_db.csv", delimiter="\t")


class Idling(smach.State):
    """
    Idling state playing welcome message

    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['hotword_recognition'], output_keys=['lang_code_out'])
        self.state = 'idling'
        self.output_msg = "Idling"

    def execute(self, userdata):
        """
        Execute function called in the state machine

        Key arguments:
        userdata -- state machine userdata object being passed around

        """

        rospy.logdebug("Welcome message")
        rospy.loginfo(self.output_msg + "\n")

        # Audio Out
        userdata.lang_code_out = DEFAULT_LANG
        self.output_msg = df[(df['Type'] == self.state) & (df['Language'] == DEFAULT_LANG)]['Message'].iloc[0]
        # audio_out_client.make_request(self.output_msg, DEFAULT_LANG)

        return 'hotword_recognition'


class HotwordRecognition(smach.State):
    """
    Hotword Recognition State waiting to detect a Hotword

    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['listening'], input_keys=['lang_code_in'], output_keys=['lang_code_out'])
        self.state = 'hotword_recognition'
        self.output_msg = "Recognizing Hotword"

    def execute(self, userdata):
        """
        Execute function called in the state machine

        Key arguments:
        userdata -- state machine userdata object being passed around

        """
        rospy.logdebug("Recognizing Hotword")
        rospy.loginfo(self.output_msg + "\n")

        hotword_client = HotwordClient()

        # Start Video Thread
        threads = []
        display_controller = DisplayController()
        video_thread = Thread(target=display_controller.play_video, args=(None, True))
        video_thread.start()
        threads.append(video_thread)

        # Request hotword
        lang_code = hotword_client.make_request(userdata.lang_code_in)

        # Terminate Video Thread
        rospy.set_param(STOP_THREAD, True)
        for process in threads:
            process.join()
        rospy.set_param(STOP_THREAD, False)

        # Defining current language
        userdata.lang_code_out = lang_code

        return 'listening'


class Listening(smach.State):
    """
    Listening State Tranlating speech into text strings

    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['interpretation'],
                             input_keys=['lang_code_in'],
                             output_keys=['sentence_out'])

        self.state = 'listening'
        self.output_msg = "Listening"

    def execute(self, userdata):
        """
        Execute function called in the state machine

        Key arguments:
        userdata -- state machine userdata object being passed around

        """
        rospy.logdebug("Recognizing Sentence")
        rospy.loginfo(self.output_msg + "\n")

        # Audio out
        self.output_msg = \
            df[(df['Type'] == self.state) & (df['Language'] == userdata.lang_code_in)]['Message'].sample(1).iloc[0]
        audio_out_client.make_request(self.output_msg, userdata.lang_code_in)

        # Start Video Thread
        threads = []
        display_controller = DisplayController()
        video_thread = Thread(target=display_controller.play_video, args=(None, True))
        video_thread.start()
        threads.append(video_thread)

        # Recognizing speech
        speech_client = SpeechRecClient()
        sentence = speech_client.make_request(userdata.lang_code_in)

        # Terminate Video Thread
        rospy.set_param(STOP_THREAD, True)
        for process in threads:
            process.join()
        rospy.set_param(STOP_THREAD, False)

        userdata.sentence_out = sentence
        return 'interpretation'


class Interpretation(smach.State):
    """
    Listening State waiting to detect a Hotword

    """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['hotword_recognition', 'listening', 'recognition', 'face_training'],
                             input_keys=['sentence_in', 'lang_code_in'],
                             output_keys=['action_out', 'name_out', 'lang_code_out', 'receiver', 'message'])
        self.output_msg = "Interpreting"
        self.state = 'interpretation'

    def execute(self, userdata):
        """
        Execute function called in the state machine

        Key arguments:
        userdata -- state machine userdata object being passed around

        """
        rospy.logdebug("Translating Sentence")
        rospy.loginfo(self.output_msg + "\n")

        # Audio out
        self.output_msg = \
            df[(df['Type'] == self.state) & (df['Language'] == userdata.lang_code_in)]['Message'].sample(1).iloc[0]
        audio_out_client.make_request(self.output_msg, userdata.lang_code_in)

        # Interpreting user input
        interpeter_client = InterpreterClient()
        action = interpeter_client.make_request(userdata.sentence_in, userdata.lang_code_in)
        special_code = INSTRUCTIONS.get(action[0], False)

        # If multiple elements in the yolo database match the requested item
        if len(action[3]) > 1:
            conflict_index = self._ask_for_clarification(action, userdata.lang_code_in)
            action = (action[0], action[1], [action[3][conflict_index]])

        # Follow the request
        userdata.action_out = action

        # If the request is not a straight forward request
        if special_code:
            return self._special_code_handling(special_code, userdata.lang_code_in, userdata)

        return 'recognition'

    def _ask_for_clarification(self, action, lang_code):
        """
        Function called if the object asked for is not clear (e.g. ball instead
        of sports ball and base ball)

        Key arguments:
        userdata -- state machine userdata object being passed around

        """

        speech_client = SpeechRecClient()

        while True:
            self.output_msg = \
                df[(df['Type'] == 'FEEDBACK_NEEDED') & (df['Language'] == lang_code)]['Message'].sample(1).iloc[0]

            # Enlist all available choices
            for word in action[3]:
                self.output_msg = self.output_msg + word + ", "
            audio_out_client.make_request(self.output_msg, lang_code)

            # Take input and see if it matches, else repeat again
            new_request = speech_client.make_request(lang_code)
            for index, word in enumerate(action[3]):
                if word.replace(" ", "") == new_request.replace(" ", ""):
                    return index

    def _special_code_handling(self, special_code, lang_code, userdata):
        """
        Function called when an error is coming up or a special code is called

        Key arguments:
        special_code -- int, indicating a particular flow of control
        lang_code -- string
        userdata -- state_machine userdata

        """
        self.output_msg = df[(df['Type'] == special_code) & (df['Language'] == lang_code)]['Message'].sample(1).iloc[0]
        audio_out_client.make_request(self.output_msg, lang_code)

        if special_code == 'CHANGE_LANGUAGE':
            hotword = HotwordService()
            userdata.lang_code_out = hotword.detect_language()
            return 'listening'

        elif special_code == 'FACE_TRAINING':
            return 'face_training'

        elif special_code == 'ACTIVATE':
            # Untuck Arms
            tuck = Tuck(False)
            tuck.supervised_tuck()
            return 'hotword_recognition'

        elif special_code == 'PASS_MESSAGE':

            response = ''
            speech_client = SpeechRecClient()
            while response.lower() != 'yes':
                name = speech_client.make_request(lang_code)

                # Is that name correct?
                output_msg = \
                    df[(df['Type'] == 'FACE_VERIFICATION') & (df['Language'] == userdata.lang_code_in)]['Message'].sample(
                        1).iloc[0]
                output_msg = name + ". " + output_msg
                audio_out_client.make_request(output_msg, userdata.lang_code_in)
                response = speech_client.make_request(userdata.lang_code_in)
                userdata.receiver = name
                if response != 'yes':
                    audio_out_client.make_request(self.output_msg, lang_code)


            self.output_msg = \
                df[(df['Type'] == 'ASK_MESSAGE') & (df['Language'] == lang_code)]['Message'].sample(1).iloc[0]
            audio_out_client.make_request(self.output_msg, lang_code)
            userdata.message = speech_client.make_request(lang_code)
            self.output_msg = \
                df[(df['Type'] == 'CONFIRM_MESSAGE') & (df['Language'] == lang_code)]['Message'].sample(1).iloc[0]
            audio_out_client.make_request(self.output_msg, lang_code)
            return 'hotword_recognition'

        elif special_code == 'DELIVER_MESSAGE':
            return 'recognition'

        else:
            return 'listening'


class Recognition(smach.State):
    """
    Recognition state querying the environment model for its current state of the world

    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['message_delivering', 'information'],
                             input_keys=['action_in', 'lang_code_in'],
                             output_keys=['recog_elements_out', 'direction_out', 'angle_out'])
        self.output_msg = "Recognizing"
        self.state = 'recognition'

    def execute(self, userdata):
        """
        Execute function called in the state machine

        Key arguments:
        userdata -- state machine userdata object being passed around

        """

        rospy.logdebug(self.output_msg + "\n")
        rospy.loginfo(self.output_msg + "\n")

        # Get information about previous inquiry
        environment_client = EnvironmentClient()
        recog_elements = environment_client.make_request(userdata.action_in)
        rospy.loginfo(recog_elements)
        userdata.recog_elements_out = recog_elements

        rospy.loginfo(self.output_msg + "\n")

        # Check for message delivery
        special_code = INSTRUCTIONS.get(recog_elements.instruction_id, False)
        if special_code == 'DELIVER_MESSAGE':
            return 'message_delivering'

        return 'information'


class Information(smach.State):
    """
    Information state outputting what DE NIRO has learned from the recognition state.
    This state controls the audio out streams as well as the display

    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['turning'], input_keys=['recog_elements_in', 'action_in', 'lang_code_in'])
        self.output_msg = "Providing Information"
        self.state = 'information'

    def execute(self, userdata):
        """
        Execute function called in the state machine

        Key arguments:
        userdata -- state machine userdata object being passed around

        """
        rospy.logdebug("Information")
        rospy.loginfo(self.output_msg + "\n")
        rospy.loginfo(userdata.recog_elements_in)

        # Prepare sentence and get information on the camera
        rospy.set_param(ACTIVE_CAMERA, 1)  # Default front camera (changed by audio request)
        text_to_speech = TextToSpeech(userdata.recog_elements_in, userdata.lang_code_in, userdata.action_in[1],
                                      userdata.action_in[2])
        self.output_msg = text_to_speech.callback()

        # Indicating to the Display Controller that the display should show what the robot thinks
        rospy.set_param(RECOG_UPPER_X, userdata.recog_elements_in.upper_x)
        rospy.set_param(RECOG_UPPER_Y, userdata.recog_elements_in.upper_y)
        rospy.set_param(RECOG_LOWER_X, userdata.recog_elements_in.lower_x)
        rospy.set_param(RECOG_LOWER_Y, userdata.recog_elements_in.lower_y)
        rospy.set_param(RECOG_LABEL, userdata.recog_elements_in.label)
        rospy.set_param(RECOG_CAMERA, userdata.recog_elements_in.camera)
        rospy.set_param(DISPLAY_INFORMATION, True)

        # Maintain the thread for the duration of the speech out
        audio_out_client.make_request(self.output_msg, userdata.lang_code_in)

        # Resetting Parameters
        rospy.set_param(RECOG_UPPER_X, [])
        rospy.set_param(RECOG_UPPER_Y, [])
        rospy.set_param(RECOG_LOWER_X, [])
        rospy.set_param(RECOG_LOWER_Y, [])
        rospy.set_param(RECOG_LABEL, [])
        rospy.set_param(RECOG_CAMERA, [])
        rospy.set_param(DISPLAY_INFORMATION, False)
        return 'turning'


class FaceTraining(smach.State):
    """
    Face training state.
    This state controls audio out, display, as well as face recognition nodes.

    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['face_training'], input_keys=['lang_code_in', 'name_in'],
                             output_keys=[True])
        self.output_msg = "Training new face"
        self.state = 'face_training'

    def execute(self, userdata):
        """
        Execute function called in the state machine

        Key arguments:
        userdata -- state machine userdata object being passed around

        """
        rospy.logdebug("Translating Sentence")
        rospy.loginfo(self.output_msg + "\n")

        # Get your name to save your face
        speech_client = SpeechRecClient()
        response = ''
        name = ''
        while response.lower() != 'yes':
            # What is your name?
            self.output_msg = \
                df[(df['Type'] == 'FACE_NAME') & (df['Language'] == userdata.lang_code_in)]['Message'].sample(1).iloc[0]
            audio_out_client.make_request(self.output_msg, userdata.lang_code_in)
            name = speech_client.make_request(userdata.lang_code_in)

            # Is that name correct?
            self.output_msg = \
                df[(df['Type'] == 'FACE_VERIFICATION') & (df['Language'] == userdata.lang_code_in)]['Message'].sample(
                    1).iloc[0]
            self.output_msg = name + ". " + self.output_msg
            audio_out_client.make_request(self.output_msg, userdata.lang_code_in)
            response = speech_client.make_request(userdata.lang_code_in)

        response = ''
        # Start Video Feed Thread
        threads = []
        display_controller = DisplayController()
        rospy.set_param(FACE_TRAINING_ACTIVE, True)
        video_thread = Thread(target=display_controller.play_video, args=(None, True))
        video_thread.start()
        threads.append(video_thread)

        # Ask for confirmation, when in video frame
        while response.lower() == '':
            self.output_msg = df[(df['Type'] == 'FACE_IN_FRAME') & (df['Language'] == userdata.lang_code_in)]['Message'].sample(1).iloc[0]
            audio_out_client.make_request(self.output_msg, userdata.lang_code_in)
            response = speech_client.make_request(userdata.lang_code_in)

        # Launch Face Training
        face_client = FaceTrainingClient()
        audio_out = AudioOutService(in_file=root_path + 'src/state_machine/src/resources/elevator.wav')
        face_client.make_request(name)
        audio_out.play_audio()
        rospy.set_param(TRAINING_DONE, False)

        # Terminate Video Feed Thread
        rospy.set_param(FACE_TRAINING_ACTIVE, False)
        for process in threads:
            process.join()



        # Audio out
        self.output_msg = \
            df[(df['Type'] == 'FACE_TRAINING_DONE') & (df['Language'] == userdata.lang_code_in)]['Message'].sample(
                1).iloc[
                0]
        audio_out_client.make_request(self.output_msg, userdata.lang_code_in)

        return 'face_training'


class Turning(smach.State):
    """
    Turning state. This state makes the robot turn if objects are detected
    behind it and also points in a given direction

    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['hotword_recognition'], input_keys=['lang_code_in'], output_keys=[True])
        self.output_msg = "Turning"
        self.state = 'turning'

    def execute(self, userdata):
        """
        Execute function called in the state machine

        Key arguments:
        userdata -- state machine userdata object being passed around

        """
        rospy.logdebug("Turning")
        rospy.loginfo(self.output_msg + "\n")

        # Turn in a given direction by 120 degrees
        direction, angle = self._determine_direction()
        rotation_client = RotationClient()
        rotation_client.make_request(direction, angle)

        # Pointing
        if rospy.get_param(DETECTED):
            pointing_client = PointingClient()
            if rospy.get_param(ACTIVE_CAMERA) == 0:
                filename = root_path + "src/state_machine/src/resources/joint_position_left.txt"
            else:
                filename = root_path + "src/state_machine/src/resources/joint_position_right.txt"
            pointing_client.make_request(filename)

            while not rospy.get_param(POINTING_COMPLETE):
                pass
        rospy.set_param(DETECTED, False)

        return 'hotword_recognition'

    @staticmethod
    def _determine_direction():
        """
        Determine the direction in which DE NIRO should turn

        """

        camera = rospy.get_param(ACTIVE_CAMERA)
        if camera == 0:
            return 'left', 120
        elif camera == 1:
            return 'left', 0
        else:
            return 'right', 120


class MessageDelivering(smach.State):
    """
    Message Delivering state. This state runs checks (see security_check), if a
    given message should be delivered to an individual

    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['hotword_recognition', 'recognition'],
                             input_keys=['receiver', 'message', 'recog_elements_in', 'lang_code_in'])
        self.output_msg = "Delivering Message"
        self.state = 'message_delivering'

    def execute(self, userdata):
        """
        Execute function called in the state machine

        Key arguments:
        userdata -- state machine userdata object being passed around

        """
        rospy.logdebug("Delivering Message")
        rospy.loginfo(self.output_msg + "\n")

        # Perform security check
        speech_client = SpeechRecClient()
        security_check = SecurityCheck(userdata.receiver, userdata.recog_elements_in)

        # Global Check if person has previously been identified
        if not rospy.get_param(PERSON_IDENTIFIED):

            # Check 1: Is the person visible
            if security_check.can_see_person():
                pass
            else:
                # Can't see the required person
                self.output_msg = \
                    df[(df['Type'] == 'NO_PERSON') & (df['Language'] == userdata.lang_code_in)]['Message'].sample(1).iloc[0]
                self.output_msg = self.output_msg.format(userdata.receiver)
                audio_out_client.make_request(self.output_msg, userdata.lang_code_in)

                # Await response
                sentence = speech_client.make_request(userdata.lang_code_in)
                if sentence.lower() == 'yes':
                    return 'recognition'
                else:
                    return 'hotword_recognition'

            # Check 2: Is the person real
            if security_check.person_is_real():
                rospy.set_param(PERSON_IDENTIFIED, True)
                pass
            else:
                # Spoofing detection
                self.output_msg = \
                    df[(df['Type'] == 'NOT_REAL') & (df['Language'] == userdata.lang_code_in)]['Message'].sample(1).iloc[0]
                self.output_msg = self.output_msg.format(userdata.receiver)
                audio_out_client.make_request(self.output_msg, userdata.lang_code_in)

                # Await response
                sentence = speech_client.make_request(userdata.lang_code_in)
                if sentence == 'yes':
                    return 'recognition'
                else:
                    return 'hotword_recognition'


        # Check 3: Is the ID presented (Skip here, if the person has previously been correctly identified)
        if security_check.id_check():
            # You have a new message
            self.output_msg = \
                df[(df['Type'] == 'PLAY_MESSAGE') & (df['Language'] == userdata.lang_code_in)]['Message'].sample(
                    1).iloc[0]
            audio_out_client.make_request(self.output_msg, userdata.lang_code_in)

            self.output_msg = userdata.message
            audio_out_client.make_request(self.output_msg, userdata.lang_code_in)

        else:
            # Show Imperial College ID
            self.output_msg = \
                df[(df['Type'] == 'NO_ID') & (df['Language'] == userdata.lang_code_in)]['Message'].sample(1).iloc[0]
            audio_out_client.make_request(self.output_msg, userdata.lang_code_in)
            return 'recognition'


        # Audio out
        self.output_msg = "Message Delivery Complete. Demo Over, De Niro Out."
        audio_out_client.make_request(self.output_msg, userdata.lang_code_in)

        # Tuck Arms
        try:
            tuck = Tuck(True)
            tuck.supervised_tuck()
        except:
            pass

        return 'hotword_recognition'
