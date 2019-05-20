#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Text To Speech.

File name: text_to_speech.py
Author: Nicolas Tobis
Date last modified: 14/05/2019
Python Version: 3.6
"""

import pandas as pd
import numpy as np
import getpass
import rospy

# Rospy Parameter for Robot Orientation and pointing
ACTIVE_CAMERA = 'active_camera'
DETECTED = 'detected'

USER = getpass.getuser()
base_path = "/home/" + USER + "/brain-cluster-ws/src/text_to_speech/scripts/"

TTS = pd.read_csv(base_path + "resources/tts.csv", delimiter='\t')

class TextToSpeech:
    def __init__(self, recog_elements, lang_code, input_faces = None, input_objects = None):
        self.df = pd.DataFrame()
        self.instruction_id = recog_elements.instruction_id
        self.df['type'] = pd.Series(recog_elements.type)
        self.df['label'] = recog_elements.label
        self.df['owner'] = recog_elements.owner
        self.df['real'] = recog_elements.real
        self.df['confidence'] = recog_elements.confidence
        self.df['camera'] = recog_elements.camera
        self.tts = TTS[TTS['lang_code'] == lang_code]
        self.faces = input_faces
        self.objects = input_objects
        self.commands = {1 : ['DETECT_OBJECT', self._detect_object],
                         2 : ['VERIFY_OBJECT', self._verify_object],
                         3 : ['DETECT_FACE', self._detect_face],
                         4 : ['VERIFY_FACE', self._verify_face],
                         5 : ['FIND_OWNER', self._find_owner],
                         6 : 'PERSON_OR_OBJECT',
                         7 : ['FIND_OBJECT', self._verify_object], # uses the same function
                         8 : ['FIND_PERSON', self._verify_face],}    # uses the same function
        self.normalize()


    def normalize(self):
        """
        Lowercase all inputs.

        Keyword arguments:
        """


        if not self.df['label'].empty:
            self.df['label'] = self.df['label'].str.lower()
        if self.faces is not None:
            [x.lower() for x in self.faces]
        if self.objects is not None:
            [x.lower() for x in self.objects]

    def callback(self):
        """
        Determining the specific audio out builder function to be called.

        Keyword arguments:
        """

        command = self.commands.get(self.instruction_id, lambda: "Invalid Code")
        code = command[0]
        function = command[1]
        return function(code)

    def _detect_object(self, code):
        """
        Build a sentence for detect object.

        Keyword arguments:
        code -- int
        """
        # Counting objects that are not people
        items = self.df[(self.df['type'] == 'object')]
        return self._build_sentence(code, items, items.shape[0])

    def _verify_object(self, code):
        """
        Build a sentence for verify object.

        Keyword arguments:
        code -- int
        """
        # Counting the object that was asked for
        items = self.df[(self.df['type'] == 'object') & (self.df['label'] == self.objects[0])]
        return self._build_sentence(code, items, items.shape[0], self.objects[0])

    def _detect_face(self, code):
        """
        Build a sentence for detect face.

        Keyword arguments:
        code -- int
        """
        # Counting objects that are not people
        items = self.df[(self.df['type'] == 'face')]
        return self._build_sentence(code, items, items.shape[0])

    def _verify_face(self, code):
        """
        Build a sentence for verifying a face.

        Keyword arguments:
        code -- int
        """

        # Counting the object that was asked for
        items = self.df[(self.df['type'] == 'face') & (self.df['label'] == self.faces[0])]
        return self._build_sentence(code, items, items.shape[0], self.faces[0])

    def _find_owner(self, code):
        """
        Build a sentence for finding an owner.

        Keyword arguments:
        code -- int
        """
        ask = self.objects[0]
        if len(self.df):
            items = self.df[(self.df['type'] == 'object') & (self.df['label'] == ask)]
            owned = self.df[(self.df['type'] == 'object') & (self.df['owner'] != 'empty') & (self.df['label'] == ask)]
        else:
            items = self.df
            owned = self.df
        items_count = items.shape[0]
        owned_count = owned.shape[0]
        if owned_count == 0:
            return self._build_sentence(code, items, owned_count, ask=ask)

        elif owned_count == 1:
            person = owned[owned['label'] == ask]['owner'].iloc[0]
            return self._build_sentence(code, items, owned_count, ask, person)

        else:
            person = owned[owned['label'] == ask]['owner']
            return self._build_sentence(code, items, owned_count, ask, person)



    def _build_sentence(self, code, items, count, ask=None, person=None):
        """
        Build the actual sentence

        Keyword arguments:
        code -- int
        count -- int (number of people/objects)
        ask -- string (person or object being asked for)
        person -- string (person supposed to own something)
        """

        # Build initial trunk of sentence
        real = 1 if items['real'].all() else 0
        trunk = self.tts[(self.tts['category'] == 'trunk') & (self.tts['code'] == code) & (self.tts['real'] == real)]

        # Sentence output based on table contents
        if count == 0:
            rospy.set_param(DETECTED, False)

            sentence = trunk[(trunk['count'] == count)]['component'].sample(1).iloc[0]
            return sentence.format(ask, items.shape[0])

        elif count == 1:
            if code == 'FIND_PERSON' or code == 'FIND_OBJECT':
                rospy.set_param(DETECTED, True)

            sentence = trunk[(trunk['count'] == count)]['component'].sample(1).iloc[0]
            confidence = int(round(items['confidence'].iloc[0]*100, 0))
            object_label = items['label'].iloc[0]
            camera = items['camera'].iloc[0]

            # Set global parameter for which camera detected the item
            rospy.set_param(ACTIVE_CAMERA, int(camera))

            position = self.tts[(self.tts['category'] == 'position') & (self.tts['code'] == code) & (self.tts['camera'] == int(camera))]['component']
            if not position.empty:
                position = position.iloc[0]
                return (sentence + " " + position).format(confidence, object_label, person)
            return (sentence).format(confidence, object_label, person)


        else:
            if code == 'FIND_PERSON' or code == 'FIND_OBJECT':
                rospy.set_param(DETECTED, True)

            # Prepare sentence components
            trunk = trunk[(trunk['count'] == 2)]['component'].sample(1).iloc[0]
            trunk_extension = ""
            snippet = self.tts[(self.tts['category'] == 'trunk_extension') & (self.tts['code'] == code) & (self.tts['count'] == 2)]['component'].sample(1).iloc[0]
            end = self.tts[(self.tts['category'] == 'end') & (self.tts['code'] == code) & (self.tts['count'] == 2) & (self.tts['real'] == real)]['component'].sample(1).iloc[0]

            # Create frequency table for objects
            object_labels = self.frequency_table('object')
            face_labels = self.frequency_table('faces')

            # Get relevant objects
            object_label = tuple(items['label'].values)

            # Decide case and build sentence
            if code == 'DETECT_OBJECT':
                for _ in range(len(object_labels) // 2 - 2):
                    trunk_extension += snippet
                sentence = trunk + trunk_extension
                if len(object_labels) > 2:
                    sentence += end
                object_label = tuple(object_labels)
                return sentence.format(*object_label)

            elif code == 'VERIFY_OBJECT':
                for _ in range(count - 1):
                    trunk_extension += snippet
                sentence = trunk + trunk_extension + end
                return sentence.format(object_label[0], count)

            elif code == 'DETECT_FACE':
                for _ in range(count - 1):
                    trunk_extension += snippet
                sentence = trunk + trunk_extension + end
                return sentence.format(*object_label)

            elif code == 'VERIFY_FACE':
                for _ in range(count - 1):
                    trunk_extension += snippet
                sentence = trunk + trunk_extension + end
                return sentence.format(object_label[0], count)

            elif code == 'FIND_OWNER':
                owners = tuple(person.values)
                for _ in range(count - 1):
                    trunk_extension += snippet
                sentence = trunk + trunk_extension + end
                return sentence.format(ask, *owners)

            elif code == 'FIND_OBJECT':
                for _ in range(count - 1):
                    trunk_extension += snippet
                sentence = trunk + trunk_extension + end
                return sentence.format(object_label[0], count)

            elif code == 'FIND_PERSON':
                for _ in range(count - 1):
                    trunk_extension += snippet
                sentence = trunk + trunk_extension + end
                return sentence.format(object_label[0], count)

            else:
                return "UNKNOWN COMMAND"

    def frequency_table(self, type):
        """
        Calculate the number of people and objects seen

        Keyword arguments:
        type -- string (object or face) 
        """

        df = pd.value_counts(self.df[self.df['type'] == type]['label']).to_frame().reset_index()
        df.columns = ['object', 'count']
        labels = []
        for index, row in df.iterrows():
            labels.append(row[1])
            if row[1] > 1:
                labels.append(row[0] + 's')
            else:
                labels.append(row[0])

        return labels
