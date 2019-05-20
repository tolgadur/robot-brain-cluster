import subprocess
import os
import string
import nltk as nl
import getpass
import pandas as pd
from nltk.corpus import stopwords

DETECT_OBJECT = 1
VERIFY_OBJECT = 2
DETECT_FACE = 3
VERIFY_FACE = 4
FIND_OWNER = 5
PERSON_OR_OBJECT = 6
FIND_OBJECT = 7
FIND_PERSON = 8

INSTRUCTION_ID = 0
FACE = 1
OBJECT = 2
OBJECT_CONFLICTS = 3


NO_ERROR = 0
INVALID_INSTRUCTION_ERROR = -1
NO_OBJECT_SPECIFIED = -2
NO_NAME_SPECIFIED = -3
INVALID_OBJECT = -4
INVALID_NAME = -5
CHANGE_LANGUAGE = -6
FEEDBACK_NEEDED = -99


USER = getpass.getuser()
base_path = "/home/" + USER + "/brain-cluster-ws/src/interpreter/scripts/"
# base_path = ""

class SpeechInterpreter:
    def __init__(self, voice_input, lang_code):
            self.voice_input = voice_input.lower()
            self.input_size = len(self.voice_input)
            self.metadata = None
            self.interpreted = []
            for i in range (4):
                self.interpreted.append([])
            self.ID = {}
            self.statements = {}

            df = pd.read_csv(base_path + "resources/commands.csv", delimiter=",")
            self.df = df[df.Language == lang_code]
            self.create_statements()

    def get_msg(self):
        print(self.interpreted)
        return self.interpreted

    def create_statements(self):
        for index, row in self.df.iterrows():
            self.statements[row['Command']] = row['ID']

    def consolidate_input(self):
        for (index, row) in self.df.iterrows():
            s = row['Command']
            if s in self.voice_input:
                if row['ID'] == CHANGE_LANGUAGE:
                    return CHANGE_LANGUAGE

                self.interpreted[int(INSTRUCTION_ID)].append(self.statements[s])

                s = self.voice_input.replace(s,'')
                break

        if not self.interpreted[INSTRUCTION_ID]:
            print("Sorry, I can't understand what you want me to do!")
            return INVALID_INSTRUCTION_ERROR

        faces = set(line.strip() for line in open(base_path + 'facedb.txt'))
        objects = set(line.strip() for line in open(base_path + 'cocodb.txt'))

        complete_matches = []
        for o in objects:
            if o in s:
                complete_matches.append(o)
                s = s.replace(o,'')

        #print(complete_matches)
        tokens = nl.word_tokenize(s)
        stop_words = set(stopwords.words('english'))
        filtered = [w for w in tokens if not w in stop_words]
        print(filtered)

        if self.interpreted[INSTRUCTION_ID] == [VERIFY_OBJECT] or self.interpreted[INSTRUCTION_ID] == [FIND_OWNER] or self.interpreted[INSTRUCTION_ID] == [FIND_OBJECT]:
            #self.interpreted[FACE].append([-1])
            if not filtered and not complete_matches:
                print("No objects specified for me to verify!")
                return NO_OBJECT_SPECIFIED
            conflicts = self.add_items(complete_matches,objects,self.interpreted,OBJECT)
            conflicts += self.add_items(filtered, objects,self.interpreted,OBJECT)
            self.interpreted[OBJECT_CONFLICTS] = conflicts
            if not self.interpreted[OBJECT] and not self.interpreted[OBJECT_CONFLICTS]:
                print("I don't know what that object is!")
                return INVALID_OBJECT

        elif self.interpreted[0] == [VERIFY_FACE] or self.interpreted[0] == [FIND_PERSON]:
            if not filtered:
                print("No names specified for me to verify!")
                return NO_NAME_SPECIFIED
            self.add_items(filtered, faces,self.interpreted,FACE)
            #self.interpreted[OBJECT].append([-1])
            if not self.interpreted[FACE]:
                print("I don't know who that is!")
                return INVALID_NAME

        else:
            self.add_items(filtered, faces,self.interpreted,FACE)
            self.add_items(filtered, objects,self.interpreted,OBJECT)

        for i in range(len(self.interpreted)):
            temp = set(self.interpreted[i])
            new = list(temp)
            self.interpreted[i] = new

        if self.interpreted[OBJECT_CONFLICTS]:
            print("There is a conflict")
            return FEEDBACK_NEEDED
        else:
            self.interpreted[OBJECT] += self.interpreted[OBJECT_CONFLICTS]
            self.interpreted[OBJECT_CONFLICTS] = []

        return NO_ERROR

    @staticmethod
    def add_items(words,index,list,type):
        length = len(list)
        conflicts = []
        for w in words:
            temp = []
            for i in index:
                if w in i:
                    temp.append(i)
            if len(temp) <= 1:
                list[type] += temp
            else:
                conflicts += temp

        return conflicts

#main program to test the class definition and methods/
if __name__ == "__main__":

    print("Testing keyword search: ")
    # input1 = input("String: ")
    input1 = "Who has the snowboard"
    type(input1)

    test = SpeechInterpreter(input1,"en-EN")
    test.consolidate_input()

    msg = test.get_msg()

    print(msg)

#     struct = []
#
#     index = create_phrase_index()
#     id_generator= create_statements()
#
#     struct = []
#
#
#     tokens = nl.word_tokenize(new)
#     stop_words = set(stopwords.words('english'))
#     filtered = [w for w in tokens if not w in stop_words]
#     faces = set(line.strip() for line in open('facedb.txt'))
#     objects = set(line.strip() for line in open('cocodb.txt'))
#
# # these can be wrapped up into one function within a class, being passed through
# # either objects or faces depending on the use case. This will make it nicer as
# # we can then not always perform some loops depending on the instruction
#     for i in filtered:
#         for f in faces:
#             if i in f:
#                 struct.append(f)
#     if len(struct) == 1:
#         struct.append(-1)
    #print(struct)
    #print(tokens)
