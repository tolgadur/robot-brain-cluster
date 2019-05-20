#!/usr/bin/env python

"""
Face Training Client.

File name: face_training_client.py
Author: Nicolas Tobis
Date last modified: 14/05/2019
Python Version: 3.6
"""

import rospy
import sys
from face_training.srv import GetName

SENTENCE = 'language'

class FaceTrainingClient():

    def make_request(self, name):
        """
        Makes service request.

        Keyword arguments:
        name -- string
        """

        try:
            rospy.wait_for_service('face_training', timeout=5)
            face_trainer = rospy.ServiceProxy('face_training', GetName)
            response = face_trainer(name)
            print "Picture Taken: " + str(response.picture_taken)
            return response.picture_taken
        except (rospy.ServiceException, rospy.ROSException), e:
            print "Service call failed: %s"%e

if __name__ == '__main__':
    client = FaceTrainingClient()
    client.make_request('Steward')
    
