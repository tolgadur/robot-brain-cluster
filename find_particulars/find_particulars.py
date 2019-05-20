#!/usr/bin/env python2.7
# coding: utf-8

"""
Converts information coming from find_object_2d nodes to our predefined
ObjectArray message type. When converting includes information about which
camera the particulars have been identified with. Publishes the converted
information on 'particular' topic in ROS.

Takes one argument from the command line (integer indicating which camera
the node belongs to).

File name: find_particulars.py
Author: Eivinas Butkus
Date created: 10/04/2019
Date last modified: 14/05/2019
Python Version: 2.7
"""

import sys
import rospy
import pandas as pd

from face_spoofing.msg import ObjectArray, Obj
from std_msgs.msg import Float32MultiArray

SCREEN_WIDTH = 1920
SCREEN_HEIGHT = 1080
PIXELS_FOR_PARTICULAR_BOUNDING_BOX = 200

class Obj_Constructor():

    def __init__(self, cam_id):
        self.cam_id = cam_id

        # reading the table that has a mapping from IDs to names of objects
        self.table = pd.read_csv("/home/robin/neuronerds/src/find_particulars_pkg/src/table_for_particulars.csv", index_col='id')

        # starting publisher where converted information will be republished
        self.publisher = rospy.Publisher('particular', ObjectArray, queue_size=100)

        print("\nObj_Constructor initialised with this particulars table:")
        print(self.table)
        print()

    def publish_obj_array(self, msg):
        '''
        Converts find_object_2d message to our own defined ObjectArray
        and publishes on the 'particular' topic.

        Arguments:
        msg -- find_object_2d message to be convrted
        '''

        print("detecting...")
        data = msg.data

        obj_array = ObjectArray()
        obj_array.id = self.cam_id

        for i in range(0, len(data), 12):
            obj = Obj()

            id = data[i]
            width = data[i + 1]
            height = data[i + 2]
            dx = data[i + 9]
            dy = data[i + 10]

            name = self.table.loc[int(id)][0]

            print(name)

            obj.label = name
            obj.upper_x = int(dx)
            obj.upper_y = int(dy)
            obj.lower_x = int(min(dx + PIXELS_FOR_PARTICULAR_BOUNDING_BOX, SCREEN_WIDTH - 1))
            obj.lower_y = int(min(dy + PIXELS_FOR_PARTICULAR_BOUNDING_BOX, SCREEN_HEIGHT - 1))
            obj.confidence = 1.0
            obj.boolean = True

            obj_array.obj_list.append(obj)

        self.publisher.publish(obj_array)

def main():
    cam_id = int(sys.argv[1])

    # determining node name to distinguish between them
    node_name = "find_particulars"
    if cam_id == 0:
        node_name += "_left"
    elif cam_id == 1:
        node_name += "_middle"
    elif cam_id == 2:
        node_name += "_right"

    rospy.init_node(node_name)
    obj_constructor = Obj_Constructor(cam_id)

    # creating a subscriber to the 'objects' topic that find_object_2d publishes to
    print("starting the subscriber to find_object_2d that will call the publisher...")
    subscriber = rospy.Subscriber(topic, Float32MultiArray, obj_constructor.publish_obj_array)
    print("done\n\n")

    rospy.spin()

if __name__ == "__main__":
    main()
