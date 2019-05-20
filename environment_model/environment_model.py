#!/usr/bin/env python2.7

"""
Environment Model responsible for aggregating information from the object
and face recognition subsystems. Constructs a model of the world around
DE NIRO that can be queried by any ROS node via the environment_service.

File name: environment_model.py
Authors: Eivinas Butkus, Joshua Harris
Date created: 01/02/2019
Date last modified: 14/05/2019
Python Version: 2.7

"""

import rospy
from copy import deepcopy
from yolo_pkg.msg import ObjectArray, Obj
from yolo_pkg.srv import DatabaseQuery
import sys

import pandas as pd
from threading import Lock

lock = Lock()

class Brain_Cluster_Database(object):

    def __init__(self):
        """
        Initialises environment model tables:

        table -- stores the objects/faces etc found in the latest frame from every camera
        aggregate_table -- stores the running total of the objects found over the previous 6 updates
        database -- the objects/faces that have been seem >2 times over the last 6 frames
        camera_id -- the id of the camera whose frame is currently being updated
        type -- the type of detection currently being updated (object/face etc)
        """
        self.table = pd.DataFrame(columns=['type', 'label', 'location', 'confidence', 'real','owner','camera'])
        self.aggregate_table = pd.DataFrame(columns=['type', 'label', 'location', 'confidence', 'real', 'owner','camera', 'count', 'found'])
        self.database = pd.DataFrame(columns=['type', 'label', 'location', 'confidence', 'real', 'owner','camera', 'count', 'found'])
        self.counterdisplay = 0
        self.camera_id = None
        self.type = None

    def display_table(self):
        """
        Prints the current database out to terminal
        """

        print('\n' * 100)
        print("****************************************** ENVIRONMENT MODEL ************************************************")
        print(self.database)

    def add(self, type, obj, camera):
        """
        Converts the custom ROS Object message into a row in the current table

        Keyword arguments:
        type -- the type of the object being added
        obj -- the ROS Object message
        camera -- the camera of the object being added
        """

        label = obj.label
        location = ((obj.upper_x, 1080-obj.upper_y), (obj.lower_x, 1080-obj.lower_y))
        confidence = obj.confidence
        spoof = obj.boolean

        self.table = self.table.append({'type':type, 'label':label, 'location':location, 'confidence':confidence, 'real':spoof, 'owner': "empty", "camera": camera}, ignore_index=True)


    def update(self, obj_array, type):
        """
        Callback function for ROS subscribers to /object, /face and /particular topics which adds the detected objects to the current frame table, then calls functions to allocate ownership and update the aggregate_table and database

        Keyword arguments:
        obj_array -- the ObjectArray message recieved from the topic
        type -- the type of the object e.g face, object, particular etc.

        """
        global lock
        lock.acquire()
        self.camera_id = obj_array.id

        self.type = type;

        # deleting objects of the type being updated
        # (i.e. of the same type and from the same camera)
        self.table = self.table[(self.table.type != type) | (self.table.camera != self.camera_id)]

        self.table = self.table.reset_index(drop=True)

        for obj in obj_array.obj_list:
            if obj_array.obj_list:
                self.add(type, obj, obj_array.id)


        self.allocate_ownership()

        self.update_aggregate_table()

        self.create_final_database()

        self.display_table()

        lock.release()


    def allocate_ownership(self):
        """
        Goes through every face found on the camera being updated, calls ownership space to find the body of the face (i.e person) and then allocates any objects that fall within this space to the face

        """
        for index in range(self.table.shape[0]):
            if self.table.loc[index]['type'] == "face" and self.table.loc[index]['camera'] == self.camera_id and self.table.loc[index]['label'] != "Unknown":
                    ownership_space = self._calc_ownership_space(self.table.loc[index]['location'],self.table.loc[index]['label'])
                    for index2 in range(self.table.shape[0]):
                        if self.table.loc[index2]['camera'] == self.camera_id:
                            if not self.table.loc[index2]['type'] == "face" and not self.table.loc[index2]['label'] == "person" and self.table.loc[index2]['owner'] == "empty":
                                if self._fully_within(self.table.loc[index2]['location'], ownership_space):
                                    self.table.at[index2,'owner'] = self.table.loc[index]['label']
                                elif self._object_overlap(self.table.loc[index2]['location'],ownership_space) > 0.5:
                                    self.table.at[index2,'owner'] = self.table.loc[index]['label']



    def update_table_values(self, table_index, agg_index):
        """
        Called for objects that already exist in the aggregate_table to update their current owners, location etc.

        Keyword argumets:

        table_index -- the index of the object in the current frame tables
        agg_index -- the index of the object in the aggregate table

        """
        self.aggregate_table.at[agg_index,'confidence'] = self.table.at[table_index,'confidence']
        self.aggregate_table.at[agg_index,'real'] = self.table.at[table_index, 'real']
        self.aggregate_table.at[agg_index,'location'] = self.table.at[table_index, 'location']
        self.aggregate_table.at[agg_index,'owner'] = self.table.at[table_index, 'owner']
        if self.aggregate_table.at[agg_index,'count'] < 6 and self.type == self.table.loc[table_index, 'type']:
            self.aggregate_table.at[agg_index,'count'] += 1
            self.aggregate_table.at[agg_index,'found'] = True



    def update_aggregate_table(self):
        """
        Updating the aggregate table:
        1. Sets all the items in the aggregate table to not being in the current frame
        2. Goes through every row in the current frame table from the camera being updated
        3. Goes through every row in the current aggregate table checking if any match i.e the object was already seen in a previous frame
        4. For any rows in the current frame table not found in the aggregate table add them
        5. Reset calculate the counters for the every row in the aggregate table of the current camera and delete any that have reached zero

        """

        for index2 in range(self.aggregate_table.shape[0]):
            self.aggregate_table.at[index2,'found'] = False

        for index in range(self.table.shape[0]):
            if self.camera_id == self.table.loc[index, 'camera']:
                included = False

                for index2 in range(self.aggregate_table.shape[0]):

                    if self.table.loc[index][['type', 'label','camera']].equals(self.aggregate_table.loc[index2][['type', 'label','camera']]):

                        if self.table.loc[index]['type'] == "object" and self._object_overlap(self.aggregate_table.loc[index2]['location'],self.table.loc[index]['location'])>0.5 and not self.aggregate_table.loc[index2]['found'] :
                            self.update_table_values(index,index2)
                            included = True
                            break

                        elif self.table.loc[index]['type'] == "face" and self.table.loc[index]['real'] == self.aggregate_table.loc[index2]['real']:
                            self.update_table_values(index,index2)
                            included = True
                            break

                        elif self.table.loc[index]['type'] == "particular":
                            self.update_table_values(index,index2)
                            included = True
                            break

                if not included:
                    self.aggregate_table = self.aggregate_table.append({'type':self.table.loc[index]['type'], 'label':self.table.loc[index]['label'], 'location':self.table.loc[index]['location'], 'confidence':self.table.loc[index]['confidence'], 'real':self.table.loc[index]['real'], 'owner': self.table.loc[index]['owner'],'camera': self.table.loc[index]['camera'],'count': 1, 'found': True}, ignore_index=True)


        for index2 in range(self.aggregate_table.shape[0]):
            if self.camera_id == self.aggregate_table.loc[index2, 'camera'] and self.type == self.aggregate_table.loc[index2, 'type']:

                if self.aggregate_table.at[index2,'found'] == False:
                    self.aggregate_table.at[index2,'count'] -= 1

                if self.aggregate_table.at[index2,'count'] <= 0:
                    self.aggregate_table = self.aggregate_table.drop([index2])

        self.aggregate_table = self.aggregate_table.reset_index(drop=True)


    def create_final_database(self):
    """
    Creates a new database table of only those objects that have been seen a threshold number of times in the last 6 frames

    """
        self.database.drop(self.database.index, inplace = True)
        for index in range(self.aggregate_table.shape[0]):
            if self.aggregate_table.at[index,'count'] >= 2:
                self.database = self.database.append(self.aggregate_table.loc[index],ignore_index = True)


    def _calc_ownership_space(self, face_location, face_label):
        """
        Calculates ownership space for a specific face:
        1. First sets ownership space to the area of the face identified
        2. Iterates over all the person objects identified in the current camera frame
        3. Checks if the face location is fully within the person and if it is sets the returns the ownership to the area of the person
        4. If not then finds any person objects that overlap >20% with the face and returns the ownership space of the best overlap
        5. If none overlap then the ownership remains the area of the face

        Keyword arguments:
        face_location -- the tuples containing the bounding box of the face
        face_label -- the label of the face identified

        Returns:
        ownership_space -- the bounding box of the combined face any person identified as the body of that face
        """

        ownership_space = deepcopy(face_location)
        best_overlap = 0
        best_index = 0

        for index in range(self.table.shape[0]):
            if self.table.loc[index]['camera'] == self.camera_id and self.table.loc[index]['label'] == "person":

                    if self._fully_within(face_location, self.table.loc[index]['location']):
                        ownership_space = deepcopy(self.table.loc[index]['location'])
                        self.table.at[index,'owner'] = face_label
                        return ownership_space

                    overlap = self._object_overlap(face_location, self.table.loc[index]['location'])
                    if overlap >= 0.2 and overlap > best_overlap:
                        best_overlap = overlap
                        best_index = index
                        ownership_space = self._combine_boxes(ownership_space, self.table.loc[index]['location'])

        if best_overlap > 0.0:
            self.table.at[best_index,'owner'] = deepcopy(face_label)
        return ownership_space



    def _fully_within(self,loc_subset, loc_set):
        """
        Checks if the subset is fully within the set

        Keyword arguments:
        loc_subset -- the tuples containing the boundinng box of the subset
        loc_set -- the tuples containing the bounding box of the set

        Returns -- boolean True if subset fully within the set, False otherwise
        """
        if loc_subset[0][0] >= loc_set[0][0] and loc_subset[1][0] <= loc_set[1][0]:
            if loc_subset[0][1] <= loc_set[0][1] and loc_subset[1][1] >= loc_set[1][1]:
                return True
        return False


    def _combine_boxes(self, loc_1, loc_2):
        """
        Combines to bounding boxes into the maximum bounding box using possible with those sets of coordinates

        Keyword arguments:
        loc_1 -- set of tuples defining the bounding box for a location
        loc_2 -- set of tuples defining the bounding box for a location

        Returns -- set of tuples defining the max bounding box of loc_1 and loc_2
        """

        combined_location = ((min(loc_1[0][0], loc_2[0][0]), max(loc_1[0][1], loc_2[0][1])),
                             (max(loc_1[1][0], loc_2[1][0]), min(loc_1[1][1], loc_2[1][1])))
        return combined_location

    def _object_overlap(self,loc_subset, loc_set):
        """
        Calculates the percent overlap of the subset with the set

        Keyword arguments:
        loc_subset -- the tuples defining the bounding box that overlaps the set
        loc_set -- the tuples defining the bounding box being overlapped

        Returns -- float percentage overlap of the subset over the set
        """

        if loc_subset[0][0] >= loc_set[1][0] or loc_subset[1][0] <= loc_set[0][0] or loc_subset[0][1] <= loc_set[1][1] or loc_subset[1][1] >= loc_set[0][1]:
                return 0
        overlap = (min(loc_subset[1][0], loc_set[1][0]) - max(loc_subset[0][0], loc_set[0][0])) * (min(loc_subset[0][1], loc_set[0][1]) - max(loc_subset[1][1], loc_set[1][1]))
        return float(overlap) / ((loc_subset[1][0] - loc_subset[0][0])*(loc_subset[0][1]-loc_subset[1][1]))



    def database_query(self, query):
        """
        Callback for the service requiring information from the database, converts the database table to arrays to return to the client of the Service

        Keyword arguments:
        query -- the integer code representing the query

        """
        global lock
        lock.acquire()
        table = deepcopy(self.database)
        lock.release()

        types = table.type.tolist()
        labels = table.label.tolist()
        owners = table.owner.tolist()
        real = table.real.tolist()
        confidences = table.confidence.tolist()
        location = table.location.tolist()
        camera = table.camera.tolist()

        upper_x = []
        upper_y = []
        lower_x = []
        lower_y = []

        for loc in location:
            upper_x.append(loc[0][0])
            upper_y.append(loc[0][1])
            lower_x.append(loc[1][0])
            lower_y.append(loc[1][1])

        return query.instruction_id, types, labels, owners, real, confidences, upper_x, upper_y, lower_x, lower_y, camera


def main():
    """
    Creates and instance of the Brain_Cluster_Database
    Creates Subscribers to the object/face/particular topics
    Creates the Service for querying the database

    """

    pd.options.display.width = 999
    rospy.init_node('brain_cluster_database')
    database = Brain_Cluster_Database()

    sub_face = rospy.Subscriber('face', ObjectArray, database.update, ('face'))
    sub_object = rospy.Subscriber('object', ObjectArray, database.update, ('object'))
    sub_particular = rospy.Subscriber('particular', ObjectArray, database.update, ('particular'))

    service = rospy.Service('environment_service', DatabaseQuery, database.database_query)

    rospy.spin()

if __name__ == '__main__':
    main()
