#!/usr/bin/env python

"""
ROS node running YOLO object recognition.

File name: object_detector.py
Authors: Eivinas Butkus, Joshua Harris
Date created: 01/02/2019
Date last modified: 14/05/2019
Python Version: 2.7

Object Recognition Model: YOLOv2 darkflow python implementation: https://github.com/thtrieu/darkflow

"""

from __future__ import print_function
import sys
from darkflow.net.build import TFNet
import cv2
import numpy as np
import rospy
from yolo_pkg.msg import ObjectArray, Obj
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Object_detector:
"""
Object_detector has as member variables the ROS pubisher and subscriber to interact with other ROS nodes and member functions to perform object recognition
"""

  def __init__(self, camera, id):
     """
     Sets up ROS publisher, ROS subscriber and YOLO model

     Keyword arguements:
     camera -- string name of the camera image topic to subscribe to
     id -- integer id of the camera being subscribed to
     """
    self.image_pub = rospy.Publisher('object', ObjectArray)
    self.camera = camera
    self.id = int(id)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(self.camera,Image,self.callback)
    self.tfnet = self.initialise_obj_detection()

  def callback(self,data):
      """
      Callback function for ROS Subscriber to the camera topic, converts image to OpenCV, calls object detection and publishes the results

      Keyword arguements:
      data -- ROS Image message
      """
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    results = self.run_obj_detection(cv_image)
    object_message = self.convert_to_obj_array(results)
    self.image_pub.publish(object_message)

  def initialise_obj_detection(self):
      """
       Initialises YOLO model with pretrained weights and 40% confidence threshold

       Returns -- configured YOLO model
       """
    options = dict({"model": "cfg/yolo.cfg", "load": "yolo.weights", "threshold": 0.4})
    return TFNet(options)

  def run_obj_detection(self,cap):
    """
    Passes image through YOLO model

    Keyword arguements:
    cap -- the OpenCV image

    Returns -- dictionary of results from YOLO object recogniton

    """
    results = self.tfnet.return_predict(cap)
    return results

  def convert_to_obj_array(self,results):
    """
    Converts dictionary of results from YOLO model into custom ObjectArray ROS message of the objects found in the image

    Keyword arguements:
    results -- the dictionary of results

    Returns -- ObjectArray

    """
    object_message = ObjectArray()
    object_message.id = self.id
    for element in results:
        object = Obj()
        object.label = element["label"]
        object.upper_x = element["topleft"]["x"]
        object.upper_y = element["topleft"]["y"]
        object.lower_x = element["bottomright"]["x"]
        object.lower_y = element["bottomright"]["y"]
        object.confidence = element["confidence"]
        object.boolean = True
        object_message.obj_list.append(object)
    return object_message

def main(args):
 """
 Creates instance of Object_detector class and the ROS node
 in which it runs
 """
  od = Object_detector(sys.argv[1],sys.argv[2])
  rospy.init_node('objects', anonymous=True)
  try:

    rospy.spin()

  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
