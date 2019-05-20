#!/usr/bin/env python
from ModelTrainer import ModelTrainer
import rospy
import argparse
import cv2


def main():
    modelTrainer = ModelTrainer("/usb_cam_left/image_raw")
    rospy.init_node('face_spoofing', anonymous=True)
    modelTrainer.train_with_camera("Pierre")
    try:
        rospy.spin()
    except KeyboardINterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
