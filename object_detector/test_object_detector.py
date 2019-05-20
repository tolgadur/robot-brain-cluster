import sys
import unittest
import rospy
from yolo_publisher import object_detector, main
from cv_bridge import CvBridge, CvBridgeError
import cv2

detector = object_detector()
test_dict = {"label": "person", "topleft": {"x": 100, "y": 200}, "bottomright": {"x": 300, "y": 400}, "confidence": 0.8}
cap = cv2.imread("/home/sonto/catkin_ws/src/test_yolo_recog/darkflow/preview.png")


class TestObject_detector(unittest.TestCase):
    def test_constructor(self):
        detector2 = object_detector()

    def test_callback(self):
        bridge = CvBridge()
        bridge_image = bridge.cv2_to_imgmsg(cap, "bgr8")
        self.assertRaisesRegexp(rospy.ROSException, r"ROS node has not been initialized yet.*", detector.callback,
                                bridge_image)

    def test_initialise_obj_detection(self):
        detector.tfnet = detector.initialise_obj_detection()
        results = (detector.run_obj_detection(cap))
        self.assertEquals(results[0]["label"], "person")

    def test_run_obj_detection(self):
        try:
            detector.initialise_obj_detection()
        except:
            self.fail()

    def test_convert_to_obj_array(self):
        obj_message = detector.convert_to_obj_array([test_dict])
        self.assertEquals(obj_message.obj_list[0].label, "person")
        self.assertEquals(obj_message.obj_list[0].upper_x, 100)
        self.assertEquals(obj_message.obj_list[0].upper_y, 200)
        self.assertEquals(obj_message.obj_list[0].lower_x, 300)
        self.assertEquals(obj_message.obj_list[0].lower_y, 400)
        self.assertEquals(obj_message.obj_list[0].confidence, 0.8)

    def test_main(self):
        self.assertRaisesRegexp(rospy.ROSException, r"init_node interrupted before it could complete", main)



if __name__ == '__main__':
    unittest.main()
