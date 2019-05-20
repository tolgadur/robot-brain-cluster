#!/usr/bin/env python2.7

import unittest

from database_pkg.msg import ObjectArray, Obj
from std_msgs.msg import Float32MultiArray

from find_particulars import Obj_Constructor

converter_cam_0 = Obj_Constructor(0)
converter_cam_2 = Obj_Constructor(2)

objects = Float32MultiArray()
objects.data = [2, 250, 300, 0, 0, 0, 0, 0, 0, 510, 420, 0,
           3, 800, 20, 0, 0, 0, 0, 0, 0, 223, 900, 0]

bad_objects = Float32MultiArray()
bad_objects.data = [5, 250, 300, 0, 0, 0, 0, 0, 0, 510, 420, 0]

class Test_Find_Particulars(unittest.TestCase):

    def test_publish_obj_array(self):
        obj_array_0 = converter_cam_0.publish_obj_array(objects)
        obj_array_2 = converter_cam_2.publish_obj_array(objects)

        self.assertEqual(obj_array_0.id, 0)
        self.assertEqual(obj_array_2.id, 2)

        self.assertEqual(obj_array_0.obj_list[0].label, "Logitech Camera Box")
        self.assertEqual(obj_array_2.obj_list[1].label, "Farrow's Giant Marrowfat Peas")

        self.assertEqual(obj_array_0.obj_list[0].upper_x, 510)
        self.assertEqual(obj_array_0.obj_list[0].lower_x, 710)

        error_caught = False
        try:
            obj_array_bad = converter_cam_0.publish_obj_array(bad_objects)
        except KeyError:
            error_caught = True
        self.assertEqual(error_caught, True)


def suite():
    suite = unittest.TestSuite()
    suite.addTest(Test_Find_Particulars('test_publish_obj_array'))

    return suite


if __name__ == '__main__':
    runner = unittest.TextTestRunner(failfast=True, verbosity=2)
    runner.run(suite())
