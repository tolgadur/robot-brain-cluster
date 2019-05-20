#!/usr/bin/env python
"""
Pointing Service.

File name: pointing_service.py
Author: Nicolas Tobis
Date last modified: 14/05/2019
Python Version: 3.6
"""

import rospy
from pointing.srv import PointingRequest
from geometry_msgs.msg import Point
from ik_service_client import get_joint_angles
import baxter_interface
import tuck_arms
from baxter_interface import CHECK_VERSION
import sys

# Rospy parameter indicating if pointing complete
POINTING_COMPLETE = 'pointing_complete'


class PointingServer:

    def try_float(self, x):
        try:
            return float(x)
        except ValueError:
            return None


    def clean_line(self, line, names):
        """
        Cleans a single line of recorded joint positions

        @param line: the line described in a list to process
        @param names: joint name keys
        """
        #convert the line of strings to a float or None
        line = [self.try_float(x) for x in line.rstrip().split(',')]
        #zip the values with the joint names
        combined = zip(names[1:], line[1:])
        #take out any tuples that have a none value
        cleaned = [x for x in combined if x[1] is not None]
        #convert it to a dictionary with only valid commands
        command = dict(cleaned)
        left_command = dict((key, command[key]) for key in command.keys()
                            if key[:-2] == 'left_')
        right_command = dict((key, command[key]) for key in command.keys()
                             if key[:-2] == 'right_')
        return (command, left_command, right_command, line)


    def map_file(self, filename, loops=1):
        """
        Loops through csv file

        @param filename: the file to play
        @param loops: number of times to loop
                      values < 0 mean 'infinite'

        Does not loop indefinitely, but only until the file is read
        and processed. Reads each line, split up in columns and
        formats each line into a controller command in the form of
        name/value pairs. Names come from the column headers
        first column is the time stamp
        """
        left = baxter_interface.Limb('left')
        right = baxter_interface.Limb('right')
        grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
        grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
        rate = rospy.Rate(1000)

        if grip_left.error():
            grip_left.reset()
        if grip_right.error():
            grip_right.reset()
        if (not grip_left.calibrated() and
            grip_left.type() != 'custom'):
            grip_left.calibrate()
        if (not grip_right.calibrated() and
            grip_right.type() != 'custom'):
            grip_right.calibrate()


        print("Playing back: %s" % (filename,))
        with open(filename, 'r') as f:
            lines = f.readlines()
        keys = lines[0].rstrip().split(',')

        l = 0
        # If specified, repeat the file playback 'loops' number of times
        while loops < 1 or l < loops:
            i = 0
            l += 1
            print("Moving to start position...")

            _cmd, lcmd_start, rcmd_start, _raw = self.clean_line(lines[1], keys)
            left.move_to_joint_positions(lcmd_start)
            right.move_to_joint_positions(rcmd_start)
            start_time = rospy.get_time()
            for values in lines[1:]:
                i += 1
                loopstr = str(loops) if loops > 0 else "forever"
                sys.stdout.write("\r Record %d of %d, loop %d of %s" %
                                 (i, len(lines) - 1, l, loopstr))
                sys.stdout.flush()

                cmd, lcmd, rcmd, values = self.clean_line(values, keys)
                #command this set of commands until the next frame
                while (rospy.get_time() - start_time) < values[0]:
                    if rospy.is_shutdown():
                        print("\n Aborting - ROS shutdown")
                        return False
                    if len(lcmd):
                        left.set_joint_positions(lcmd)
                    if len(rcmd):
                        right.set_joint_positions(rcmd)
                    if ('left_gripper' in cmd and
                        grip_left.type() != 'custom'):
                        grip_left.command_position(cmd['left_gripper'])
                    if ('right_gripper' in cmd and
                        grip_right.type() != 'custom'):
                        grip_right.command_position(cmd['right_gripper'])
                    rate.sleep()
            print
        return True


    def move_arm(self, request):
        """
        Perform service call of moving an arm

        Keyword arguments:
        request -- ROS message
        """
        
        print("Getting robot state... ")
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled

        def clean_shutdown():
            print("\nExiting example...")
            if not init_state:
                print("Disabling robot...")
                rs.disable()
        rospy.on_shutdown(clean_shutdown)

        print("Enabling robot... ")
        rs.enable()
        self.map_file(request.filename)
        rospy.set_param(POINTING_COMPLETE, True)


    def enable_arms(self):
        """
        Enables the robot which enables the arms

        :return: bool
        """

        rospy.loginfo("Attempting to enabling robot.")
        rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)

        try:
            rs.enable()
        except Exception, e:
            rospy.logerr(e.strerror)
            rospy.logerr("Failed to enable arms.")
            return False

        rospy.loginfo("Successfully enabled robot.")
        return True

if __name__ == "__main__":
    rospy.init_node('pointing_server')
    pointing_service = PointingServer()
    pointing_service.enable_arms()
    s = rospy.Service('pointing', PointingRequest, pointing_service.move_arm)
    rospy.spin()
