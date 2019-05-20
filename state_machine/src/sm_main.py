#!/usr/bin/env python
"""
Main, running the state machine.

Author: Nicolas Tobis
Date: 02/2018
"""

import rospy
import smach_ros

import unittest
import threading
import time

# Custom modules import
import sm_smach

if __name__ == "__main__":
    rospy.init_node('hide_and_seek')

    # Create the state machine
    sm = sm_smach.init_sm()

    # Create and start introspection server - automatically traverses sm's child
    # containers, so only need to add this to the top-level state machine
    # sis = smach_ros.IntrospectionServer("Fezzik_Introspection_Server", sm, "/SM_TOP")
    # sm.start()

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()

    # sis.stop()
