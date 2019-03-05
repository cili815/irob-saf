#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2017-07-22

# (C) Copyright 2017 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file>

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > rosrun dvrk_python dvrk_mtm_cartesian_impedance <arm-name>

import dvrk
import math
import sys
import rospy
import numpy
import PyKDL
import threading
from sensor_msgs.msg import Joy
from cisst_msgs.msg import prmCartesianImpedanceGains

# example of application using arm.py
class example_application:

    # configuration
    def configure(self, robot_name):
        print rospy.get_caller_id(), ' -> configuring dvrk_arm_test for ', robot_name
        self.arm = dvrk.mtm(robot_name)
        self.coag_event = threading.Event()
        rospy.Subscriber('/dvrk/footpedals/coag',
                         Joy, self.coag_event_cb)
        self.set_gains_pub = rospy.Publisher(self.arm._arm__full_ros_namespace + '/set_cartesian_impedance_gains',
                                             prmCartesianImpedanceGains, latch = True, queue_size = 1)

    # homing example
    def home(self):
        print rospy.get_caller_id(), ' -> starting home'
        self.arm.home()
        # get current joints just to set size
        goal = numpy.copy(self.arm.get_current_joint_position())
        # go to zero position
        goal.fill(0)
        self.arm.move_joint(goal, interpolate = True)

    # foot pedal callback
    def coag_event_cb(self, data):
        if (data.buttons[0] == 1):
            self.coag_event.set()

    # wait for foot pedal
    def wait_for_coag(self):
        self.coag_event.clear()
        self.coag_event.wait(600)


    # tests
    def tests(self):
        gains = prmCartesianImpedanceGains()
        # set orientation to identity quaternions
        gains.ForceOrientation.w = 1.0
        gains.TorqueOrientation.w = 1.0
        self.arm.lock_orientation_as_is()

        print rospy.get_caller_id(), ' -> press COAG pedal to set straight position'
        self.wait_for_coag()
        radius = 0.12

        currpos = self.arm.get_current_position()

        center = numpy.array([currpos.p[0], currpos.p[1], currpos.p[2]])
        center = center - numpy.array([radius,0.0,  0.0])






        gains.PosStiffNeg.y = -400.0;
        gains.PosStiffPos.y = -400.0;
        gains.PosDampingNeg.y = -10.0;
        gains.PosDampingPos.y = -10.0;

        gains.PosStiffNeg.z = 0.0;
        gains.PosStiffPos.z = 0.0;
        gains.PosDampingNeg.z = 0.0;
        gains.PosDampingPos.z = 0.0;

        gains.PosStiffNeg.x = -400.0;
        gains.PosStiffPos.x = -400.0;
        gains.PosDampingNeg.x = -10.0;
        gains.PosDampingPos.x = -10.0;

        gains.ForcePosition.y = self.arm.get_current_position().p[1]

        rate = rospy.Rate(100) # 10hz
        while not rospy.is_shutdown():
          #print rospy.get_caller_id(), ' -> Refresh gains'
          currpos = self.arm.get_current_position()
          pos = numpy.array([currpos.p[0], currpos.p[1], currpos.p[2]])
          dir = pos - center
          dir = dir / numpy.linalg.norm(dir)

          target = center + (dir * radius)

          gains.ForcePosition.x = target[0]
          gains.ForcePosition.z = target[2]


          v_h = numpy.array([1.0, 0.0])
          v_s = numpy.array([dir[0], dir[2]])
          v_s = v_s / numpy.linalg.norm(v_s)

          steer_angle = numpy.arctan2(v_s[1], v_s[0])


          gains.ForceOrientation.x = 0.0
          gains.ForceOrientation.y = numpy.cos(steer_angle/2.0)
          gains.ForceOrientation.z = 0.0
          gains.ForceOrientation.w = numpy.sin(steer_angle/2.0)

          self.set_gains_pub.publish(gains)
          rate.sleep()


        gains.PosStiffNeg.y = 0.0;
        gains.PosStiffPos.y = 0.0;
        gains.PosDampingNeg.y = 0.0;
        gains.PosDampingPos.y = 0.0;

        gains.PosStiffNeg.z = 0.0;
        gains.PosStiffPos.z = 0.0;
        gains.PosDampingNeg.z = 0.0;
        gains.PosDampingPos.z = 0.0;

        gains.PosStiffNeg.x = 0.0;
        gains.PosStiffPos.x = 0.0;
        gains.PosDampingNeg.x = 0.0;
        gains.PosDampingPos.x = 0.0;
        self.set_gains_pub.publish(gains)


    # main method
    def run(self):
        self.home()
        self.tests()

if __name__ == '__main__':
    try:
        if (len(sys.argv) != 2):
            print sys.argv[0], ' requires one argument, i.e. MTML or MTMR'
        else:
            application = example_application()
            application.configure(sys.argv[1])
            application.run()

    except rospy.ROSInterruptException:
        pass
