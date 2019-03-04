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
    def configure(self, robot_name_l, robot_name_r):
        print rospy.get_caller_id(), ' -> configuring dvrk_arm_test for ', robot_name_l, ' and ' robot_name_r
        self.arm_l = dvrk.mtm(robot_name_l)
        self.arm_r = dvrk.mtm(robot_name_r)
        self.coag_event = threading.Event()
        rospy.Subscriber('/dvrk/footpedals/coag',
                         Joy, self.coag_event_cb)
        self.set_gains_r_pub = rospy.Publisher(self.arm._arm__full_ros_namespace + '/set_cartesian_impedance_gains',
                                             prmCartesianImpedanceGains, latch = True, queue_size = 1)
        self.set_gains_l_pub = rospy.Publisher(self.arm._arm__full_ros_namespace + '/set_cartesian_impedance_gains',
                                                                                  prmCartesianImpedanceGains, latch = True, queue_size = 1)

    # homing example
    def home(self, arm):
        print rospy.get_caller_id(), ' -> starting home'
        arm.home()
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


        couple_f = 1.0



        gains.PosStiffNeg.y = -400.0;
        gains.PosStiffPos.y = -400.0;
        gains.PosDampingNeg.y = -10.0;
        gains.PosDampingPos.y = -10.0;

        gains.PosStiffNeg.z = 0.0;
        gains.PosStiffPos.z = 0.0;
        gains.PosDampingNeg.z = 2.0;
        gains.PosDampingPos.z = 2.0;

        gains.PosStiffNeg.x = -400.0;
        gains.PosStiffPos.x = -400.0;
        gains.PosDampingNeg.x = -10.0;
        gains.PosDampingPos.x = -10.0;

        gains_l = gains
        gains_r = gains

        gains_l.ForcePosition.y = self.arm_l.get_current_position().p[1]
        gains_r.ForcePosition.y = self.arm_r.get_current_position().p[1]

        rate = rospy.Rate(100) # 10hz
        while not rospy.is_shutdown():
          #print rospy.get_caller_id(), ' -> Refresh gains'
          currpos_l = self.arm_l.get_current_position()
          currpos_r = self.arm_r.get_current_position()

          steer_angle_l, target_l = self.get_steer_angle(radius, center, currpos_l):
          steer_angle_r, target_r = self.get_steer_angle(radius, center, currpos_r):


          gains_l.ForcePosition.x = target_l[0]
          gains_l.ForcePosition.z = target_l[2]

          gains_r.ForcePosition.x = target_r[0]
          gains_r.ForcePosition.z = target_r[2]




          gains_l.ForceOrientation.x = 0.0
          gains_l.ForceOrientation.y = numpy.cos(steer_angle_l/2.0)
          gains_l.ForceOrientation.z = 0.0
          gains_l.ForceOrientation.w = numpy.sin(steer_angle_l/2.0)

          gains_r.ForceOrientation.x = 0.0
          gains_r.ForceOrientation.y = numpy.cos(steer_angle_r/2.0)
          gains_r.ForceOrientation.z = 0.0
          gains_r.ForceOrientation.w = numpy.sin(steer_angle_r/2.0)

          angle_diff = (steer_angle_l-steer_angle_r)-numpy.pi

          if angle_diff > 0.0:
            gains_l.ForceBiasPos.z = couple_f * angle_diff
            gains_l.ForceBiasNeg.z = couple_f * angle_diff
            gains_r.ForceBiasPos.z = -couple_f * angle_diff
            gains_r.ForceBiasNeg.z = -couple_f * angle_diff
          else:
            gains_l.ForceBiasPos.z = -couple_f * angle_diff
            gains_l.ForceBiasNeg.z = -couple_f * angle_diff
            gains_r.ForceBiasPos.z = couple_f * angle_diff
            gains_r.ForceBiasNeg.z = couple_f * angle_diff


          self.set_gains_l_pub.publish(gains_l)
          self.set_gains_r_pub.publish(gains_r)


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



    def get_steer_angle(self, radius, center, currpos):
        pos = numpy.array([currpos.p[0], currpos.p[1], currpos.p[2]])
        dir = pos - center
        dir = dir / numpy.linalg.norm(dir)

        target = center + (dir * radius)


        v_h = numpy.array([1.0, 0.0])
        v_s = numpy.array([dir[0], dir[2]])
        v_s = v_s / numpy.linalg.norm(v_s)

        steer_angle = numpy.arctan2(v_s[1], v_s[0])
        return steer_angle, target

    # main method
    def run(self):
        self.home(self.arm_l)
        self.home(self.arm_r)
        self.tests()

if __name__ == '__main__':
    try:
        if (len(sys.argv) != 2):
            print sys.argv[0], ' requires one argument, i.e. MTML or MTMR'
        else:
            application = example_application()
            application.configure(sys.argv[1], sys.argv[2])
            application.run()

    except rospy.ROSInterruptException:
        pass
