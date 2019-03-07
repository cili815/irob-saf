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
        print rospy.get_caller_id(), ' -> configuring dvrk_arm_test'
        self.arm_l = dvrk.mtm(robot_name_l)
        self.arm_r = dvrk.mtm(robot_name_r)
        self.coag_event = threading.Event()
        rospy.Subscriber('/dvrk/footpedals/coag',
                         Joy, self.coag_event_cb)
        self.set_gains_r_pub = rospy.Publisher(self.arm_r._arm__full_ros_namespace + '/set_cartesian_impedance_gains',
                                             prmCartesianImpedanceGains, latch = True, queue_size = 1)
        self.set_gains_l_pub = rospy.Publisher(self.arm_l._arm__full_ros_namespace + '/set_cartesian_impedance_gains',
                                                                                  prmCartesianImpedanceGains, latch = True, queue_size = 1)

    # homing example
    def home(self, arm):
        print rospy.get_caller_id(), ' -> starting home'
        arm.home()
        # get current joints just to set size
        goal = numpy.copy(arm.get_current_joint_position())
        # go to zero position
        goal.fill(0)
        arm.move_joint(goal, interpolate = True)

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
        gains_l = prmCartesianImpedanceGains()
        gains_r = prmCartesianImpedanceGains()
        # set orientation to identity quaternions
        gains_l.ForceOrientation.w = 1.0
        gains_l.TorqueOrientation.w = 1.0
        gains_r.ForceOrientation.w = 1.0
        gains_r.TorqueOrientation.w = 1.0
        self.arm_l.lock_orientation_as_is()
        self.arm_r.lock_orientation_as_is()
		
        print rospy.get_caller_id(), ' -> press COAG pedal to set straight position'
        self.wait_for_coag()
        #radius = 0.12

        currpos_l = self.arm_l.get_current_position()
        currpos_r = self.arm_r.get_current_position()
		
        center = numpy.array([(currpos_l.p[0] + currpos_r.p[0]) / 2.0, (currpos_l.p[1] + currpos_r.p[1]) / 2.0, (currpos_l.p[1] + currpos_r.p[2]) / 2.0])
       
        radius = numpy.abs(center_l.p[0] - currpos_l.p[0])
        
        couple_f = 20.0

        gains_l.PosStiffNeg.y = -400.0;
        gains_l.PosStiffPos.y = -400.0;
        gains_l.PosDampingNeg.y = -10.0;
        gains_l.PosDampingPos.y = -10.0;

        # Tangential direction
        gains_l.PosStiffNeg.z = -1.0;
        gains_l.PosStiffPos.z = -1.0;
        gains_l.PosDampingNeg.z = -3.0;
        gains_l.PosDampingPos.z = -3.0;

        gains_l.PosStiffNeg.x = -400.0;
        gains_l.PosStiffPos.x = -400.0;
        gains_l.PosDampingNeg.x = -10.0;
        gains_l.PosDampingPos.x = -10.0;
        
        gains_r.PosStiffNeg.y = -400.0;
        gains_r.PosStiffPos.y = -400.0;
        gains_r.PosDampingNeg.y = -2.0;
        gains_r.PosDampingPos.y = -2.0;

        # Tangential direction
        gains_r.PosStiffNeg.z = -1.0;
        gains_r.PosStiffPos.z = -1.0;
        gains_r.PosDampingNeg.z = -3.0;
        gains_r.PosDampingPos.z = -3.0;

        gains_r.PosStiffNeg.x = -400.0;
        gains_r.PosStiffPos.x = -400.0;
        gains_r.PosDampingNeg.x = -10.0;
        gains_r.PosDampingPos.x = -10.0;



        gains_l.ForcePosition.y = self.arm_l.get_current_position().p[1]
        gains_r.ForcePosition.y = self.arm_r.get_current_position().p[1]

        rate = rospy.Rate(100) # 10hz
        while not rospy.is_shutdown():
          #print rospy.get_caller_id(), ' -> Refresh gains'
          currpos_l = self.arm_l.get_current_position()
          currpos_r = self.arm_r.get_current_position()

          steer_angle_l, target_l = self.get_steer_angle(radius, center, currpos_l)
          steer_angle_r, target_r = self.get_steer_angle(radius, center, currpos_r) 
          #print 'steer_angle_l', steer_angle_l
          #print 'steer_angle_r', numpy.rad2deg(steer_angle_r)

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

          #angle_diff = (steer_angle_l-steer_angle_r)-numpy.pi
          steer_angle_l_rel = 0.0
          if steer_angle_l < 0.0:
            steer_angle_l_rel = numpy.pi + steer_angle_l
          else:
            steer_angle_l_rel = steer_angle_l - numpy.pi
          steer_angle_r_rel = steer_angle_r
          
          angle_diff = (steer_angle_l_rel-steer_angle_r_rel)
          #print 'angle_diff', numpy.rad2deg(angle_diff)
          #print 'steer_angle_l_rel', numpy.rad2deg(steer_angle_l_rel)
          #print 'steer_angle_r_rel', numpy.rad2deg(steer_angle_r_rel)
          
          if angle_diff > 0.0:
            gains_l.ForceBiasPos.z = couple_f * angle_diff
            gains_l.ForceBiasNeg.z = couple_f * angle_diff
            gains_r.ForceBiasPos.z = -couple_f * angle_diff
            gains_r.ForceBiasNeg.z = -couple_f * angle_diff
          else:
            gains_l.ForceBiasPos.z = couple_f * angle_diff
            gains_l.ForceBiasNeg.z = couple_f * angle_diff
            gains_r.ForceBiasPos.z = -couple_f * angle_diff
            gains_r.ForceBiasNeg.z = -couple_f * angle_diff


          self.set_gains_l_pub.publish(gains_l)
          self.set_gains_r_pub.publish(gains_r)


          rate.sleep()



    def get_steer_angle(self, radius, center, currpos):
        pos = numpy.array([currpos.p[0], currpos.p[1], currpos.p[2]])
        direction = pos - center
        direction = direction / numpy.linalg.norm(direction)

        target = center + (direction * radius)


        v_h = numpy.array([1.0, 0.0])
        v_s = numpy.array([direction[0], direction[2]])
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
        if (len(sys.argv) != 3):
            print sys.argv[0], ' requires two arguments, i.e. MTML or MTMR'
        else:
            application = example_application()
            application.configure(sys.argv[1], sys.argv[2])
            application.run()

    except rospy.ROSInterruptException:
        pass
