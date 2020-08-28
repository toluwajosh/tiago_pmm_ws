#!/usr/bin/env python

# Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Author:
#   * Sam Pfeiffer
#   * Job van Dieten
#   * Jordi Pages

import rospy
import time
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient

from copy import deepcopy
from math_util import *

# added
from moveit_commander import MoveGroupCommander#, PlanningSceneInterface

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

import numpy as np
from std_srvs.srv import Empty

import cv2
from cv_bridge import CvBridge

from moveit_msgs.msg import MoveItErrorCodes
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

class SphericalService(object):
    def __init__(self):
        rospy.loginfo("Starting Spherical Grab Service")
        self.pick_type = PickAruco()
        rospy.loginfo("Finished SphericalService constructor")
        self.place_gui = rospy.Service("/place_gui", Empty, self.start_aruco_place)
        self.pick_gui = rospy.Service("/pick_gui", Empty, self.start_aruco_pick)

    def start_aruco_pick(self, req):
        self.pick_type.pick_aruco("pick")
        return {}

    def start_aruco_place(self, req):
        self.pick_type.pick_aruco("place")
        return {}

class PickAruco(object):
    def __init__(self):
        rospy.loginfo("Initalizing...")
        self.bridge = CvBridge()
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_l = tf2_ros.TransformListener(self.tfBuffer)
                
        rospy.loginfo("Waiting for /pickup_pose AS...")
        self.pick_as = SimpleActionClient('/pickup_pose', PickUpPoseAction)
        time.sleep(1.0)
        if not self.pick_as.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Could not connect to /pickup_pose AS")
            exit()
        rospy.loginfo("Waiting for /place_pose AS...")
        self.place_as = SimpleActionClient('/place_pose', PickUpPoseAction)

        self.place_as.wait_for_server()

        rospy.loginfo("Setting publishers to torso and head controller...")
        self.torso_cmd = rospy.Publisher(
            '/torso_controller/command', JointTrajectory, queue_size=1)
        self.head_cmd = rospy.Publisher(
            '/head_controller/command', JointTrajectory, queue_size=1)
        self.detected_pose_pub = rospy.Publisher('/detected_aruco_pose',
                             PoseStamped,
                             queue_size=1,
                             latch=True)

        # /arm_controller/command (trajectory_msgs/JointTrajectory) 
        self.trajectory_cmd = rospy.Publisher(
            '/arm_controller/command', JointTrajectory, queue_size=1)

        self.gripper_cmd = rospy.Publisher(
            '/gripper_controller/command', JointTrajectory, queue_size=1)

        rospy.loginfo("Waiting for '/play_motion' AS...")
        self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
        if not self.play_m_as.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Could not connect to /play_motion AS")
            exit()
        rospy.loginfo("Connected!")
        rospy.sleep(1.0)
        rospy.loginfo("Done initializing PickAruco.")

    def strip_leading_slash(self, s):
        return s[1:] if s.startswith("/") else s
        
    def pick_aruco(self, string_operation):
        self.prepare_robot()

        rospy.sleep(2.0)
        rospy.loginfo("spherical_grasp_gui: Waiting for an aruco detection")

        # self.turn_hand() # experimental function

        # get aruco pose here: from marker
        aruco_pose = rospy.wait_for_message('/aruco_single/pose', PoseStamped)
        aruco_pose.header.frame_id = self.strip_leading_slash(aruco_pose.header.frame_id)
        rospy.loginfo("Got: " + str(aruco_pose))
        rospy.loginfo("spherical_grasp_gui: Transforming from frame: " +
        aruco_pose.header.frame_id + " to 'base_footprint'")


        ps = PoseStamped()
        ps.pose.position = aruco_pose.pose.position
        ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", aruco_pose.header.frame_id)
        ps.header.frame_id = aruco_pose.header.frame_id
        transform_ok = False
        while not transform_ok and not rospy.is_shutdown():
            try:
                transform = self.tfBuffer.lookup_transform("base_footprint", 
                                       ps.header.frame_id,
                                       rospy.Time(0))
                aruco_ps = do_transform_pose(ps, transform)
                transform_ok = True
            except tf2_ros.ExtrapolationException as e:
                rospy.logwarn(
                    "Exception on transforming point... trying again \n(" +
                    str(e) + ")")
                rospy.sleep(0.01)
                ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", aruco_pose.header.frame_id)
            pick_g = PickUpPoseGoal()

        if string_operation == "pick":

            rospy.loginfo("Setting cube pose based on ArUco detection")
            pick_g.object_pose.pose.position = aruco_ps.pose.position
            pick_g.object_pose.pose.position.z -= 0.1*(1.0/2.0)

            rospy.loginfo("aruco pose in base_footprint:" + str(pick_g))

            pick_g.object_pose.header.frame_id = 'base_footprint'
            pick_g.object_pose.pose.orientation.w = 1.0
            self.detected_pose_pub.publish(pick_g.object_pose)
            rospy.loginfo("Gonna pick:" + str(pick_g))
            self.pick_as.send_goal_and_wait(pick_g)
            rospy.loginfo("Done!")

            # some_pose=arm.get_current_pose(end_effector_link).pose

            # rospy.loginfo("current hand pose "+str(some_pose))

            result = self.pick_as.get_result()

            if str(moveit_error_dict[result.error_code]) != "SUCCESS":
                rospy.logerr("Failed to pick, not trying further")
                return

            
            # Move torso to its maximum height
            self.lift_torso()
            aruco_pose.pose.position.x += 0.2
            aruco_pose.pose.position.y += 0.2
            aruco_pose.pose.position.z += 0.2


            drop_pose = deepcopy(aruco_pose)
            # drop_pose.pose.position.x += 0.2
            # drop_pose.pose.position.y += 0.3
            # drop_pose.pose.position.z += 0.2
            # optimize_goal_pose(drop_pose)
            """
            The following are good good_candidates for pose selection (in eular angles): 
            [[0, 0, 0], [0, 0, 270], [0, 180, 0], [0, 180, 180] - no, [0, 180, 270], [0, 270, 0], [0, 270, 180], [0, 270, 270]]
            """
            x, y, z, w = eular_to_q(0,270,180)
            # x, y, z, w = eular_to_q(0,270,270)
            drop_pose.pose.orientation.x = -0.5
            drop_pose.pose.orientation.y = 0
            drop_pose.pose.orientation.z = 0
            drop_pose.pose.orientation.w = 1.2
            cartesian_move_to(drop_pose, True)

            x, y, z = q_to_eular(-0.5, 0, 0, 1.2)
            rospy.loginfo("End effector eular angles: " + str([x,y,z]))
            rospy.sleep(3)
            self.open_gripper()

            # arm=MoveGroupCommander('arm')
            # arm.allow_replanning(True)
            # end_effector_link=arm.get_end_effector_link()
            # arm.set_goal_position_tolerance(0.03)
            # arm.set_goal_orientation_tolerance(0.025)
            # arm.allow_replanning(True)

            # reference_frame='base_footprint'
            # arm.set_pose_reference_frame(reference_frame)
            # arm.set_planning_time(5)

            # rospy.sleep(2)
            # start_pose=arm.get_current_pose(end_effector_link).pose

            # rospy.loginfo("End effector start pose: " + str(start_pose))
            # x = start_pose.orientation.x
            # y = start_pose.orientation.y
            # z = start_pose.orientation.z
            # w = start_pose.orientation.w

            # x, y, z = q_to_eular(x, y, z, w)
            # rospy.loginfo("End effector eular angles: " + str([x,y,z]))


            # # Raise arm
            # rospy.loginfo("Moving arm to a safe pose")
            # pmg = PlayMotionGoal()
            # pmg.motion_name = 'pick_final_pose'
            # pmg.skip_planning = False
            # self.play_m_as.send_goal_and_wait(pmg)
            # rospy.loginfo("Raise object done.")

            # # Place the object back to its position
            # rospy.loginfo("Gonna place near where it was")
            # pick_g.object_pose.pose.position.z += 0.05
            # pick_g.object_pose.pose.position.y += 0.2
            # self.place_as.send_goal_and_wait(pick_g)
            # rospy.loginfo("Done!")

    def lift_torso(self):
        rospy.loginfo("Moving torso up")
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.34]
        jtp.time_from_start = rospy.Duration(2.5)
        jt.points.append(jtp)
        self.torso_cmd.publish(jt)

    def turn_hand(self):
        rospy.loginfo("Turning Arm")
        jt = JointTrajectory()
        jt.joint_names = ['arm_1_joint',
        'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 
        'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        jtp.time_from_start = rospy.Duration(3)
        jt.points.append(jtp)

        # jtp = JointTrajectoryPoint()
        # jtp.positions = [1.5,0.0,0.0,0.0,0.0,0.0,0.0]
        # jtp.time_from_start = rospy.Duration(6)
        # jt.points.append(jtp)

        # jtp = JointTrajectoryPoint()
        # jtp.positions = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        # jtp.time_from_start = rospy.Duration(9)
        # jt.points.append(jtp)
        self.trajectory_cmd.publish(jt)
        rospy.loginfo("Done 1")

        rospy.sleep(0.1)

        jt = JointTrajectory()
        jt.joint_names = ['arm_1_joint',
        'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 
        'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [3,0.0,0.0,0.0,0.0,0.0,0.0]
        jtp.time_from_start = rospy.Duration(6)
        jt.points.append(jtp)
        self.trajectory_cmd.publish(jt)
        rospy.loginfo("Done 2")

        rospy.sleep(0.1)

        jt = JointTrajectory()
        jt.joint_names = ['arm_1_joint',
        'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 
        'arm_5_joint', 'arm_6_joint', 'arm_7_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        jtp.time_from_start = rospy.Duration(9)
        jt.points.append(jtp)
        self.trajectory_cmd.publish(jt)
        rospy.loginfo("Done 3")



        # rospy.sleep(2.0)



        rospy.loginfo("Done.")

    def lower_head(self):
        rospy.loginfo("Moving head down")
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0, -0.75]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)
        rospy.loginfo("Done.")

    def prepare_robot(self):
        rospy.loginfo("Unfold arm safely")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'pregrasp'
        pmg.skip_planning = False
        self.play_m_as.send_goal_and_wait(pmg)
        rospy.loginfo("Done.")

        self.lower_head()

        rospy.loginfo("Robot prepared.")

    def open_gripper(self):
        rospy.loginfo("Opening Gripper")
        jt = JointTrajectory()
        jt.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.044, 0.044]
        jtp.time_from_start = rospy.Duration(0.5)
        jt.points.append(jtp)
        self.gripper_cmd.publish(jt)
        rospy.loginfo("Done.")


def cartesian_move_to(pose_, execute=False):
    goal_pose = deepcopy(pose_)
    ######################################## experiment start
    # for hardcoded aruco pose
    # some known arm positions
    # aruco pose in simulation: 0.52680940312, -0.0490591337742, 0.921301848054, 0.504516550338, 0.506271228009, 0.497290765692, 0.49178693403
    # in base_footprint: 0.52680940312, -0.0490591337742, 0.871301848054, 0, 0, 0, 0
    # pick pose: 0.52680940312, -0.0490591337742, 0.871301848054, 0, 0, 0, 0
    # final hand pose in simulation: 0.47653800831, -0.396454309047, 1.05656705145, -0.630265833017, -0.318648344327, 0.582106845777, 0.402963810396
    
    # goal_pose = PoseStamped()
    # goal_pose.header.frame_id = 'base_footprint'
    
    # goal_pose.pose.position.x = 0.52680940312
    # goal_pose.pose.position.y = -0.0490591337742
    # goal_pose.pose.position.z = 0.871301848054
    # goal_pose.pose.orientation.x = 0
    # goal_pose.pose.orientation.y = 0
    # goal_pose.pose.orientation.z = 0
    # goal_pose.pose.orientation.w = 1


    arm=MoveGroupCommander('arm')
    arm.allow_replanning(True)
    end_effector_link=arm.get_end_effector_link()
    arm.set_goal_position_tolerance(0.03)
    arm.set_goal_orientation_tolerance(0.025)
    arm.allow_replanning(True)

    reference_frame='base_footprint'
    arm.set_pose_reference_frame(reference_frame)
    arm.set_planning_time(5)

    rospy.sleep(2)
    start_pose=arm.get_current_pose(end_effector_link).pose

    rospy.loginfo("End effector start pose: " + str(start_pose))
    rospy.loginfo("Aruco pose: " + str(goal_pose))

    waypoints=[]
    waypoints.append(start_pose)

    # goal_pose.pose.orientation.x = -0.5
    # goal_pose.pose.orientation.y = -0.5
    # goal_pose.pose.orientation.z = -0.5
    # goal_pose.pose.orientation.w = 1

    # goal_pose.pose.orientation.y -= 0.1*(1.0/2.0)

    waypoints.append(deepcopy(goal_pose.pose))
    fraction=0.0
    maxtries=100
    attempts=0
    while fraction<1.0 and attempts<maxtries:
        (plan,fraction)=arm.compute_cartesian_path(waypoints,0.01,0.0,True)
        attempts+=1
        if attempts %20==0:
            if (attempts<100):
                rospy.loginfo("still trying after"+str(attempts)+" attempts...")
            else : 
                rospy.loginfo("Finished after  "+str(attempts)+" attempts...")

        if fraction>0.89:
            rospy.loginfo("path compute successfully. Arm is moving.")
            print(plan.joint_trajectory.points[len(plan.joint_trajectory.points)-1].positions)
            print(plan)
            #for item in plan.joint_trajectory.points[len(plan.joint_trajectory.points)-1]
            if execute:
                arm.execute(plan)
                rospy.loginfo("path execution complete. ")
                rospy.sleep(2)              
        if  (attempts %100==0 and fraction<0.9):
            rospy.loginfo("path planning failed with only  " + str(fraction*100)+ "% success after  "+ str(maxtries)+" attempts")
    return fraction

def optimize_goal_pose(pose_):
    e_x = [0]
    e_y = [0, 90, 180, 270, 360]
    e_z = [0, 90, 180, 270, 360]
    chosen = None
    good_candidates = []
    score = 0
    for x_ in e_x:
        for y_ in e_y:
            for z_  in e_z:
                w, x, y, z = eular_to_q(x_, y_, z_)
                pose_.pose.orientation.x = x
                pose_.pose.orientation.y = y
                pose_.pose.orientation.z = z
                pose_.pose.orientation.w = w

                success = cartesian_move_to(pose_)
                if success > 0.89:
                    good_candidates.append([x_, y_, z_])
                if success > score:
                    chosen = [x, y, z, w]
                    score = success
    # return qx, qy, qz, qw
    if score > 0.89:
        x, y, z, w = chosen
        pose_.pose.orientation.x = x
        pose_.pose.orientation.y = y
        pose_.pose.orientation.z = z
        pose_.pose.orientation.w = w
        cartesian_move_to(pose_, execute=True)
        rospy.loginfo("Chosen candiate is : "+str(chosen))
        rospy.loginfo("the following are good good_candidates for \
            pose selection: "+ str(good_candidates))
    else:
        rospy.loginfo("Could not optimize cartesian manipulation planning with highest score = "+str(score))
    # return chosen

if __name__ == '__main__':
    rospy.init_node('pick_aruco_demo')
    sphere = SphericalService()
    rospy.spin()

