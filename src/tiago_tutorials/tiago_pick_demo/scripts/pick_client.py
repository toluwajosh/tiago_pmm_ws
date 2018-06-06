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
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from moveit_commander import MoveGroupCommander#, PlanningSceneInterface
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Header

from copy import deepcopy

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

        # for gripper:
        # to send commands
        self.gripper_cmd = rospy.Publisher('/parallel_gripper_controller/follow_joint_trajectory', 
            FollowJointTrajectoryActionGoal, queue_size=1)
        # we also need to know the state of the gripper tho

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

        # get pose of object here:
        # aruco_pose = rospy.wait_for_message('/aruco_single/pose', PoseStamped)
        # aruco_pose.header.frame_id = self.strip_leading_slash(aruco_pose.header.frame_id)

        aruco_pose = PoseStamped()
        aruco_pose.header.frame_id = 'base_footprint'
        rospy.loginfo("Got: " + str(aruco_pose))

        # spherical grasp used
        rospy.loginfo("spherical_grasp_gui: Transforming from frame: " +
        aruco_pose.header.frame_id + " to 'base_footprint'")

        ####################################################
        # # alternative approach for arm manipulation, start
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
        rospy.loginfo("Aruco pose: " + str(aruco_pose))

        waypoints=[]
        waypoints.append(start_pose)

        # # 0.867343 0.020112  0.722288 -0.004629 0.002833 8.8e-05
        aruco_pose.pose.position.x = 0.867343
        # aruco_pose.pose.position.x = 0.536
        # aruco_pose.pose.position.y = -0.049
        aruco_pose.pose.position.y = 0.020112
        # aruco_pose.pose.position.z = 0.930
        aruco_pose.pose.position.z = 0.722288
        aruco_pose.pose.orientation.x = 0.5
        aruco_pose.pose.orientation.y = 0.5
        aruco_pose.pose.orientation.z = 0.5


        aruco_pose.pose.orientation.w = 1.0

        aruco_pose.pose.orientation.y -= 0.1*(1.0/2.0)

        waypoints.append(deepcopy(aruco_pose.pose))
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
                arm.execute(plan)
                rospy.loginfo("path execution complete. ")
                rospy.sleep(2)              
            if  (attempts %100==0 and fraction<0.9):
                rospy.loginfo("path planning failed with only  " + str(fraction*100)+ "% success after  "+ str(maxtries)+" attempts")

        # # alternative approach for arm manipulation, end
        ####################################################

        # create a pose object
        # This part transforms the intended pose relative to the base_footprint
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

            # continue original manipulation from here
            self.detected_pose_pub.publish(pick_g.object_pose)
            rospy.loginfo("Gonna pick:" + str(pick_g))
            self.pick_as.send_goal_and_wait(pick_g)
            rospy.loginfo("Done!")

            result = self.pick_as.get_result()
            if str(moveit_error_dict[result.error_code]) != "SUCCESS":
                rospy.logerr("Failed to pick, not trying further")
                return

            # finished picking ---
            ####################################################
            # # alternative approach for arm manipulation, start - for placing
            start_pose=arm.get_current_pose(end_effector_link).pose

            waypoints=[]
            waypoints.append(start_pose)
            # pick_g.object_pose.pose.orientation.y -= 0.1*(1.0/2.0)

            aruco_pose.pose.orientation.y += 0.05
            aruco_pose.pose.orientation.w = arm.get_current_pose(end_effector_link).pose.orientation.w

            rospy.loginfo("Hand pose:" + str(aruco_pose.pose.orientation.w))

            # back to original pose
            waypoints.append(deepcopy(aruco_pose.pose))
            aruco_pose.pose.orientation.y += 0.05
            waypoints.append(deepcopy(aruco_pose.pose))
            aruco_pose.pose.orientation.y += 0.05
            waypoints.append(deepcopy(aruco_pose.pose))
            aruco_pose.pose.orientation.y += 0.05
            waypoints.append(deepcopy(aruco_pose.pose))
            aruco_pose.pose.orientation.y += 0.05
            waypoints.append(deepcopy(aruco_pose.pose))
            aruco_pose.pose.orientation.w = -1
            waypoints.append(deepcopy(aruco_pose.pose))

            
            fraction=0.0
            maxtries=100
            attempts=0
            while fraction<1.0 and attempts<maxtries:
                (plan,fraction)=arm.compute_cartesian_path(waypoints,0.05,0.0,True)
                attempts+=1
                if attempts %20==0:
                    if (attempts<100):
                        rospy.loginfo("still trying after"+str(attempts)+" attempts...")
                    else : 
                        rospy.loginfo("Finished after  "+str(attempts)+" attempts...")

                if fraction>0.79:
                    rospy.loginfo("path compute successfully. Arm is moving.")
                    print(plan.joint_trajectory.points[len(plan.joint_trajectory.points)-1].positions)
                    print(plan)
                    #for item in plan.joint_trajectory.points[len(plan.joint_trajectory.points)-1]
                    arm.execute(plan)
                    rospy.loginfo("path execution complete. ")
                    rospy.sleep(2)              
                if  (attempts %100==0 and fraction<0.8):
                    rospy.loginfo("path planning failed with only  " + str(fraction*100)+ "% success after  "+ str(maxtries)+" attempts")

            # # alternative approach for arm manipulation, end
            ####################################################


            # open the gripper - user defined
            # self.open_gripper()

            pick_g.object_pose.pose.position.y += 0.05

            gripper_pose=arm.get_current_pose(end_effector_link).pose
            rospy.loginfo("Gripper final pose; "+str(gripper_pose))
            
            self.place_as.send_goal_and_wait(pick_g)
            rospy.loginfo("Done!")

    def lift_torso(self):
        rospy.loginfo("Moving torso up")
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.34]
        jtp.time_from_start = rospy.Duration(2.5)
        jt.points.append(jtp)
        self.torso_cmd.publish(jt)

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

    def close_gripper(self):
        pass

    def open_gripper(self):
        rospy.loginfo("Opening gripper")
        open_cmd = FollowJointTrajectoryGoal()
        open_cmd_header = Header()
        # open_cmd_header.stamp = rospy.Time.now()
        open_cmd_goal_id = GoalID()
        # open_cmd_goal_id.stamp = rospy.Time.now()

        open_cmd.trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
        open_cmd_pt = JointTrajectoryPoint()
        open_cmd_pt.positions = [0.04, 0.04]
        open_cmd.trajectory.points.append(open_cmd_pt)
        open_cmd_go = FollowJointTrajectoryActionGoal(open_cmd_header, open_cmd_goal_id, open_cmd)
        self.gripper_cmd.publish(open_cmd_go)
        # self.gripper_client.send_goal(open_cmd)
        rospy.loginfo("Gripper Opened")


if __name__ == '__main__':
    rospy.init_node('pick_aruco_demo')
    sphere = SphericalService()
    rospy.spin()

