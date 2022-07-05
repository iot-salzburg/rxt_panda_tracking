#! /usr/bin/env python2

import rospy
import sys
import copy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class panda_moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg1_node_print_pose_joint_angles', anonymous=True)

        self._planning_group = "panda_arm"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> panda init done." + '\033[0m')

    def print_pose_ee(self):
        pose_values = self._group.get_current_pose().pose

        # Convert Quaternion to Euler (Roll, Pitch, Yaw)
        q_x = pose_values.orientation.x
        q_y = pose_values.orientation.y
        q_z = pose_values.orientation.z
        q_w = pose_values.orientation.w

        quaternion_list = [q_x, q_y, q_z, q_w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)

        rospy.loginfo('\033[94m' + "\n" + "End-Effector ({}) Pose quaternion value : \n\n".format(self._eef_link) +
                      "x: {}\n".format(pose_values.position.x) +
                      "y: {}\n".format(pose_values.position.y) +
                      "z: {}\n\n".format(pose_values.position.z) +
                      "orientation_x: {}\n".format(q_x) +
                      "orientation_y: {}\n".format(q_y) +
                      "orientation_z: {}\n".format(q_z) +
                      "orientation_w: {}\n".format(q_w) +
                      '\033[0m')

        rospy.loginfo('\033[94m' + "\n" + "End-Effector ({}) Pose in euler value: \n\n".format(self._eef_link) +
                      "x: {}\n".format(pose_values.position.x) +
                      "y: {}\n".format(pose_values.position.y) +
                      "z: {}\n\n".format(pose_values.position.z) +
                      "roll: {}\n".format(roll) +
                      "pitch: {}\n".format(pitch) +
                      "yaw: {}\n".format(yaw) +
                      '\033[0m')

    def print_joint_angles(self):
        list_joint_values = self._group.get_current_joint_values()

        rospy.loginfo('\033[94m' + "\nJoint Values: \n\n" +
                      "joint_1: {}\n".format(math.degrees(list_joint_values[0])) +
                      "joint_2: {}\n".format(math.degrees(list_joint_values[1])) +
                      "joint_3: {}\n".format(math.degrees(list_joint_values[2])) +
                      "joint_4: {}\n".format(math.degrees(list_joint_values[3])) +
                      "joint_5: {}\n".format(math.degrees(list_joint_values[4])) +
                      "joint_6: {}\n".format(math.degrees(list_joint_values[5])) +
                      "joint_7: {}\n".format(math.degrees(list_joint_values[6])) +
                      '\033[0m')

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class panda_moveit Deleted." + '\033[0m')


def main():

    panda = panda_moveit()

    while not rospy.is_shutdown():
        panda.print_pose_ee()
        panda.print_joint_angles()
        rospy.sleep(1)

    del panda


if __name__ == '__main__':
    main()

