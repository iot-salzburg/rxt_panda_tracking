#! /usr/bin/env python2

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math


class panda_moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

        self._planning_group = "panda_arm"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
        self._group.set_max_velocity_scaling_factor(0.2)
        self._group.set_max_acceleration_scaling_factor(0.2)
        self._group.set_goal_position_tolerance(10)
        self._group.set_goal_orientation_tolerance(10)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def set_joint_angles2(self):
        list_joint_values = self._group.get_current_joint_values()
        list_joint_values[6] = list_joint_values[6] - 3.14159
        self.set_joint_angles(list_joint_values)
        return True

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    panda = panda_moveit()

    lst_joint_angles = [math.radians(-40.0837883365),
                          math.radians(-36.4071977201),
                          math.radians(41.3337358411),
                          math.radians(-131.774625362),
                          math.radians(144.446053503),
                          math.radians(160.085656335),
                          math.radians(101.874760213)]

    


    

    
    lst_joint_angles1 = [-2.10487496453, -1.08952988331, -0.128724902482, -2.5020268644, 1.10386924876, 2.59814835606, 2.72855164111]

    lst_joint_angles2 = [-2.12989212618, -0.315124112542, -0.390308081519, -2.6037152449, 1.94523100666, 2.12424094467, 1.41917996051]

    lst_joint_angles3 = [-2.01488530487, -0.00452550545346, -0.328690348709, -2.30402307034, 2.04269225217, 2.08763738966, 1.44880202791]



    while not rospy.is_shutdown():

        panda.set_joint_angles(lst_joint_angles2)

        panda.set_joint_angles(lst_joint_angles1)
        # rospy.sleep(2)
        panda.set_joint_angles(lst_joint_angles2)
        # rospy.sleep(2)
        panda.set_joint_angles(lst_joint_angles3)
        # rospy.sleep(2)
        break

    del panda


if __name__ == '__main__':
    main()
