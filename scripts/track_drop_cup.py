#! /usr/bin/env python2

import rospy
import sys
import copy
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from moveit_msgs.msg import RobotState
from aruco_recognition.msg import aruco_pose #importing the message file which recognised Aruco data will be published



class CartesianPath:

    # Constructor
    def __init__(self):

        rospy.init_node('panda_track_drop', anonymous=True)
        self.pose_subscriber = rospy.Subscriber("/aruco_data",aruco_pose, self.aruco_data) # subscribing the detected aruco data and callback to "update_pose" function

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

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

    def aruco_data(self,data):
        for i in range(len(data.ar_id)):
            if 2 in data.ar_id:
                index = data.ar_id.index(1)
            else:
                index = len(data.ar_id) + 1

        self.ar_id = np.array(data.ar_id)[index] # aruco id of detected marker
        self.center_x = np.array(data.center_x)[index] # center_x pixel of the detected aruco marker
        self.center_y = np.array(data.center_y)[index] # center_y pixel of the detected aruco marker
        self.angle_aruco = np.array(data.theta)[index]

        # print(self.ar_id, self.center_x, self.center_y,self.angle_aruco)



    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        moveit_robot_state = RobotState()
        self._group.set_max_velocity_scaling_factor(0.20)
        self._group.set_max_acceleration_scaling_factor(0.20)
        self._group.set_goal_orientation_tolerance(0.5)
        self._group.set_goal_position_tolerance(0.05)

        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)  
        wpose.position.y = waypoints[0].position.y + (trans_y)  
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = waypoints[0].orientation.x 
        wpose.orientation.y = waypoints[0].orientation.y 
        wpose.orientation.z = waypoints[0].orientation.z
        wpose.orientation.w = waypoints[0].orientation.w


        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))

        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)



    
    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan



    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')



def main():
    panda = CartesianPath()
    while not rospy.is_shutdown():
        panda.ee_cartesian_translation(-0.1, 0.2,0)
        break
    del panda


if __name__ == '__main__':
    main()