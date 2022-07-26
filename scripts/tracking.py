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
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from rxt_aruco_recognition.msg import aruco_pose #importing the message file which recognised Aruco data will be published



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
                index1 = data.ar_id.index(2)
            if 0 in data.ar_id:
                index2 = data.ar_id.index(0)
            else:
                index1 = len(data.ar_id) + 1
                index2 = len(data.ar_id) + 1

        self.ar_id = np.array(data.ar_id)[index1] # aruco id of detected marker
        self.center_x = np.array(data.center_x)[index1] # center_x pixel of the detected aruco marker
        self.center_y = np.array(data.center_y)[index1] # center_y pixel of the detected aruco marker
        self.angle_aruco = np.array(data.theta)[index1] # theta/ orientation of the detectected aruco marker

        self.target_ar_id = np.array(data.ar_id)[index2] # aruco id of detected marker
        self.target_center_x = np.array(data.center_x)[index2] # center_x pixel of the detected aruco marker
        self.target_center_y = np.array(data.center_y)[index2] # center_y pixel of the detected aruco marker
        self.target_angle_aruco = np.array(data.theta)[index2] # theta/ orientation of the detectected aruco marker


    def scale_cartesian_speed(self,traj,scale):
        # ref : https://groups.google.com/g/moveit-users/c/7n45S8DUjys
        new_traj = RobotTrajectory()

        # Initialize the new trajectory to be the same as the planned trajectory
        new_traj.joint_trajectory = traj.joint_trajectory
        
        # Get the number of joints involved
        n_joints = len(traj.joint_trajectory.joint_names)
        
        # Get the number of points on the trajectory
        n_points = len(traj.joint_trajectory.points)
        
        # Store the trajectory points
        points = list(traj.joint_trajectory.points)
        
        # Cycle through all points and scale the time from start, speed and acceleration
        for i in range(n_points):
            point = JointTrajectoryPoint()
            point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
            point.velocities = list(traj.joint_trajectory.points[i].velocities)
            point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
            point.positions = traj.joint_trajectory.points[i].positions
                            
            for j in range(n_joints):
                point.velocities[j] = point.velocities[j] * scale
                point.accelerations[j] = point.accelerations[j] * scale * scale
            
            points[i] = point

        # Assign the modified points to the new trajectory
        new_traj.joint_trajectory.points = points

        # Return the new trajecotry
        return new_traj




    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        moveit_robot_state = RobotState()


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
        # num_pts = len(plan.joint_trajectory.points)
        # if (num_pts >= 3):
        #     del plan.joint_trajectory.points[0]
        #     del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path

        plan2 = self.scale_cartesian_speed(plan,0.4)

        self._group.execute(plan2)

    def hard_ee_cartesian_translation(self, trans_x, trans_y, trans_z, arg_max_attempts):
        """hard cartesian performes n number of attempts to reach the traget pose"""

        number_attempts = 0
        flag_success = False

        while ((number_attempts <= arg_max_attempts) and  (flag_success is False)):
            number_attempts += 1
            flag_success = self.ee_cartesian_translation(trans_x, trans_y, trans_z)
            rospy.logwarn("attempts: {}".format(number_attempts))



    
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



    def tracking(self):
        if self.center_x < self.target_center_x:
            position_x_cartesian =  -1 * abs((self.center_x) - (self.target_center_x)) * 0.001262626
        if self.center_x > self.target_center_x:
            position_x_cartesian = abs((self.center_x) - (self.target_center_x)) *  0.001262626
        if self.center_y < self.target_center_y:
            position_y_cartesian = abs((self.center_y) - (self.target_center_y)) * 0.001407407
        if self.center_y > self.target_center_y:
            position_y_cartesian = -1 * abs((self.center_y) - (self.target_center_y)) * 0.00135017

        self.hard_ee_cartesian_translation(position_y_cartesian,position_x_cartesian,0,5)



    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')



def main():
    panda = CartesianPath()
    rospy.sleep(1)
    while not rospy.is_shutdown():
        panda.tracking()
        break
    del panda


if __name__ == '__main__':
    main()
