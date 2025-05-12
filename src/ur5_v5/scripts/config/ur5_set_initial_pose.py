#!/usr/bin/env python3

import rospy
from controller_manager_msgs.srv import ListControllers
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def wait_for_controller():
    rospy.wait_for_service('/controller_manager/list_controllers')
    list_controllers = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
    while not rospy.is_shutdown():
        controllers = list_controllers()
        for c in controllers.controller:
            if c.name == 'eff_joint_traj_controller' and c.state == 'running':
                rospy.loginfo('Controller is running.')
                return
        rospy.loginfo('Waiting for eff_joint_traj_controller to be running...')
        rospy.sleep(1)

def send_initial_pose():
    pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)  # give publisher time to connect

    traj = JointTrajectory()
    traj.joint_names = [
        'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
        'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
    ]
    point = JointTrajectoryPoint()
    point.positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
    point.velocities = [0.0] * 6
    point.time_from_start = rospy.Duration(1.0)
    traj.points.append(point)

    rospy.loginfo('Publishing initial pose...')
    pub.publish(traj)
    rospy.loginfo('Initial pose published.')

if __name__ == '__main__':
    rospy.init_node('set_initial_pose')
    wait_for_controller()
    send_initial_pose()