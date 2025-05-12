#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

# Publisher global para el gripper
gripper_pub = None

def open_gripper():
    gripper_group.set_named_target("open")
    gripper_group.go(wait=True)

def close_gripper():
    gripper_group.set_named_target("close")
    gripper_group.go(wait=True)

def move_gripper(position, duration=2.0):
    global gripper_pub
    if gripper_pub is None:
        gripper_pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)
        rospy.sleep(1)  # Espera a que el publisher esté listo

    traj = JointTrajectory()
    traj.joint_names = ['robotiq_85_left_knuckle_joint']

    point = JointTrajectoryPoint()
    point.positions = [position]
    point.time_from_start = rospy.Duration(duration)

    traj.points.append(point)

    rospy.loginfo(f"Moviendo gripper a {position} rad")
    gripper_pub.publish(traj)
    rospy.sleep(duration + 1)

def go_to_pose(pose_target):
    arm_group.set_pose_target(pose_target)
    success = arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()
    return success

def create_pose(x, y, z, roll=0, pitch=0, yaw=0):
    qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
    
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_pick_and_place', anonymous=True)
    rospy.sleep(2)  # Da tiempo para que todo se inicialice

    arm_group = moveit_commander.MoveGroupCommander("manipulator")
    gripper_group = moveit_commander.MoveGroupCommander("gripper")

    arm_group.set_max_velocity_scaling_factor(0.5)
    arm_group.set_max_acceleration_scaling_factor(0.5)

    # Poses para recoger y colocar
    approach_pick = create_pose(0.7, 0.1, 0.3, 0, 3.14, 0)
    pick_pose = create_pose(0.7, 0.1, 0.18, 0, 3.14, 0)
    approach_place = create_pose(-0.5, 0.5, 0.5, 0, 3.14, 0)
    place_pose = create_pose(-0.5, 0.5, 0.4, 0, 3.14, 0)
    #approach_place = create_pose(-0.25, 0.65, 0.7, 0, 3.14, 0)  # ajusta coordenadas
    #place_pose = create_pose(-0.25, 0.65, 0.6, 0, 3.14, 0)      # ajusta coordenadas

    # Secuencia Pick and Place
    if not go_to_pose(approach_pick):
        rospy.logerr("Error al mover a approach_pick"); exit(1)

    if not go_to_pose(pick_pose):
        rospy.logerr("Error al mover a pick_pose"); exit(1)

    move_gripper(0.39)  # Cerrar gripper

    if not go_to_pose(approach_pick):
        rospy.logerr("Error al regresar a approach_pick"); exit(1)

    if not go_to_pose(approach_place):
        rospy.logerr("Error al mover a approach_place"); exit(1)

    if not go_to_pose(place_pose):
        rospy.logerr("Error al mover a place_pose"); exit(1)

    move_gripper(0.0)  # Abrir gripper

    if not go_to_pose(approach_place):
        rospy.logerr("Error al regresar a approach_place"); exit(1)

    rospy.loginfo("✅ Pick and Place completado exitosamente.")
    moveit_commander.roscpp_shutdown()