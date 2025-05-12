#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

def open_gripper():
    # Si tienes control por joint
    gripper_group.set_named_target("open")
    gripper_group.go(wait=True)

def close_gripper():
    gripper_group.set_named_target("close")
    gripper_group.go(wait=True)

def go_to_pose(pose_target):
    arm_group.set_pose_target(pose_target)
    plan = arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()

def create_pose(x, y, z, roll=0, pitch=0, yaw=0):
    # Convertir de RPY (roll, pitch, yaw) a quaternion (x, y, z, w)
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

    arm_group = moveit_commander.MoveGroupCommander("manipulator")  # nombre de grupo en MoveIt
    gripper_group = moveit_commander.MoveGroupCommander("gripper")  # nombre de grupo del gripper en MoveIt (ajústalo)

    # Velocidades (opcional)
    arm_group.set_max_velocity_scaling_factor(0.5)
    arm_group.set_max_acceleration_scaling_factor(0.5)

    # 1️⃣ Aproach sobre objeto
    approach_pick = create_pose(0.7, 0.1, 0.3, 0, 3.14, 0)  # ajusta coordenadas
    pick_pose = create_pose(0.7, 0.1, 0.2, 0, 3.14, 0)      # ajusta coordenadas

    # 2️⃣ Aproach sobre posición de depósito
#    approach_place = create_pose(-0.25, 0.65, 0.7, 0, 3.14, 0)  # ajusta coordenadas
 #   place_pose = create_pose(-0.25, 0.65, 0.6, 0, 3.14, 0)      # ajusta coordenadas

    approach_place = create_pose(-0.5, 0.5, 0.5, 0, 3.14, 0)  # ajusta coordenadas
    place_pose = create_pose(-0.5, 0.5, 0.4, 0, 3.14, 0)      # ajusta coordenadas

     # 3️⃣ Regresar a la posición "up"
    arm_group.set_named_target("up")  # Usamos la posición "up" definida
    arm_group.go(wait=True)

    # Mover al approach_pick
    go_to_pose(approach_pick)

    # Bajar al pick_pose
    go_to_pose(pick_pose)

    # Cerrar gripper
    close_gripper()

    # Subir a approach_pick
    go_to_pose(approach_pick)

    # Mover al approach_place
    go_to_pose(approach_place)

    # Bajar al place_pose
    go_to_pose(place_pose)

    # Abrir gripper
    open_gripper()

    # Subir a approach_place
    go_to_pose(approach_place)

    arm_group.set_named_target("up")  # Usamos la posición "up" definida
    arm_group.go(wait=True)

    rospy.loginfo("Pick and Place completado ")

    moveit_commander.roscpp_shutdown()