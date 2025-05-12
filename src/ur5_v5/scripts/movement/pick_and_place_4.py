#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from math import pi

# Inicializa MoveIt! y ROS
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('pick_and_place_node', anonymous=True)

# Inicializa los grupos de movimiento
arm_group = moveit_commander.MoveGroupCommander("manipulator")
gripper_group = moveit_commander.MoveGroupCommander("gripper")

# Define la pose de aproximación al objeto
approach_pick = geometry_msgs.msg.Pose()
approach_pick.orientation.w = 1.0
approach_pick.position.x = 0.4
approach_pick.position.y = 0.0
approach_pick.position.z = 0.3

# Función para mover el brazo a una pose
def go_to_pose(pose_goal):
    waypoints = []
    waypoints.append(pose_goal)

    eef_step = 0.01
    jump_threshold = 0.0

    # ✅ Corrección: evitar colisiones solo como argumento posicional
    (plan, fraction) = arm_group.compute_cartesian_path(
        waypoints,
        eef_step,
        jump_threshold,
        True  # <--- solo posicional, no nombrado
    )

    arm_group.execute(plan, wait=True)

# Función para abrir la garra
def open_gripper():
    joint_goal = gripper_group.get_current_joint_values()
    joint_goal[0] = 0.04  # abrir los dedos
    gripper_group.go(joint_goal, wait=True)
    gripper_group.stop()

# Función para cerrar la garra
def close_gripper():
    joint_goal = gripper_group.get_current_joint_values()
    joint_goal[0] = 0.0  # cerrar los dedos
    gripper_group.go(joint_goal, wait=True)
    gripper_group.stop()

# Secuencia de pick and place
open_gripper()
go_to_pose(approach_pick)
close_gripper()

# Finaliza
moveit_commander.roscpp_shutdown()