#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from robotiq_85_gripper_control.msg import Robotiq85Gripper_robot_output
from math import pi

def activate_gripper(pub):
    cmd = Robotiq85Gripper_robot_output()
    cmd.rACT = 1  # Activar el gripper
    cmd.rGTO = 1  # Habilitar la acción de agarre
    cmd.rSPA = 255  # Velocidad máxima
    cmd.rFRA = 150  # Fuerza de agarre
    pub.publish(cmd)
    rospy.sleep(1)

def open_gripper(pub):
    cmd = Robotiq85Gripper_robot_output()
    cmd.rACT = 1  # Activar el gripper
    cmd.rGTO = 1  # Habilitar la acción de agarre
    cmd.rSPA = 255  # Velocidad máxima
    cmd.rFRA = 150  # Fuerza de agarre
    cmd.rPR = 0  # Posición abierta
    pub.publish(cmd)
    rospy.sleep(1)

def close_gripper(pub):
    cmd = Robotiq85Gripper_robot_output()
    cmd.rACT = 1  # Activar el gripper
    cmd.rGTO = 1  # Habilitar la acción de agarre
    cmd.rSPA = 255  # Velocidad máxima
    cmd.rFRA = 150  # Fuerza de agarre
    cmd.rPR = 255  # Posición cerrada
    pub.publish(cmd)
    rospy.sleep(1)

def plan_cartesian_path(group, z_offset):
    waypoints = []
    current_pose = group.get_current_pose().pose
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = current_pose.position.x
    target_pose.position.y = current_pose.position.y
    target_pose.position.z = current_pose.position.z + z_offset
    target_pose.orientation = current_pose.orientation
    waypoints.append(target_pose)

    (plan, fraction) = group.compute_cartesian_path(
        waypoints,
        0.01,
        0.0
    )

    return plan, fraction

def move_to_pose(group, pose_target):
    group.set_pose_target(pose_target)
    plan = group.plan()
    if plan and len(plan.joint_trajectory.points) > 0:
        group.execute(plan, wait=True)
    else:
        rospy.logwarn("No se encontró un plan válido para la pose objetivo")

def pick_and_place(group, gripper_pub, pick_pose, place_pose, approach_distance=0.1):
    # Aproximar por arriba del objeto
    approach_pick = geometry_msgs.msg.Pose()
    approach_pick.position.x = pick_pose.position.x
    approach_pick.position.y = pick_pose.position.y
    approach_pick.position.z = pick_pose.position.z + approach_distance
    approach_pick.orientation = pick_pose.orientation

    move_to_pose(group, approach_pick)

    # Bajar en línea recta
    plan, fraction = plan_cartesian_path(group, -approach_distance)
    if fraction == 1.0:
        group.execute(plan, wait=True)
    else:
        rospy.logwarn("No se pudo planear el descenso al pick pose completo")

    close_gripper(gripper_pub)

    # Subir en línea recta
    plan, fraction = plan_cartesian_path(group, approach_distance)
    if fraction == 1.0:
        group.execute(plan, wait=True)
    else:
        rospy.logwarn("No se pudo planear el ascenso tras agarrar el objeto")

    # Aproximar por arriba del lugar de colocación
    approach_place = geometry_msgs.msg.Pose()
    approach_place.position.x = place_pose.position.x
    approach_place.position.y = place_pose.position.y
    approach_place.position.z = place_pose.position.z + approach_distance
    approach_place.orientation = place_pose.orientation

    move_to_pose(group, approach_place)

    # Bajar en línea recta
    plan, fraction = plan_cartesian_path(group, -approach_distance)
    if fraction == 1.0:
        group.execute(plan, wait=True)
    else:
        rospy.logwarn("No se pudo planear el descenso al place pose completo")

    open_gripper(gripper_pub)

    # Subir en línea recta
    plan, fraction = plan_cartesian_path(group, approach_distance)
    if fraction == 1.0:
        group.execute(plan, wait=True)
    else:
        rospy.logwarn("No se pudo planear el ascenso tras soltar el objeto")

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_pick_and_place', anonymous=True)

    arm_group = moveit_commander.MoveGroupCommander("manipulator")
    arm_group.set_max_velocity_scaling_factor(0.5)  # Opcional: reduce velocidad
    arm_group.set_max_acceleration_scaling_factor(0.5)  # Opcional: reduce aceleración

    gripper_pub = rospy.Publisher('/Robotiq85GripperRobotOutput', Robotiq85Gripper_robot_output, queue_size=10)
    rospy.sleep(0.5)
    activate_gripper(gripper_pub)
    open_gripper(gripper_pub)

    # Define las poses de pick y place
    pick_pose = geometry_msgs.msg.Pose()
    pick_pose.position.x = 0.5
    pick_pose.position.y = 0.2
    pick_pose.position.z = 0.1
    pick_pose.orientation.x = 0
    pick_pose.orientation.y = 0
    pick_pose.orientation.z = 0
    pick_pose.orientation.w = 1

    place_pose = geometry_msgs.msg.Pose()
    place_pose.position.x = 0.7
    place_pose.position.y = 0.3
    place_pose.position.z = 0.1
    place_pose.orientation.x = 0
    place_pose.orientation.y = 0
    place_pose.orientation.z = 0
    place_pose.orientation.w = 1

    pick_and_place(arm_group, gripper_pub, pick_pose, place_pose)