#!/usr/bin/env python3
from tokenize import String
import numpy as np
from typing import List
import rospy
import joint_control as jtc


# ******* ROBOT CANNOT BE IN STANDING POSITION PRIOR TO RUNNING THIS SCRIPT *********

class ZeroValues:
    VELOCITY = list(jtc.JointConstants.EIGHTEEN_ZEROS)
    ACCELERATION = list(jtc.JointConstants.EIGHTEEN_ZEROS)
    EFFORT = list(jtc.JointConstants.EIGHTEEN_ZEROS)


def go_to_super_waypoint(controller: jtc.JointTargetController, target_pos: List[float],
                         tolerance: float, name_of_pos: String) -> String:
    print("entering")
    while (not np.allclose(controller.get_state_positions(), target_pos, 0.0, tolerance, equal_nan=False
                           )) and (not rospy.is_shutdown()):
        rospy.sleep(0.05)
        controller.specify_full_trajectory_point(target_pos, velocity=ZeroValues.VELOCITY,
                                                 acceleration=ZeroValues.ACCELERATION, effort=ZeroValues.EFFORT)
    print("exiting")
    return "Just reached " + str(name_of_pos)


def detect_standing(controller: jtc.JointTargetController, standing_pos: List[float], tolerance: float) -> bool:
    if np.allclose(controller.get_state_positions(), standing_pos, tolerance, tolerance, equal_nan=False):
        rospy.loginfo("Robot's starting position: " + str(controller.get_state_positions()))
        rospy.loginfo("Robot's desired position: " + str(standing_pos))
        return True
    else:
        return False


def go_to_leg_wave_position(controller: jtc.JointTargetController, target_pos: List[float],
                            tolerance: float, name_of_pos: String, joint_number: int) -> String:
    print("entering")
    while (not np.isclose(controller.get_state_positions()[joint_number],
                          target_pos[joint_number], 0.0, tolerance, equal_nan=False)) and (not rospy.is_shutdown()):
        rospy.sleep(0.05)
        new_pos = list(controller.get_state_positions())
        new_pos[joint_number] = target_pos[joint_number]
        # noinspection PyTypeChecker
        controller.specify_full_trajectory_point(tuple(new_pos), velocity=ZeroValues.VELOCITY,
                                                 acceleration=ZeroValues.ACCELERATION, effort=ZeroValues.EFFORT)
    print("exiting")
    return "Just reached " + str(name_of_pos)


def stand_up_from_sit(wave_enabled: bool, suppress_init=False):
    tolerance = 1e-1
    tolerance1 = 0.5
    wave_tolerance = 0.2
    position1 = [0.15, -1.5, 0.0, -0.15, 1.5, 0.0, 0.15, -1.5, 0.0,
                 -0.15, 1.5, 0.0, 0.15, -1.5, 0.0, -0.15, 1.5, 0]
    position2 = [0.15, -1.5, -1.5, -0.15, 1.5, 1.5, 0.15, -1.5, -1.5,
                 -0.15, 1.5, 1.5, 0.15, -1.5, -1.5, -0.15, 1.5, 1.5]
    position3 = [0.15, 0.15, -1.7, -0.15, -0.15, 1.7, 0.15, 0.15, -1.7,
                 -0.15, -0.15, 1.7, 0.15, 0.15, -1.7, -0.15, -0.15, 1.7]

    controller: jtc.JointTargetController = jtc.JointTargetController(prepend_lily_to_joint_name=True,
                                                                      suppress_init=True)
    rospy.sleep(2)

    robot_stance = detect_standing(controller, position3, 0.5)

    if robot_stance:
        rospy.logerr("Could not execute because robot is in unsafe position! Re-orient robot and try again!")
        return

    # getting to initial upright position 
    print(go_to_super_waypoint(controller, position1, tolerance, "position1"))
    print(go_to_super_waypoint(controller, position2, tolerance, "position2"))
    print(go_to_super_waypoint(controller, position3, tolerance1, "position3"))
    rospy.sleep(1)
    rospy.loginfo("Current joint_state: " + str(controller.get_state_positions()))

    if not wave_enabled:
        rospy.logwarn("Wave motion not enabled")
        return 

    new_rest_pos = controller.get_state_positions()
    leg1_lifted = list(new_rest_pos)
    leg2_lifted = list(new_rest_pos)
    leg3_lifted = list(new_rest_pos)
    leg4_lifted = list(new_rest_pos)
    leg5_lifted = list(new_rest_pos)
    leg6_lifted = list(new_rest_pos)
    leg1_lifted[1] -= 0.3
    leg2_lifted[4] += 0.3
    leg3_lifted[7] -= 0.3
    leg4_lifted[10] += 0.3
    leg5_lifted[13] -= 0.3
    leg6_lifted[16] += 0.3

    rospy.loginfo(str(new_rest_pos))
    rospy.loginfo(str(leg1_lifted))
    rospy.sleep(1)

    # wave-like motion to adjust elbow of each leg
    print(go_to_leg_wave_position(controller, leg1_lifted, wave_tolerance, "leg1_lifted", 1))
    rospy.sleep(0.25)
    print(go_to_super_waypoint(controller, new_rest_pos, wave_tolerance, "final leg position"))
    rospy.sleep(0.25)
    print(go_to_leg_wave_position(controller, leg2_lifted, wave_tolerance, "leg2_lifted", 4))
    rospy.sleep(0.25)
    print(go_to_super_waypoint(controller, new_rest_pos, wave_tolerance, "final leg position"))
    rospy.sleep(0.25)
    print(go_to_leg_wave_position(controller, leg3_lifted, wave_tolerance, "leg3_lifted", 7))
    rospy.sleep(0.25)
    print(go_to_super_waypoint(controller, new_rest_pos, wave_tolerance, "final leg position"))
    rospy.sleep(0.25)
    print(go_to_leg_wave_position(controller, leg4_lifted, wave_tolerance, "leg4_lifted", 10))
    rospy.sleep(0.25)
    print(go_to_super_waypoint(controller, new_rest_pos, wave_tolerance, "final leg position"))
    rospy.sleep(0.25)
    print(go_to_leg_wave_position(controller, leg5_lifted, wave_tolerance, "leg5_lifted", 13))
    rospy.sleep(0.25)
    print(go_to_super_waypoint(controller, new_rest_pos, wave_tolerance, "final leg position"))
    rospy.sleep(0.25)
    print(go_to_leg_wave_position(controller, leg6_lifted, wave_tolerance, "leg6_lifted", 16))
    rospy.sleep(0.25)
    print(go_to_super_waypoint(controller, new_rest_pos, wave_tolerance, "final leg position"))

    
if __name__ == "__main__":
    try:
        stand_up_from_sit(wave_enabled=False)
    except rospy.ROSInterruptException:
        pass
