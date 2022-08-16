from hebirobot.go_to_location import Axis, SQUARE_CENTERS, CORNERS, \
    DIRECTION_NORTH, DIRECTION_EAST, DIRECTION_SOUTH, DIRECTION_WEST, CLOCKWISE, COUNTER_CLOCKWISE, \
    SOUTHWEST_MOST_CORNER

from envs.navigation import ViconObject
from sensor_msgs.msg import JointState
from core.policy import Policy, FixedPolicy
# noinspection PyPep8Naming
from hebirobot.joint_control import JointTargetController, JointConstants as JC
from typing import List, Tuple, Union
from dataclasses import dataclass
import rospy
import time
import enum
from hebirobot.stand_up_robust import stand_up_from_sit



class PolicyOutputTypes(enum.Enum):
    POSITION = 0
    VELOCITY = 1
    POSITION_VELOCITY = 2


ROTATE_RIGHT = 1
ROTATE_LEFT = -1


@dataclass
class GoToLocationJCPrimitivePolicies:

    policy_output_type: PolicyOutputTypes
    forward_policy: Policy = None
    backward_policy: Policy = None
    left_policy: Policy = None
    right_policy: Policy = None
    rotate_right_policy: Policy = None
    rotate_left_policy: Policy = None

    @staticmethod
    def load_new(fw_pol: str = 'pols/walk_forward_bc_policy_2022-07-26_10-30.pol', bk_pol: str = None,
                 lf_pol: str = None, rt_pol: str = None, rt_rt_pol: str = None, rt_lf_pol: str = None,
                 output_type: PolicyOutputTypes = PolicyOutputTypes.POSITION):
        prims = GoToLocationJCPrimitivePolicies(PolicyOutputTypes.POSITION)
        if fw_pol is not None:
            prims.load_forward(fw_pol)
        if bk_pol is not None:
            prims.load_backward(bk_pol)
        if lf_pol is not None:
            prims.load_left(lf_pol)
        if rt_pol is not None:
            prims.load_right(rt_pol)
        if rt_rt_pol is not None:
            prims.load_rot_right(rt_rt_pol)
        if rt_lf_pol is not None:
            prims.load_rot_left(rt_lf_pol)
        prims.policy_output_type = output_type
        return prims

    def go_forward(self, observation: List[float]) -> Union[Tuple[float, ...], List[float]]:
        return self.forward_policy.get_action(observation)

    def go_backward(self, observation: List[float]) -> Union[Tuple[float, ...], List[float]]:
        return self.backward_policy.get_action(observation)

    def go_left(self, observation: List[float]) -> Union[Tuple[float, ...], List[float]]:
        return self.left_policy.get_action(observation)

    def go_right(self, observation: List[float]) -> Union[Tuple[float, ...], List[float]]:
        return self.right_policy.get_action(observation)

    def rotate_left(self, observation: List[float]) -> Union[Tuple[float, ...], List[float]]:
        return self.rotate_left_policy.get_action(observation)

    def rotate_right(self, observation: List[float]) -> Union[Tuple[float, ...], List[float]]:
        return self.rotate_right_policy.get_action(observation)

    def load_forward(self, fw_pol: str):
        self.forward_policy = FixedPolicy(saved_pol=fw_pol)

    def load_backward(self, back_pol: str):
        self.backward_policy = FixedPolicy(saved_pol=back_pol)

    def load_left(self, left_pol: str):
        self.forward_policy = FixedPolicy(saved_pol=left_pol)

    def load_right(self, right_pol: str):
        self.backward_policy = FixedPolicy(saved_pol=right_pol)

    def load_rot_left(self, rot_left_pol: str):
        self.forward_policy = FixedPolicy(saved_pol=rot_left_pol)

    def load_rot_right(self, rot_right_pol: str):
        self.backward_policy = FixedPolicy(saved_pol=rot_right_pol)


@dataclass
class GoToLocationJCConfig:
    suppress_init: bool = False  # if True, do not call rospy.init_node
    point_a: Tuple[float, float] = (5.5, 4.7)
    point_b: Tuple[float, float] = (5.5, -5.3)
    step_wait_s: float = 0.1  # how long is a timestep
    robot_topic_name: str = "hex_lily5"
    robo_joints_topic: str = "/joint_states"
    max_angular_robot_speed: float = 1.0
    done_threshold: float = 0.05  # in meters, used in go_to_location_joint_control


class GoToLocationJC(object):

    # TODO try out with JoinControl TrajectoryPoint method in different ways see what happens
    @staticmethod
    def policy_output_to_command(jc: JointTargetController,
                                 policy_output: Union[Tuple[float, ...], List[float]],
                                 policy_type: PolicyOutputTypes) -> None:
        if policy_type == PolicyOutputTypes.POSITION:
            jc.specify_full_trajectory_point(position=policy_output, velocity=JC.EIGHTEEN_NANS,
                                             effort=JC.EIGHTEEN_NANS)
        elif policy_type == PolicyOutputTypes.VELOCITY:
            jc.specify_full_trajectory_point(position=JC.EIGHTEEN_NANS, velocity=policy_output, effort=JC.EIGHTEEN_NANS)
        elif policy_type == PolicyOutputTypes.POSITION_VELOCITY:
            jc.specify_full_trajectory_point(position=policy_output[0:18], velocity=policy_output[18:36],
                                             effort=JC.EIGHTEEN_NANS)
        else:
            raise ValueError("Unrecognized policy output type.")

    def __init__(self, c: GoToLocationJCConfig,
                 primitives: GoToLocationJCPrimitivePolicies,
                 go_to_location_obj_is_node: bool = True):
        if go_to_location_obj_is_node:
            rospy.init_node("Hebi_go_to_location_joint_control")
        self.vicon: ViconObject = ViconObject(object_name=c.robot_topic_name)
        self._joint_controller: JointTargetController = JointTargetController(suppress_init=True)
        self._gc: GoToLocationJCConfig = c
        self._prim: GoToLocationJCPrimitivePolicies = primitives
        x, y = self.vicon.get_location()
        self._joints: List[float] = []
        self._joint_sub: rospy.Subscriber = rospy.Subscriber(c.robo_joints_topic, JointState,
                                                             self._hex_joint_callback)
        self._wait_for_state_subscribe_successful()
        print("Ready to go!  Current location: (%f, %f)" % (x, y))

    def go_to_location(self, x: float, y: float, first_axis: Axis = Axis.Y):
        self.spin_until(0)
        if first_axis is Axis.Y:
            self._go_until(y, Axis.Y)
            self._go_until(x, Axis.X)
        elif first_axis is Axis.X:
            self._go_until(x, Axis.X)
            self._go_until(y, Axis.Y)
        else:
            raise NotImplementedError("Not recognized axis.")

    def spin_until(self, val: float, direction: int = ROTATE_RIGHT):
        assert abs(direction) == 1
        _, _, z = self.vicon.get_location_pose()
        while abs(z - val) > self._gc.done_threshold:
            obs: List[float] = self._get_state()
            cmd: Union[List[float], Tuple[float, ...]] = self._prim.rotate_right_policy.get_action(obs) if (
                    direction == ROTATE_RIGHT) else self._prim.rotate_left_policy.get_action(obs)
            self.policy_output_to_command(self._joint_controller, cmd, self._prim.policy_output_type)
            time.sleep(0.4)
            _, _, z = self.vicon.get_location_pose()
        self._stop()

    def _wait_for_state_subscribe_successful(self):
        rospy.loginfo("Waiting for joint message. (Waiting for %s topic to not be empty.)" % self._gc.robo_joints_topic)
        msg = rospy.wait_for_message(self._gc.robo_joints_topic, JointState)
        rospy.loginfo("Joint message received.")
        # noinspection PyTypeChecker
        self._hex_joint_callback(msg)

    def _get_state(self) -> List[float]:
        return self._joints

    def _go_until(self, val: float, a: Axis) -> None:
        xy = self.vicon.get_location()
        axis: int = 0 if a is Axis.X else 1
        while xy[axis] < val:
            obs: List[float] = self._get_state()
            cmd: Union[List[float], Tuple[float, ...]] = (self._prim.right_policy.get_action(obs) if axis else
                                                          self._prim.forward_policy.get_action(obs))
            self.policy_output_to_command(self._joint_controller, cmd, self._prim.policy_output_type)
            time.sleep(0.4)
        while xy[axis] > val:
            obs: List[float] = self._get_state()
            cmd: Union[List[float], Tuple[float, ...]] = (self._prim.left_policy.get_action(obs) if axis else
                                                          self._prim.backward_policy.get_action(obs))
            self.policy_output_to_command(self._joint_controller, cmd, self._prim.policy_output_type)

            time.sleep(0.4)
        self._stop()

    def _stop(self):
        stop_vel: List[float] = [0.0] * 18
        self._joint_controller.set_velocity(stop_vel)

    def _hex_joint_callback(self, data: JointState) -> None:
        assert len(data.position) == 18
        self._joints = list(data.position)


if __name__ == "__main__":
    # create primitive policy data classes for each action space with default policies being loaded
    pos_pols: GoToLocationJCPrimitivePolicies = GoToLocationJCPrimitivePolicies.load_new(
            fw_pol='pols/pos_saved_forward_bc_policy_2022-08-05_12-28.pol', output_type=PolicyOutputTypes.POSITION)
    vel_pols: GoToLocationJCPrimitivePolicies = GoToLocationJCPrimitivePolicies.load_new(
        fw_pol='pols/vel_forward_saved_bc_policy_2022-08-05_14-17.pol', output_type=PolicyOutputTypes.VELOCITY)
    pos_vel_pols: GoToLocationJCPrimitivePolicies = GoToLocationJCPrimitivePolicies.load_new(
        fw_pol='pols/pos_vel_forward_saved_bc_policy_2022-08-05_14-24.pol',
        output_type=PolicyOutputTypes.POSITION_VELOCITY)
    gc = GoToLocationJCConfig(point_a=SQUARE_CENTERS[1],
                              robot_topic_name="hex_lily4")

    # pols = pos_pols
    pols = pos_vel_pols
    gtl: GoToLocationJC = GoToLocationJC(gc, pols)
    stand_up_from_sit(wave_enabled=False, suppress_init=True)
    # gtl.go_to_location(*SQUARE_CENTERS[1])
    # gtl.go_to_location(*CORNERS[SOUTHWEST_MOST_CORNER], first_axis=Axis.X)
    # gtl.go_to_location(-4.0, 4.7)  # walk collection start west

    # DEBUG temporary just to test the forward policy
    # for ts in range(100):
    #   # noinspection PyProtectedMember
    #     obs: List[float] = gtl._get_state()
    #     print(len(obs))
    #     new_position = pols.go_forward(obs)
    #     print(new_position)
    #     gtl._joint_controller.to_position(new_position)
    #     time.sleep(0.1)
    #  for ts in range(100):
    #   # noinspection PyProtectedMember
    #     obs: List[float] = gtl._get_state()
    #     print(len(obs))
    #     new_vel = pols.go_forward(obs)
    #     print(new_vel)
    #     gtl._joint_controller.set_velocity(new_vel)
    #     time.sleep(0.1)
    for ts in range(100):
      # noinspection PyProtectedMember
        obs: List[float] = gtl._get_state()
        print(len(obs))
        pv = pols.go_forward(obs)
        print(pv)
        gtl._joint_controller.specify_full_trajectory_point(
            position=pv[0:18],
            velocity=pv[18:36],
            acceleration=JC.EIGHTEEN_ZEROS,
            effort=JC.EIGHTEEN_ZEROS,
        )
        time.sleep(1)


