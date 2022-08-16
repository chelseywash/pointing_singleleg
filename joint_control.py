#!/usr/bin/env python3
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from typing import Tuple, List, Dict, Union
import rospy


# noinspection PyPep8Naming
class JointConstants:
    L1_J1_BASE = "L1_J1_base"
    L1_J2_SHOULDER = "L1_J2_shoulder"
    L1_J3_ELBOW = "L1_J3_elbow"
    L2_J1_BASE = "L2_J1_base"
    L2_J2_SHOULDER = "L2_J2_shoulder"
    L2_J3_ELBOW = "L2_J3_elbow"
    L3_J1_BASE = "L3_J1_base"
    L3_J2_SHOULDER = "L3_J2_shoulder"
    L3_J3_ELBOW = "L3_J3_elbow"
    L4_J1_BASE = "L4_J1_base"
    L4_J2_SHOULDER = "L4_J2_shoulder"
    L4_J3_ELBOW = "L4_J3_elbow"
    L5_J1_BASE = "L5_J1_base"
    L5_J2_SHOULDER = "L5_J2_shoulder"
    L5_J3_ELBOW = "L5_J3_elbow"
    L6_J1_BASE = "L6_J1_base"
    L6_J2_SHOULDER = "L6_J2_shoulder"
    L6_J3_ELBOW = "L6_J3_elbow"
    JOINT_NAMES = (L1_J1_BASE, L1_J2_SHOULDER, L1_J3_ELBOW, L2_J1_BASE, L2_J2_SHOULDER, L2_J3_ELBOW,
                   L3_J1_BASE, L3_J2_SHOULDER, L3_J3_ELBOW, L4_J1_BASE, L4_J2_SHOULDER, L4_J3_ELBOW,
                   L5_J1_BASE, L5_J2_SHOULDER, L5_J3_ELBOW, L6_J1_BASE, L6_J2_SHOULDER, L6_J3_ELBOW)
    JOINT_NAMES2 = tuple(["Lily/" + name for name in JOINT_NAMES])
    JOINT_CONTROLLER_NAME = "JointController"
    EIGHTEEN_NANS = (np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan,
                     np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan)
    EIGHTEEN_ZEROS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


class JointsByIndex:
    L1_J1_BASE = 0
    L1_J2_SHOULDER = 1
    L1_J3_ELBOW = 2
    L2_J1_BASE = 3
    L2_J2_SHOULDER = 4
    L2_J3_ELBOW = 5
    L3_J1_BASE = 6
    L3_J2_SHOULDER = 7
    L3_J3_ELBOW = 8
    L4_J1_BASE = 9
    L4_J2_SHOULDER = 10
    L4_J3_ELBOW = 11
    L5_J1_BASE = 12
    L5_J2_SHOULDER = 13
    L5_J3_ELBOW = 14
    L6_J1_BASE = 15
    L6_J2_SHOULDER = 16
    L6_J3_ELBOW = 17


class JointController(object):

    def __init__(self,
                 joint_states_topic: str = "/joint_states",
                 joint_target_topic: str = "/joint_target",
                 joint_waypoints_topic: str = "/joint_waypoints",
                 prepend_lily_to_joint_name: bool = False,
                 suppress_init: bool = False
                 ):
        if not suppress_init:
            rospy.init_node("JointController")
            rospy.loginfo("JointController node inited.")
        else:
            rospy.logdebug("JointController is not a node.")
        self.state_names: Tuple[str, ...] = JointConstants.JOINT_NAMES if not prepend_lily_to_joint_name else JointConstants.JOINT_NAMES2
        self.joint_states_topic: str = joint_states_topic
        self.joint_target_topic: str = joint_target_topic
        self.joint_waypoints_topic = joint_waypoints_topic
        self.state_positions: List[float] = []
        self.state_vel: List[float] = []
        self.state_eff: List[float] = []
        self._ready: bool = False
        self.sub_joint_state: rospy.Subscriber = rospy.Subscriber(self.joint_states_topic, JointState,
                                                                  self._joint_callback)
        
        self.pub_joint_command_waypoints: rospy.Publisher = rospy.Publisher(self.joint_waypoints_topic,
                                                                            JointTrajectory, queue_size=10)
        self.pub_joint_command_target_point: rospy.Publisher = rospy.Publisher(self.joint_target_topic,
                                                                               JointTrajectoryPoint, queue_size=10)
        rospy.loginfo("JointController Instantiated")

    def is_ready(self) -> bool:
        return self._ready

    def get_joint_state_dict(self) -> Dict[str, float]:
        return {a: b for a, b in zip(self.state_names, self.state_positions)}

    def get_joint_state_list(self) -> Tuple[List[float], Union[List[str], Tuple[str, ...]]]:
        assert len(self.state_positions) > 0
        return self.state_positions, self.state_names

    def _joint_callback(self, data: JointState):
        # TODO Aaron Fix Properly
        self.state_names = tuple(data.name)
        if set(self.state_names) != set(JointConstants.JOINT_NAMES) and set(self.state_names) != set(JointConstants.JOINT_NAMES2):
            raise ValueError("Joint name mismatch:\nFound: %s\nExpected: %s"
                             "" % (str(self.state_names), str(JointConstants.JOINT_NAMES)))
        self.state_positions = data.position
        self.state_vel = data.velocity
        self.state_eff = data.effort
        self._ready = True

    def get_state_positions(self) -> List[float]:
        return self.state_positions

    @staticmethod
    def _create_joint_trajectory_point_msg(positions: List[float],
                                           velocities: List[float] = JointConstants.EIGHTEEN_ZEROS,
                                           accelerations: List[float] = JointConstants.EIGHTEEN_ZEROS,
                                           effort: List[float] = JointConstants.EIGHTEEN_ZEROS,
                                           ) -> JointTrajectoryPoint:
        jtp: JointTrajectoryPoint = JointTrajectoryPoint()
        jtp.positions = positions
        jtp.velocities = velocities
        jtp.accelerations = accelerations
        jtp.effort = effort
        return jtp

    # TODO what was I thinking with this method it doesn't match the return signature?
    @staticmethod
    def _create_joint_trajectory_msg(names: List[str], positions: List[float]) -> JointTrajectory:
        jtp: JointTrajectoryPoint = JointTrajectoryPoint()
        jtp.positions = positions
        # jtp.effort = [0] * len(positions)
        jtp.velocities = [0] * len(positions)
        jtp.accelerations = [0] * len(positions)
        jt: JointTrajectory = JointTrajectory()
        jt.joint_names = names
        jt.points = [jtp]
        return jt


class JointTargetController(JointController):

    def to_position_partially_specified(self, position: Dict[JointsByIndex, float]):
        pos: List[float] = self.state_positions
        for pidx, val in position:
            pos[pidx] = val
        self.to_position(pos)

    def to_position_by_delta(self, delta: List[float]):
        pos: List[float] = self.state_positions
        for i in range(len(pos)):
            pos[i] += delta[i]
        self.to_position(pos)

    def to_position(self, position: List[float]):
        self.specify_full_trajectory_point(position=position,
                                           velocity=list(JointConstants.EIGHTEEN_ZEROS),
                                           acceleration=list(JointConstants.EIGHTEEN_ZEROS),
                                           effort=list(JointConstants.EIGHTEEN_ZEROS))

    def set_velocity_partially_specified(self, velocity: Dict[JointsByIndex, float]):
        vel: List[float] = self.state_vel
        for pidx, val in velocity:
            vel[pidx] = val
        self.set_velocity(vel)

    def set_velocity(self, velocity: List[float]):
        self.specify_full_trajectory_point(position=list(JointConstants.EIGHTEEN_NANS),
                                           velocity=velocity,
                                           effort=list(JointConstants.EIGHTEEN_NANS),
                                           acceleration=list(JointConstants.EIGHTEEN_NANS))

    def specify_full_trajectory_point(self, position: List[float],
                                      velocity: List[float],
                                      acceleration: List[float],
                                      effort: List[float]) -> None:
        jtp: JointTrajectoryPoint = self._create_joint_trajectory_point_msg(positions=position,
                                                                            velocities=velocity,
                                                                            accelerations=acceleration,
                                                                            effort=effort)
        self.pub_joint_command_target_point.publish(jtp)
        # used to show the message being published to joint_target_topic
        # rospy.loginfo(jtp)



# TODO this does not yet work
class JointWaypointController(JointController):

    # specify a subset of positions to specify position of, all others will be same position
    def to_position(self, position: Dict[str, float]):
        new_state: Dict[str, float] = self.get_joint_state_dict()
        for k in position:
            new_state[k] = position[k]
        self.to_position_fully_specified(new_state)

    # specify a subset of positions to specify delta of, all others will be same position
    def delta_position(self, deltas: Dict[str, float]):
        new_state: Dict[str, float] = self.get_joint_state_dict()
        for k in deltas:
            new_state[k] = new_state[k] + deltas[k]
        self.to_position_fully_specified(new_state)

    # specify position to go to
    def to_position_fully_specified(self, positions: Dict[str, float]) -> None:
        if not set(positions.keys()) == set(self.state_names) :
            raise ValueError("positions argument is not fully/correctly specified, all %d elements required: %s"
                             "" % (len(self.state_names), str(self.state_names)))
        if not self._ready:
            raise rospy.ROSException("Have not yet received data from %s" % self.joint_states_topic)
        (names, pos_values) = zip(*positions.items())
        jtraj: JointTrajectory = self._create_joint_trajectory_msg(names, pos_values)
        self.pub_joint_command_waypoints.publish(jtraj)
