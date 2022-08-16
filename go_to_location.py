from envs.navigation import ViconObject, NavigationConfig
from geometry_msgs.msg import TransformStamped, Twist
from hebirobot.movement import create_twist, PRIMITIVES
import numpy as np
import enum
import rospy
import time


class Axis(enum.Enum):
    X = 0
    Y = 1


SQUARE_CENTERS = [
    None,
    (5.8, 4.5),
    None,
    None
]

SOUTHWEST_MOST_CORNER = "SWmost-corner"

CORNERS = {
    SOUTHWEST_MOST_CORNER: (9.4, -7.1)
}

DIRECTION_NORTH = np.pi/2
DIRECTION_EAST = 0.0
DIRECTION_WEST = np.pi
DIRECTION_SOUTH = -np.pi/2

CLOCKWISE = -1
COUNTER_CLOCKWISE = 1


class GoToLocation(object):
    
    def __init__(self, c: NavigationConfig, go_to_location_obj_is_node: bool = True):
        if go_to_location_obj_is_node:
            rospy.init_node("Hebi_go_to_location")
        self.vicon: ViconObject = ViconObject(object_name=c.robot_topic_name)
        self._robo_command_topic = c.robo_command_topic
        self._hex_pub: rospy.Publisher = rospy.Publisher(self._robo_command_topic, Twist, queue_size=10)
        self._nc = c
        x, y = self.vicon.get_location()
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

    def spin_until(self, val: float, direction: int = 1):
        assert abs(direction) == 1
        _, _, z = self.vicon.get_location_pose()
        while abs(z - val) > self._nc.done_threshold:
            t: Twist = create_twist(0, 0, ang_z=direction)
            self._hex_pub.publish(t)
            time.sleep(0.4)
            _, _, z = self.vicon.get_location_pose()
        self._stop()

    def _go_until(self, val: float, a: Axis) -> None:
        axis: int = 0 if a is Axis.X else 1
        xy = self.vicon.get_location()
        while xy[axis] < val:
            t: Twist = create_twist(1*(not axis), 1*axis)
            self._hex_pub.publish(t)
            time.sleep(0.4)
            xy = self.vicon.get_location()
        while xy[axis] > val:
            t: Twist = create_twist(-1*(not axis), -1*axis)
            self._hex_pub.publish(t)
            time.sleep(0.4)
            xy = self.vicon.get_location()
        self._stop()

    def _stop(self):
        t = PRIMITIVES.STOP
        self._hex_pub.publish(t)


# TODO for now just goes to center of square 2, in future make it more customizable with command line args
if __name__ == "__main__":
    nc = NavigationConfig(square_center=SQUARE_CENTERS[1],
                          # robot_topic_name = "hex_lily3",
                          robot_topic_name="hex_lily5",
                          )
    gtl: GoToLocation = GoToLocation(nc)
    # gtl.go_to_location(*SQUARE_CENTERS[1])
    gtl.go_to_location(*CORNERS[SOUTHWEST_MOST_CORNER], first_axis=Axis.X)
    # gtl.go_to_location(-4.0, 4.7)  # walk collection start west
