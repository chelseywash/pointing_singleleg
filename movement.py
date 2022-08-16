from geometry_msgs.msg import Twist


# linear x, y, z; angular x, y, z
def create_twist(lin_x: float, lin_y: float, lin_z: float = 0.0,
                 ang_x: float = 0.0, ang_y: float = 0.0, ang_z: float = 0.0) -> Twist:
    msg: Twist = Twist()
    msg.linear.x = lin_x
    msg.linear.y = lin_y
    msg.linear.z = lin_z
    msg.angular.x = ang_x
    msg.angular.y = ang_y
    msg.angular.z = ang_z
    return msg


STD_SPEED = 0.1


# noinspection PyClassHasNoInit
class PRIMITIVES:
    FORWARD = create_twist(STD_SPEED, 0, 0)
    STOP = create_twist(0, 0, 0, 0, 0, 0)
