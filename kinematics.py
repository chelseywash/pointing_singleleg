from typing import List, Any

import hebi
import numpy as np
import math
import joint_control as jc
import rospy as rp
import stand_up_robust as stu
import time as t


def point():
    controller = jc.JointTargetController(prepend_lily_to_joint_name=True, suppress_init=False)  # create object
    rp.sleep(2)
    print(controller.get_state_positions())
    stu.stand_up_from_sit(wave_enabled=False)

    # save current positions
    current_positions = list(controller.get_state_positions())
    print(current_positions)

    count = 0.0
    while count < 1:
        current_positions[13] = -1.57   # [shoulder: radians]
        current_positions[14] = -1.57   # [elbow: radians]
        controller.set_velocity(2)
        stu.go_to_super_waypoint(controller, current_positions,  # moves leg 5
                                 .1, "Pointing Position")
        for i in range(3):
            current_positions[13] = -1.30   # [radians]
            current_positions[14] = -1.32  # [radians]
            stu.go_to_super_waypoint(controller, current_positions,  # moves leg 5
                                 .1, "Pointing Position")
            t.sleep(0.25)
            current_positions[13] = -1.57  # [radians]
            current_positions[14] = -1.57  # [radians]
            stu.go_to_super_waypoint(controller, current_positions,  # moves leg 5
                                     .1, "Pointing Position")
            t.sleep(0.25)

        # t.sleep(1.0)
        current_positions[12] = -1.17
        stu.go_to_super_waypoint(controller, current_positions,  # moves leg 5
                                 .1, "Turning Position")
        for i in range(3):
            current_positions[13] = -1.30   # [radians]
            current_positions[14] = -1.32  # [radians]
            stu.go_to_super_waypoint(controller, current_positions,  # moves leg 5
                                 .1, "Pointing Position")
            t.sleep(0.25)
            current_positions[13] = -1.57  # [radians]
            current_positions[14] = -1.57  # [radians]
            stu.go_to_super_waypoint(controller, current_positions,  # moves leg 5
                                     .1, "Pointing Position")
            t.sleep(0.25)

        # t.sleep(1.0)
        current_positions[12] = 1.17
        stu.go_to_super_waypoint(controller, current_positions,  # moves leg 5
                                 .1, "Turning Position")
        for i in range(3):
            current_positions[13] = -1.30   # [radians]
            current_positions[14] = -1.32  # [radians]
            stu.go_to_super_waypoint(controller, current_positions,  # moves leg 5
                                 .1, "Pointing Position")
            t.sleep(0.25)
            current_positions[13] = -1.57  # [radians]
            current_positions[14] = -1.57  # [radians]
            stu.go_to_super_waypoint(controller, current_positions,  # moves leg 5
                                     .1, "Pointing Position")
            t.sleep(0.25)

        # t.sleep(1.0)

        current_positions[12] = 0.0
        stu.go_to_super_waypoint(controller, current_positions,  # moves leg 5
                                 .1, "Pointing Position")

        count = count + 1

    # back to ground
    current_positions[13] = -0.012
    current_positions[14] = -1.63
    stu.go_to_super_waypoint(controller, current_positions,  # moves leg 5
                             .1, "Pointing Position")
    print('Last Run:', current_positions)


if __name__ == "__main__":
    try:
        point()
    except rospy.ROSInterruptException:  # stops program
        pass
