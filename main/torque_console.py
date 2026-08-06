#!/usr/bin/env python3
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from agx_msgs.msg import PhorceFeedback


class TorqueConsole(Node):
    def __init__(self):
        super().__init__("shock_guard_torque_console")
        self.latest = None
        self.last_print = 0.0
        self.create_subscription(PhorceFeedback, "/phorce/feedback", self.callback,
                                 qos_profile_sensor_data)

    def callback(self, msg):
        now = time.monotonic()
        self.latest = msg
        if now - self.last_print < 0.1:
            return
        self.last_print = now
        lines = ["G-SHOCK 실시간 외란/전류 (10 Hz)", "축   valid     DOB(A)   current(A)   상태"]
        for i, axis in enumerate(msg.axis):
            if axis.valid:
                lines.append(f"{i:2d}     yes    {axis.dob_a:+8.3f}   {axis.current_a:+9.3f}   {'FAULT' if axis.fault else 'OK'}")
            elif axis.oper or axis.stale or axis.fault:
                lines.append(f"{i:2d}      no          -           -   stale/fault")
        sys.stdout.write("\033[2J\033[H" + "\n".join(lines) + "\n")
        sys.stdout.flush()


def main():
    rclpy.init()
    node = TorqueConsole()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
