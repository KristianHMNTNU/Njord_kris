#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Thrust allocation for Munin.

Takes the desired generalized force tau = [X, Y, N] from the controller and
distributes it to the four fixed thrusters with a pseudo-inverse (skadipy).

  in:  control/force/command          geometry_msgs/Wrench  (force.x, force.y, torque.z)
  out: thruster/<name>/command        geometry_msgs/Wrench  (force.x = thrust in N)
"""

import numpy as np
import rclpy
from rclpy.node import Node
import geometry_msgs.msg
import skadipy


# Must match THRUSTERS in njord_simulator/munin.py and munin.urdf.xacro.
# Body frame is NED: x forward, y starboard. Yaw is the direction of positive thrust.
THRUSTER_ANGLE = np.radians(40.0)
THRUSTERS = {
    #                 position [m]      yaw [rad]
    "fore_port":      ([0.276, -0.330], -THRUSTER_ANGLE),
    "fore_starboard": ([0.276, 0.330], THRUSTER_ANGLE),
    "aft_port":       ([-0.379, -0.324], -(np.pi - THRUSTER_ANGLE)),
    "aft_starboard":  ([-0.379, 0.324], np.pi - THRUSTER_ANGLE),
}


class ThrustAllocation(Node):

    def __init__(self):
        super().__init__("thrust_allocation")

        self.declare_parameter("max_thrust", 1.0)
        self.max_thrust = self.get_parameter("max_thrust").value

        # 1. Describe the thrusters
        self.actuators = [
            skadipy.actuator.Fixed(
                position=skadipy.toolbox.Point([x, y, 0.0]),
                orientation=skadipy.toolbox.Quaternion(axis=(0.0, 0.0, 1.0), radians=yaw),
            )
            for (x, y), yaw in THRUSTERS.values()
        ]

        # 2. Degrees of freedom we control: surge, sway and yaw
        dofs = [
            skadipy.allocator.ForceTorqueComponent.X,
            skadipy.allocator.ForceTorqueComponent.Y,
            skadipy.allocator.ForceTorqueComponent.N,
        ]

        # 3. Allocation method, and build the configuration matrix B
        self.allocator = skadipy.allocator.PseudoInverse(
            actuators=self.actuators, force_torque_components=dofs
        )
        self.allocator.compute_configuration_matrix()

        self.thruster_publishers = {
            name: self.create_publisher(geometry_msgs.msg.Wrench, f"thruster/{name}/command", 10)
            for name in THRUSTERS
        }
        self.create_subscription(
            geometry_msgs.msg.Wrench, "control/force/command", self.cb_force_command, 10
        )

        self.get_logger().info(f"Thrust allocation ready for {list(THRUSTERS)}, max thrust {self.max_thrust} N")

    def cb_force_command(self, msg: geometry_msgs.msg.Wrench):
        tau = np.array([
            msg.force.x, msg.force.y, msg.force.z,
            msg.torque.x, msg.torque.y, msg.torque.z,
        ]).reshape((6, 1))

        # 4. Distribute tau to the thrusters
        self.allocator.allocate(tau=tau)
        forces = np.array([actuator.force[0] for actuator in self.actuators]).flatten()

        # The pseudo-inverse knows nothing about thrust limits. If a thruster is
        # saturated, scale all of them down equally so the direction of tau is kept.
        largest = np.max(np.abs(forces))
        if largest > self.max_thrust:
            forces *= self.max_thrust / largest

        for name, force in zip(THRUSTERS, forces):
            command = geometry_msgs.msg.Wrench()
            command.force.x = float(force)
            self.thruster_publishers[name].publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = ThrustAllocation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
