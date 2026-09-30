#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from njord_simulator.base import BaseSimulator
import numpy as np
import shoeboxpy.model6dof
import skadipy
import rclpy
import geometry_msgs.msg


# Shoebox computes the mass as rho*L*B*T, which for Munin's real dimensions
# (L=1.0, B=0.886, T=0.26) would give ~230 kg. Instead we use Voyager's box and
# scale its mass, added mass and damping matrices up slightly.
VOYAGER_L, VOYAGER_B, VOYAGER_T = 1.0, 0.3, 0.08
MASS_DAMPING_SCALE = 1.2

GM_PHI = 0.483
GM_THETA = 0.483

# Angle between each thruster and the length axis, pointing outwards from its corner.
# At 45 degrees all thrust lines pass through (almost) the same point, since the
# thrusters sit in a near square, and the vessel cannot yaw on its own.
# Must match thruster_angle in munin.urdf.xacro.
THRUSTER_ANGLE = np.radians(40.0)

# Fixed thrusters in each corner of the hulls, measured from the STL.
# Body frame is NED: x forward, y starboard, z down.
# Yaw is the direction of positive thrust.
THRUSTERS = {
    #                 position [m]              yaw [rad]
    "fore_port":      ([0.276, -0.330, 0.124], -THRUSTER_ANGLE),
    "fore_starboard": ([0.276, 0.330, 0.124], THRUSTER_ANGLE),
    "aft_port":       ([-0.379, -0.324, 0.118], -(np.pi - THRUSTER_ANGLE)),
    "aft_starboard":  ([-0.379, 0.324, 0.118], np.pi - THRUSTER_ANGLE),
}
THRUST_LIMIT = 1.0
DEADBAND = 0.05


class MuninSimulator(BaseSimulator):
    """
    Concrete simulator for the Munin vessel.
    Defines vessel geometry, thruster configuration, and topic handling for thruster commands.
    """

    def __init__(self):
        super().__init__(node_name="munin_simulator")

    def _create_vessel(self):
        vessel = shoeboxpy.model6dof.Shoebox(
            L=VOYAGER_L, B=VOYAGER_B, T=VOYAGER_T,
            GM_theta=GM_THETA, GM_phi=GM_PHI,
            eta0=self.eta0.flatten(),
        )
        vessel.m *= MASS_DAMPING_SCALE
        vessel.MRB *= MASS_DAMPING_SCALE
        vessel.MA *= MASS_DAMPING_SCALE
        vessel.D *= MASS_DAMPING_SCALE
        vessel.M_eff = vessel.MRB + vessel.MA
        vessel.invM_eff = np.linalg.inv(vessel.M_eff)
        return vessel

    def _init_allocator(self):
        # This order is important when unpacking the command vector
        actuators = [
            skadipy.actuator.Fixed(
                position=skadipy.toolbox.Point(position),
                orientation=skadipy.toolbox.Quaternion(axis=(0.0, 0.0, 1.0), radians=yaw),
            )
            for position, yaw in THRUSTERS.values()
        ]
        dofs = [
            skadipy.allocator._base.ForceTorqueComponent.X,
            skadipy.allocator._base.ForceTorqueComponent.Y,
            skadipy.allocator._base.ForceTorqueComponent.Z,
            skadipy.allocator._base.ForceTorqueComponent.K,
            skadipy.allocator._base.ForceTorqueComponent.M,
            skadipy.allocator._base.ForceTorqueComponent.N
        ]
        return skadipy.allocator.PseudoInverse(actuators=actuators, force_torque_components=dofs)

    def setup_thrusters(self):
        """
        Sets up thruster publishers/subscriptions for Munin.
        Munin has four fixed thrusters, one element each in the command vector.
        Commands are read from force.x on thruster/<name>/command.
        """
        self.u = np.zeros((len(THRUSTERS), 1))

        self.thruster_subscribers = []
        self.thruster_publishers = []
        for index, name in enumerate(THRUSTERS):
            publisher = self.create_publisher(
                geometry_msgs.msg.WrenchStamped, f"thruster/{name}/issued", 1
            )
            subscriber = self.create_subscription(
                geometry_msgs.msg.Wrench,
                f"thruster/{name}/command",
                lambda msg, index=index, name=name, publisher=publisher:
                    self.cb_thruster(msg, index, name, publisher),
                10,
            )
            self.thruster_publishers.append(publisher)
            self.thruster_subscribers.append(subscriber)

    def cb_thruster(self, msg: geometry_msgs.msg.Wrench, index, name, publisher):
        fx = np.clip(msg.force.x, -THRUST_LIMIT, THRUST_LIMIT)
        if abs(fx) < DEADBAND:
            fx = 0.0
        self.u[index] = fx
        issued = geometry_msgs.msg.WrenchStamped()
        issued.header.frame_id = self._frame(f"{name}_thruster_link")
        issued.header.stamp = self.get_clock().now().to_msg()
        issued.wrench.force.x = fx
        publisher.publish(issued)

# ----------------------------------------------------------------------------
# Main entry point
# ----------------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)

    simulator = MuninSimulator()
    rclpy.spin(simulator)
    simulator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
