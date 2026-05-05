"""
moveit_collision_checker.py

Summary:
    Provides a ROS 2 wrapper around MoveIt's /check_state_validity service.

    This class is used by the grid planners to ask MoveIt whether a JACO arm
    joint configuration is collision-free. The planners work in discretized
    joint space, but MoveIt performs collision checking using the actual robot
    model, planning scene, and obstacle geometry.

    Main purpose:
        - Convert a joint vector q into a MoveIt RobotState.
        - Send that RobotState to /check_state_validity.
        - Return whether the state is valid.
        - Optionally report collision contact pairs.

    This file is shared by:
        - A* grid planner
        - Classical grid-RRT
        - Quantum grid-RRT

    Requirement:
        MoveIt move_group must be running before this script or any planner
        that uses this class can check collisions.
"""

from typing import List, Optional, Sequence

import numpy as np
import rclpy
from rclpy.node import Node

from moveit_msgs.msg import Constraints, RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState


class MoveItCollisionChecker(Node):
    """
    ROS 2 node that checks robot joint-state validity through MoveIt.

    The planner gives this class a joint vector, and this class asks MoveIt
    whether that robot configuration is valid in the current planning scene.
    """

    def __init__(
        self,
        group_name: str,
        joint_names: Sequence[str],
        service_name: str = "/check_state_validity",
        wait_timeout_sec: float = 10.0,
    ) -> None:
        """
        Create the collision checker and connect to MoveIt's validity service.

        Args:
            group_name:
                MoveIt planning group name. For the JACO arm, this is usually "arm".
            joint_names:
                Ordered joint names matching the joint vector q.
            service_name:
                MoveIt service used for validity checking.
            wait_timeout_sec:
                Maximum time to wait for the service before failing.

        Raises:
            RuntimeError:
                If /check_state_validity is not available.
        """
        super().__init__("moveit_collision_checker")

        self.group_name = group_name
        self.joint_names = list(joint_names)

        # Create a client for MoveIt's state-validity service.
        self._client = self.create_client(GetStateValidity, service_name)

        self.get_logger().info(f"Waiting for service: {service_name}")
        if not self._client.wait_for_service(timeout_sec=wait_timeout_sec):
            raise RuntimeError(
                f"Service {service_name} not available. "
                "Make sure `move_group` is running."
            )

        self.get_logger().info("Connected to MoveIt state validity service.")

    def _make_robot_state(self, q: Sequence[float]) -> RobotState:
        """
        Convert a joint vector into a MoveIt RobotState message.

        Args:
            q:
                Joint positions in the same order as self.joint_names.

        Returns:
            RobotState message containing the requested joint positions.

        Raises:
            ValueError:
                If q does not have the expected number of joint values.
        """
        if len(q) != len(self.joint_names):
            raise ValueError(
                f"Expected {len(self.joint_names)} joints, got {len(q)}."
            )

        rs = RobotState()
        rs.joint_state = JointState()
        rs.joint_state.name = list(self.joint_names)
        rs.joint_state.position = [float(x) for x in q]
        return rs

    def check_state_validity(
        self,
        q: Sequence[float],
        constraints: Optional[Constraints] = None,
    ) -> GetStateValidity.Response:
        """
        Call MoveIt to check whether a joint configuration is valid.

        Args:
            q:
                Joint vector to test.
            constraints:
                Optional MoveIt constraints. If None, an empty Constraints
                message is used.

        Returns:
            Full GetStateValidity service response.

        Raises:
            RuntimeError:
                If the service call fails.
        """
        req = GetStateValidity.Request()
        req.robot_state = self._make_robot_state(q)
        req.group_name = self.group_name
        req.constraints = constraints if constraints is not None else Constraints()

        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            raise RuntimeError("GetStateValidity service call failed.")

        return future.result()

    def is_state_valid(self, q: Sequence[float]) -> bool:
        """
        Return only the boolean validity result for a joint configuration.

        This is the main method used by the planners.

        Args:
            q:
                Joint vector to test.

        Returns:
            True if MoveIt reports the state as valid, otherwise False.
        """
        resp = self.check_state_validity(q)
        return bool(resp.valid)

    def collision_details(self, q: Sequence[float]) -> List[str]:
        """
        Return collision contact information for an invalid state.

        Args:
            q:
                Joint vector to test.

        Returns:
            List of contact-body pairs, such as:
                "robot_link <-> obstacle_name"

            If the state is valid, the list is empty.
        """
        resp = self.check_state_validity(q)
        details: List[str] = []

        if resp.valid:
            return details

        for c in resp.contacts:
            details.append(f"{c.contact_body_1} <-> {c.contact_body_2}")

        return details


def main() -> None:
    """
    Standalone test entry point.

    This lets the file be run directly to test whether MoveIt collision
    checking is working for a sample JACO arm configuration.
    """
    # Example for Kinova JACO j2n6s300 / j2s6s300 style 6-DOF arm.
    joint_names = [
        "j2n6s300_joint_1",
        "j2n6s300_joint_2",
        "j2n6s300_joint_3",
        "j2n6s300_joint_4",
        "j2n6s300_joint_5",
        "j2n6s300_joint_6",
    ]

    # Example joint state to test.
    q_test = np.array([0.0, 0.5, 0.5, 0.0, 0.0, 0.0], dtype=float)

    rclpy.init()
    node = None

    try:
        node = MoveItCollisionChecker(
            group_name="arm",
            joint_names=joint_names,
            service_name="/check_state_validity",
        )

        ok = node.is_state_valid(q_test)
        print("valid:", ok)

        if not ok:
            print("contacts:")
            for item in node.collision_details(q_test):
                print("  ", item)

    finally:
        # Always clean up the ROS node before shutting down rclpy.
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
