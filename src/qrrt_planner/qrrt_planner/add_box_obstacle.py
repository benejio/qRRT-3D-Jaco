"""
add_box_obstacle.py

Summary:
    This ROS 2 node adds or removes simple collision objects in the MoveIt
    planning scene. It is used to create obstacle scenarios for the JACO arm
    planning experiments.

    Supported obstacle shapes:
        - box
        - sphere
        - cylinder

    The node talks to MoveIt through the /apply_planning_scene service.
    When adding an object, it creates a CollisionObject with the requested
    shape, size, pose, and frame, then sends it as a planning-scene diff.
    When removing an object, it sends a REMOVE operation for the given object ID.

Example usage:
    Add a box:
        ros2 run qrrt_planner add_box_obstacle \
            --id box1 \
            --shape box \
            --x 0.25 --y 0.00 --z 0.38 \
            --sx 0.30 --sy 0.95 --sz 0.10

    Remove an object:
        ros2 run qrrt_planner add_box_obstacle --remove --id box1
"""

import argparse

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class PlanningSceneObstacleClient(Node):
    """
    ROS 2 node that sends obstacle add/remove requests to MoveIt.

    This class wraps the /apply_planning_scene service so the planner can
    programmatically add benchmark obstacles before running A*, classical RRT,
    or quantum RRT experiments.
    """

    def __init__(self):
        """
        Create the ROS node and initialize the ApplyPlanningScene client.

        The service is provided by MoveIt. It must be available before
        obstacles can be added or removed.
        """
        super().__init__("add_obstacle_client")
        self.client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def wait_for_service(self):
        """
        Block until MoveIt's /apply_planning_scene service is available.

        This prevents the script from trying to add/remove obstacles before
        MoveIt has fully started.
        """
        self.get_logger().info("Waiting for /apply_planning_scene ...")
        self.client.wait_for_service()
        self.get_logger().info("Connected to /apply_planning_scene")

    def apply_primitive(
        self,
        object_id: str,
        frame_id: str,
        shape: str,
        x: float,
        y: float,
        z: float,
        sx: float,
        sy: float,
        sz: float,
    ) -> bool:
        """
        Add a primitive collision object to the MoveIt planning scene.

        Args:
            object_id:
                Unique name for the object in the planning scene.
            frame_id:
                Coordinate frame where the object pose is defined.
                Usually "world".
            shape:
                Shape type to add: "box", "sphere", or "cylinder".
            x, y, z:
                Position of the object center in the selected frame.
            sx, sy, sz:
                Shape dimensions.
                For boxes:
                    sx = x size, sy = y size, sz = z size.
                For spheres:
                    sx = radius. sy and sz are ignored.
                For cylinders:
                    sx = radius, sz = height. sy is ignored.

        Returns:
            True if MoveIt accepts the planning-scene update, otherwise False.
        """
        collision = CollisionObject()
        collision.id = object_id
        collision.header.frame_id = frame_id

        primitive = SolidPrimitive()

        # Configure the primitive dimensions according to the requested shape.
        if shape == "box":
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [sx, sy, sz]
        elif shape == "sphere":
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions = [sx]
        elif shape == "cylinder":
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [sz, sx]
        else:
            self.get_logger().error(f"Unsupported shape: {shape}")
            return False

        # Define the obstacle pose. Orientation is identity, so the object is
        # axis-aligned with the frame.
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        # Attach the primitive geometry and pose to the collision object.
        collision.primitives.append(primitive)
        collision.primitive_poses.append(pose)
        collision.operation = CollisionObject.ADD

        # Mark this as a planning-scene diff so MoveIt updates the current scene
        # instead of replacing the whole scene.
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(collision)

        request = ApplyPlanningScene.Request()
        request.scene = scene

        # Send the update request and wait for MoveIt to respond.
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("Service call failed.")
            return False

        ok = bool(future.result().success)
        if ok:
            self.get_logger().info(
                f"Added {shape} '{object_id}' in frame '{frame_id}' at "
                f"({x:.3f}, {y:.3f}, {z:.3f})"
            )
        else:
            self.get_logger().error("MoveIt rejected the planning scene update.")

        return ok

    def remove_object(self, object_id: str, frame_id: str) -> bool:
        """
        Remove a collision object from the MoveIt planning scene.

        Args:
            object_id:
                Name of the object to remove.
            frame_id:
                Frame associated with the object. Usually "world".

        Returns:
            True if MoveIt accepts the removal request, otherwise False.
        """
        collision = CollisionObject()
        collision.id = object_id
        collision.header.frame_id = frame_id
        collision.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(collision)

        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("Service call failed.")
            return False

        ok = bool(future.result().success)
        if ok:
            self.get_logger().info(f"Removed object '{object_id}'")
        else:
            self.get_logger().error("MoveIt rejected the removal request.")

        return ok


def main():
    """
    Parse command-line arguments, create the obstacle client, and add/remove
    the requested planning-scene object.

    This is the entry point used by:
        ros2 run qrrt_planner add_box_obstacle
    """
    parser = argparse.ArgumentParser(
        description="Add or remove an obstacle in the MoveIt planning scene."
    )

    # Object identity and reference frame.
    parser.add_argument("--id", type=str, default="benchmark_obstacle")
    parser.add_argument("--frame", type=str, default="world")

    # Shape type. The dimensions below are interpreted differently depending
    # on this selected shape.
    parser.add_argument("--shape", type=str, default="box", choices=["box", "sphere", "cylinder"])

    # Object center position.
    parser.add_argument("--x", type=float, default=0.45)
    parser.add_argument("--y", type=float, default=0.00)
    parser.add_argument("--z", type=float, default=0.35)

    # Shape dimensions.
    parser.add_argument("--sx", type=float, default=0.20, help="box x-size, sphere radius, cylinder radius")
    parser.add_argument("--sy", type=float, default=0.20, help="box y-size only")
    parser.add_argument("--sz", type=float, default=0.40, help="box z-size or cylinder height")

    # If provided, remove the object instead of adding one.
    parser.add_argument("--remove", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = PlanningSceneObstacleClient()
    node.wait_for_service()

    if args.remove:
        ok = node.remove_object(
            object_id=args.id,
            frame_id=args.frame,
        )
    else:
        ok = node.apply_primitive(
            object_id=args.id,
            frame_id=args.frame,
            shape=args.shape,
            x=args.x,
            y=args.y,
            z=args.z,
            sx=args.sx,
            sy=args.sy,
            sz=args.sz,
        )

    node.destroy_node()
    rclpy.shutdown()

    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
