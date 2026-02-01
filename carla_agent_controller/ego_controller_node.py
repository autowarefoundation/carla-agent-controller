#!/usr/bin/env python3

# ros
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

# carla
import carla

# util
from carla_agent_controller.util import ros_2_carla_pose


class EgoController(Node):
    """
    to do: add documentation
    """

    def __init__(self, host="127.0.0.1", port=2000, hz=20.0):
        super().__init__("carla_ego_controller")
        self.get_logger().info("launch carla_ego_controller")

        # connect carla
        host = self.declare_parameter("host", "127.0.0.1").value
        port = self.declare_parameter("port", 2000).value
        time_out = self.declare_parameter("time_out", 5.0).value
        self.client = carla.Client(host, port)
        self.client.set_timeout(time_out)
        map_name = self.declare_parameter("map", "").value
        self.world = self.client.get_world()
        if map_name != self.world.get_map().name:
            self.world = self.client.load_world(map_name)
        self.get_logger().info(f"Map name: {self.world.get_map().name}")

        bp_lib = self.world.get_blueprint_library()
        self.veh_bp = bp_lib.find("vehicle.toyota.prius")

        # set subscriber
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/simulation/debug/localization/pose_estimator/pose_with_covariance",
            self.callback,
            10,
        )

        # ego
        self.ego = None

    def callback(self, msg: PoseWithCovarianceStamped) -> None:
        spawn_pose = ros_2_carla_pose(msg.pose.pose)
        self.update_ego_pose(spawn_pose)

        return

    def update_ego_pose(self, spawn_pose: carla.Transform) -> None:
        if self.ego is None:
            spawn_pose.location.z = 1.0
            try:
                self.ego = self.world.spawn_actor(self.veh_bp, spawn_pose)
            except Exception as e:
                self.get_logger().warning(f"{e}")
        else:
            spawn_pose.location.z = self.ego.get_transform().location.z
            try:
                self.ego.set_transform(spawn_pose)
            except Exception as e:
                self.get_logger().warning(f"{e}")
        return

    def destroy_node(self) -> None:
        self.get_logger().info("carla_ego_controlle_node is destroy")
        try:
            self.ego.destroy()
        except:
            pass
        return


def main():
    rclpy.init()
    node = None
    try:
        node = EgoController()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
