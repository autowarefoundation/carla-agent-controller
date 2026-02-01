#!/usr/bin/env python3

# ros
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

# lib
import math
import carla
import numpy as np
from tf_transformations import (
    # euler_matrix,
    # quaternion_from_matrix,
    # quaternion_matrix,
    euler_from_quaternion,
)


class EgoController(Node):
    """
    to do: add documentation
    """

    def __init__(self, host="127.0.0.1", port=2000, hz=20.0):
        super().__init__("carla_agent_controller")
        self.get_logger().info("launch carla_agent_controller")

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

        # Transformation Matrix
        # self.T_mgrs_carla = np.eye(4, dtype=np.float64)
        # # rotation
        # roll = self.declare_parameter("euler.roll", 3.141592653589793).value
        # pitch = self.declare_parameter("euler.pitch", 0.0).value
        # yaw = self.declare_parameter("euler.yaw", 0.0).value
        # matrix = euler_matrix(roll, pitch, yaw, axes="sxyz").astype(np.float64)
        # self.T_mgrs_carla = matrix
        # # translation
        # self.T_mgrs_carla[0][3] = self.declare_parameter(
        #     "translation.x", -81655.015625
        # ).value
        # self.T_mgrs_carla[1][3] = self.declare_parameter(
        #     "translation.y", 50135.9421875
        # ).value
        # self.T_mgrs_carla[2][3] = self.declare_parameter(
        #     "translation.z", 43.09799999999389
        # ).value

        # ego
        self.ego = None

    def callback(self, msg: PoseWithCovarianceStamped) -> None:
        spawn_pose = self.get_carla_pose(msg.pose.pose)
        self.update_ego_pose(spawn_pose)

        return

    def get_carla_pose(self, ros_pose: Pose) -> carla.Transform:
        quaternion = np.array(
            [
                ros_pose.orientation.x,
                ros_pose.orientation.y,
                ros_pose.orientation.z,
                ros_pose.orientation.w,
            ],
            dtype=np.float64,
        )
        ros_roll, ros_pitch, ros_yaw = euler_from_quaternion(quaternion)

        # right_hand → left_hand
        spawn_pose = carla.Transform(
            carla.Location(
                x=np.float64(ros_pose.position.x),
                y=np.float64((-1) * ros_pose.position.y),
                z=np.float64(ros_pose.position.z),
            ),
            carla.Rotation(
                pitch=np.float64(ros_pitch * (180.0 / math.pi)),
                yaw=np.float64(ros_yaw * (180.0 / math.pi)),
                roll=np.float64(ros_roll * (180.0 / math.pi)),
            ),
        )
        return spawn_pose

    # def get_carla_pose(self, ros_pose: Pose) -> carla.Transform:
    #     T_mgrs_object = np.eye(4, dtype=np.float64)
    #     q = np.array(
    #         [
    #             ros_pose.orientation.x,
    #             ros_pose.orientation.y,
    #             ros_pose.orientation.z,
    #             ros_pose.orientation.w,
    #         ],
    #         dtype=np.float64,
    #     )
    #     T_mgrs_object = quaternion_matrix(q).astype(np.float64)

    #     p = np.array(
    #         [ros_pose.position.x, ros_pose.position.y, ros_pose.position.z],
    #         dtype=np.float64,
    #     )
    #     T_mgrs_object[0:3, 3] = p

    #     T_carla_object = np.eye(4, dtype=np.float64)
    #     T_carla_object = self.T_carla_mgrs @ T_mgrs_object

    #     position = T_carla_object[0:3, 3]
    #     quaternion = quaternion_from_matrix(T_carla_object)
    #     ros_roll, ros_pitch, ros_yaw = euler_from_quaternion(quaternion)
    #     spawn_pose = carla.Transform(
    #         carla.Location(
    #             x=np.float64(position[0]),
    #             y=np.float64(position[1]),
    #             z=np.float64(position[2]),
    #         ),
    #         carla.Rotation(
    #             pitch=np.float64(ros_pitch * (180.0 / math.pi)),
    #             yaw=np.float64(ros_yaw * (180.0 / math.pi)),
    #             roll=0.0,  # np.float64(ros_roll*(180.0/math.pi))
    #         ),
    #     )

    #     return spawn_pose

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
        self.get_logger().info("Node is destroy")
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
