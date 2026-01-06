#!/usr/bin/env python3

# ros
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

# lib
import math
import uuid
import carla
import numpy as np
from tf_transformations import (
    euler_matrix,
    quaternion_from_matrix,
    quaternion_matrix,
    euler_from_quaternion,
)

# autoware
from autoware_perception_msgs.msg import PredictedObjects


class CarlaAgentController(Node):
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
        self.world = self.client.load_world(map_name)
        self.get_logger().info(f"Map name: {self.world.get_map().name}")

        bp_lib = self.world.get_blueprint_library()
        self.veh_bp = bp_lib.find("vehicle.tesla.model3")

        # set subscriber
        self.subscription = self.create_subscription(
            PredictedObjects,
            "/perception/object_recognition/objects",
            self.callback,
            10,
        )

        # Transformation Matrix
        self.T_mgrs_carla = np.eye(4, dtype=np.float64)
        # rotation
        roll = self.declare_parameter("euler.roll", 3.141592653589793).value
        pitch = self.declare_parameter("euler.pitch", 0.0).value
        yaw = self.declare_parameter("euler.yaw", 0.0).value
        matrix = euler_matrix(roll, pitch, yaw, axes="sxyz").astype(np.float64)
        self.T_mgrs_carla = matrix
        # translation
        self.T_mgrs_carla[0][3] = (
            self.declare_parameter("translation.x", -81655.015625).value
        )
        self.T_mgrs_carla[1][3] = (
            self.declare_parameter("translation.y", 50135.9421875).value
        )
        self.T_mgrs_carla[2][3] = (
            self.declare_parameter("translation.z", 43.09799999999389).value
        )

        # for managing the state of NPC
        self.npc_map = {}

    def callback(self, msg: PredictedObjects) -> None:
        self.get_logger().info(f"{self.T_mgrs_carla}")
        predictedObjects = msg.objects
        msg_uuid_set = set()
        if msg.objects is None:
            self.npc_map = []
        for object in predictedObjects:
            spawn_pose = self.get_carla_pose(
                object.kinematics.initial_pose_with_covariance.pose
            )
            object_uuid = uuid.UUID(bytes=bytes(object.object_id.uuid))
            msg_uuid_set.add(object_uuid)
            self.update_npc(object_uuid, spawn_pose)

        for object_uuid in self.npc_map.keys():
            if object_uuid not in msg_uuid_set:
                try:
                    self.npc_map[object_uuid].destroy()
                except Exception as e:
                    self.get_logger().warning(f"{e}")

        return

    def get_carla_pose(self, ros_pose: Pose) -> carla.Transform:
        T_mgrs = np.eye(4, dtype=np.float64)
        q = np.array(
            [
                ros_pose.orientation.x,
                ros_pose.orientation.y,
                ros_pose.orientation.z,
                ros_pose.orientation.w,
            ],
            dtype=np.float64,
        )
        T_mgrs = quaternion_matrix(q).astype(np.float64)

        p = np.array(
            [ros_pose.position.x, ros_pose.position.y, ros_pose.position.z],
            dtype=np.float64,
        )
        T_mgrs[0:3, 3] = p

        T_carla = np.eye(4, dtype=np.float64)
        T_carla = self.T_mgrs_carla @ T_mgrs

        position = T_carla[0:3, 3]
        quaternion = quaternion_from_matrix(T_carla)
        ros_roll, ros_pitch, ros_yaw = euler_from_quaternion(quaternion)
        spawn_pose = carla.Transform(
            carla.Location(
                x=np.float64(position[0]),
                y=np.float64(position[1]),
                z=np.float64(position[2]),
            ),
            carla.Rotation(
                pitch=np.float64(ros_pitch * (180.0 / math.pi)),
                yaw=np.float64(ros_yaw * (180.0 / math.pi)),
                roll=0.0,  # np.float64(ros_roll*(180.0/math.pi))
            ),
        )

        return spawn_pose

    def update_npc(self, uuid: uuid.UUID, spawn_pose: carla.Transform) -> None:
        if uuid in self.npc_map:
            self.get_logger().info("object is already spawn")
            spawn_pose.location.z = self.npc_map[uuid].get_transform().location.z
            try:
                self.npc_map[uuid].set_transform(spawn_pose)
            except Exception as e:
                self.get_logger().warning(f"{e}")
        else:
            self.get_logger().info("object spawn")
            try:
                self.npc_map[uuid] = self.world.spawn_actor(self.veh_bp, spawn_pose)
            except Exception as e:
                self.get_logger().warning(f"{e}")
        return

    def destroy_node(self) -> None:
        self.get_logger().info("Node is destroy")
        for actor in self.npc_map.values():
            if actor is not None:
                actor.destroy()
        return


def main():
    rclpy.init()
    node = None
    try:
        node = CarlaAgentController()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
