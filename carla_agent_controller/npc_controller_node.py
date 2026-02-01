#!/usr/bin/env python3

# ros
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

# lib
import uuid
import carla

# autoware
from autoware_perception_msgs.msg import PredictedObjects

# util
from carla_agent_controller.util import ros_2_carla_pose


class AgentController(Node):
    """
    to do: add documentation
    """

    def __init__(self):
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
        self.veh_bp = bp_lib.find("vehicle.tesla.model3")

        # set subscriber
        self.subscription = self.create_subscription(
            PredictedObjects,
            "/perception/object_recognition/objects",
            self.callback,
            10,
        )

        # for managing the state of NPC
        self.npc_map = {}

    def callback(self, msg: PredictedObjects) -> None:
        predictedObjects = msg.objects
        msg_uuid_set = set()
        if msg.objects is None:
            self.npc_map = []
        for object in predictedObjects:
            spawn_pose = self.ros_2_carla_pose(
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
                spawn_pose.location.z = 1.0
                self.npc_map[uuid] = self.world.spawn_actor(self.veh_bp, spawn_pose)
            except Exception as e:
                self.get_logger().warning(f"{e}")
        return

    def destroy_node(self) -> None:
        self.get_logger().info("carla_agent_controller_node is destroy")
        for actor in self.npc_map.values():
            if actor is not None:
                actor.destroy()
        return


def main():
    rclpy.init()
    node = None
    try:
        node = AgentController()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
