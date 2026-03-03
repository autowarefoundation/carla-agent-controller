#!/usr/bin/env python3

# ros
import rclpy
from rclpy.node import Node

# carla
import carla

# util
import numpy as np
from carla_agent_controller.util import connect_to_carla

# autoware
from autoware_perception_msgs.msg import TrafficLightGroupArray


class TrafficLightSynchronizer(Node):
    """
        This node synchronize Autoware and carla traffic light.
    Subscribes:
        /perception/traffic_light_recognition/traffic_signals (TrafficLightGroupArray)
            for synchronizing traffic light color.
    """

    def __init__(self):
        super().__init__(
            "carla_trafficlight_synchronizer",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        self.get_logger().info("launch carla_trafficlight_synchronizer")

        # load param
        host = self.get_parameter("host").value
        port = self.get_parameter("port").value
        time_out = self.get_parameter("time_out").value
        map_name = self.get_parameter("map").value

        # load carla simulator
        self.client, self.world, _dummy = connect_to_carla(
            host, port, time_out, map_name
        )
        self.get_logger().info(f"Map name: {self.world.get_map().name}")
        self.carla_traffic_lights = self.world.get_actors().filter(
            "traffic.traffic_light"
        )

        # set subscriber
        self.get_logger().info(f"Set subscriber")
        self.subscription = self.create_subscription(
            TrafficLightGroupArray,
            "input_topic",
            self.callback,
            10,
        )

        # set traffic_right
        self.color_map = {
            1: carla.TrafficLightState.Red,
            2: carla.TrafficLightState.Yellow,
            3: carla.TrafficLightState.Green,
        }

        # create lanelet2_traffic id to carla map
        self.get_logger().info(f"create right map")
        self.trafficlight_dict = self.get_parameters_by_prefix("target_traffic_lights")
        self.light_map = {}
        for key, val in self.trafficlight_dict.items():
            id = key.replace("id_", "")
            target_xy = val.value
            closest_tl = self.find_closest_light(target_xy, self.carla_traffic_lights)
            if closest_tl:
                self.light_map[id] = closest_tl
                self.get_logger().info(
                    f"Lanelet id {id} to CARLA Actor {closest_tl.id}"
                )

    def find_closest_light(self, target_xy, carla_lights):
        min_dist = float("inf")
        closest_actor = None

        # search carla-trafficlight
        for tl in carla_lights:
            tl_loc = tl.get_transform().location
            dist = np.sqrt(
                (tl_loc.x - target_xy[0]) ** 2 + (tl_loc.y - target_xy[1]) ** 2
            )

            if dist < min_dist:
                min_dist = dist
                closest_actor = tl

        if closest_actor is None or min_dist > 10.0:
            self.get_logger().warn(f"No traffic light found within 10m of {target_xy}")
            return None

        return closest_actor

    def callback(self, msg) -> None:
        for signal in msg.traffic_light_groups:
            lanelet_trafficright_id = str(signal.traffic_light_group_id)
            if lanelet_trafficright_id in self.light_map:
                target_carla_light = self.light_map[lanelet_trafficright_id]

                color = self.color_map[signal.elements[0].color]
                target_carla_light.set_state(color)
                target_carla_light.freeze(True)
                self.get_logger().info(
                    f"Sync: ID {lanelet_trafficright_id} set to {color} in CARLA"
                )

        return

    def destroy_node(self) -> None:
        self.get_logger().info("carla_trafficlight_synchronizer is destroy")
        return


def main():
    rclpy.init()
    node = None
    try:
        node = TrafficLightSynchronizer()
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
