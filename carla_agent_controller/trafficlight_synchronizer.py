#!/usr/bin/env python3
import carla

# ros
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

# lanelet2
from lanelet2.projection import UtmProjector, LocalCartesianProjector
from lanelet2.io import Origin, load
from lanelet2.core import TrafficLight

# autoware
from autoware_map_msgs.msg import LaneletMapBin, MapProjectorInfo
from autoware_perception_msgs.msg import TrafficLightGroupArray

# util
from carla_agent_controller.util import connect_to_carla
import numpy as np

# any
import tempfile


class TrafficLightSynchronizer(Node):
    def __init__(self):
        super().__init__("carla_trafficlight_synchronizer")
        self.get_logger().info("launch carla_trafficlight_synchronizer")

        #  get lanelet2_bin and compute traffic_light cordinates
        self.projector_info = self._wait_for_map_projector_info()
        self.get_logger().info(
            f"Projector info  {self.projector_info}."
        )

        lanelet2_map_bin = self._wait_for_map_message()
        self.traffic_light_points = self.search_traffic_light(lanelet2_map_bin)
        self.get_logger().info(
            f"Initialized with {len(self.traffic_light_points)} traffic light points."
        )
        self.get_logger().info(f"Initialized with {self.traffic_light_points}")

        # load param
        host = self.declare_parameter("host", "").value
        port = self.declare_parameter("port", 0).value
        time_out = self.declare_parameter("time_out", 0.0).value
        map_name = self.declare_parameter("map", "").value
        self.use_y_inversion = self.declare_parameter("use_y_inversion", False).value

        # load carla simulator
        self.client, self.world, _dummy = connect_to_carla(
            host, port, time_out, map_name
        )
        self.get_logger().info(f"Map name: {self.world.get_map().name}")
        self.carla_traffic_lights = self.world.get_actors().filter(
            "traffic.traffic_light"
        )

        self.color_map = {
            1: carla.TrafficLightState.Red,
            2: carla.TrafficLightState.Yellow,
            3: carla.TrafficLightState.Green,
        }

        # self.trafficlight_dict = self.get_parameters_by_prefix("target_traffic_lights")
        self.light_map = {}
        for data in self.traffic_light_points:
            id = data["id"]
            if(self.use_y_inversion):
                target_xy = [data["x"], (-1*data["y"])] # invert the y-axis.
            else:
                target_xy = [data["x"], (data["y"])]
            closest_tl = self.find_closest_light(target_xy)
            if closest_tl:
                self.light_map[id] = closest_tl
                self.get_logger().info(
                    f"Lanelet id {id} to CARLA Actor {closest_tl.id}"
                )
        
        # set subscriber
        self.get_logger().info(f"Set subscriber")
        self.subscription = self.create_subscription(
            TrafficLightGroupArray,
            "input_trafficlight",
            self.callback,
            10,
        )

    def _wait_for_map_projector_info(self):
        self._received_map_msg = None
    
        qos = QoSProfile(
            depth=1, 
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
    
        sub = self.create_subscription(
            MapProjectorInfo, 
            "input_projector_info", 
            self._map_callback, 
            qos
        )

        self.get_logger().info("Waiting for LaneletMapBin to extract ProjectorInfo...")
    
        while rclpy.ok():
            if self._received_map_msg is not None:
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        self.destroy_subscription(sub)

        return self._received_map_msg

    def _map_callback(self, msg):
        self._received_map_msg = msg

    def _wait_for_map_message(self):
        self.received_msg = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = self.create_subscription(
            LaneletMapBin, "input_lanelet", self.lanelet_callback, qos
        )

        self.get_logger().info("Waiting for LaneletMapBin on /map/vector_map...")
        while rclpy.ok() and not self.received_msg:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.destroy_subscription(sub)
        return self.received_msg[0]

    def lanelet_callback(self, msg):
        self.received_msg.append(msg)

    def search_traffic_light(self, map_bin):
        self.get_logger().info("Starting map analysis via subprocess...")

        with tempfile.NamedTemporaryFile(suffix=".bin", delete=True) as temp_map:
            # create temp file
            temp_map.write(bytes(map_bin.data))
            temp_map.flush()

            # load  laneletmap
            if self.projector_info.projector_type == "Local":
                projector = UtmProjector(Origin(
                    self.projector_info.map_origin.latitude,
                    self.projector_info.map_origin.longitude,
                    self.projector_info.map_origin.altitude))
            elif self.projector_info.projector_type == "MGRS":
                self.get_logger().WARN("The MGRS hasn't been tested enough.")
                projector = UtmProjector(Origin(
                    0.0,
                    0.0,
                    0.0))
            else:
                raise ValueError(f"Unknown projector type: {self.projector_info.projector_type}")
            l_map = load(temp_map.name, projector)

        points = []
        for reg_el in l_map.regulatoryElementLayer:
            if isinstance(reg_el, TrafficLight):
                for light in reg_el.trafficLights:
                    for pt in light:
                        points.append(
                            {
                                "id": int(reg_el.id),
                                "x": float(pt.x),
                                "y": float(pt.y),
                                "z": float(pt.z),
                            }
                        )
                        break

        return points

    def find_closest_light(self, target_xy):
        min_dist = float("inf")
        closest_actor = None

        # search carla-trafficlight
        for tl in self.carla_traffic_lights:
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
            lanelet_trafficright_id = int(signal.traffic_light_group_id)
            if lanelet_trafficright_id in self.light_map:
                target_carla_light = self.light_map[lanelet_trafficright_id]

                color = self.color_map[signal.elements[0].color]
                target_carla_light.set_state(color)
                target_carla_light.freeze(True)
                self.get_logger().info(
                    f"Sync: ID {lanelet_trafficright_id} set to {color} in CARLA"
                )

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
