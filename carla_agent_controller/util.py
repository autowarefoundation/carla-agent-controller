import carla
import numpy as np
import math
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion


def connect_to_carla(
    host: str, port: int, time_out: int, map_name: str
) -> tuple[carla.Client, carla.libcarla.World, carla.BlueprintLibrary]:
    client = carla.Client(host, port)
    client.set_timeout(time_out)
    world = client.get_world()
    if map_name != world.get_map().name:
        world = client.load_world(map_name)
    bp_lib = world.get_blueprint_library()
    return client, world, bp_lib


def ros_pose_to_carla_transform(ros_pose: Pose) -> carla.Transform:
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
            yaw=np.float64((-1) * ros_yaw * (180.0 / math.pi)),
            roll=np.float64(ros_roll * (180.0 / math.pi)),
        ),
    )
    return spawn_pose


# todo: add rosMgrs_to_CarlaPose
