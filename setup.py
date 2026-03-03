from setuptools import setup

package_name = "carla_agent_controller"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/params.yaml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/carla_agent_controller_launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="KazushiFujimoto",
    maintainer_email="kazushi.fujimoto.2@tier4.jp",
    description="carla_agent_controller with Autoware",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "npc_controller_node = carla_agent_controller.npc_controller_node:main",
            "ego_controller_node = carla_agent_controller.ego_controller_node:main",
            "trafficlight_synchronizer_node = carla_agent_controller.trafficlight_synchronizer:main",
        ],
    },
)
