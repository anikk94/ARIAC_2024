import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory



from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    declared_arguments = []

    parameters = {"use_sim_time": True}

    urdf = os.path.join(get_package_share_directory("ariac_description"), "urdf/ariac_robots/ariac_robots.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("ariac_robots", package_name="ariac_moveit_config")
        .robot_description(urdf)
        .robot_description_semantic(file_path="config/ariac_robots.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("ariac_tutorials") + "/config/moveit_config.yaml"
        )
        .to_moveit_configs()
    )

    parameters.update(moveit_config.to_dict())

    smallcatCCS_node = Node(
        package="smallcat",
        executable="smallcatFrontend.py",
        # name="smallcatCCS", # DON'T USE NAME ARGUMENT BEACSUE MULTIPLE NODES IN THIS .py
        output="screen",
        parameters=[parameters],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("smallcat"), "config", "rviz", "smallcat.rviz")],
        parameters=[parameters],
    )
    
    moveg_roup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_moveit_config"), "/launch", "/ariac_robots_moveit.launch.py"]
        ),
        # condition=IfCondition(str(parameters["use_moveit"]))
    )

    return LaunchDescription(
        [
            moveg_roup_node,
            smallcatCCS_node,
            # rviz_node,
        ]
    )
        