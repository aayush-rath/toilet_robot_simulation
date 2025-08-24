from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # URDF from panda_description with required arguments
    robot_description = {
        "robot_description": Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("panda_description"),
                "urdf",
                "panda.urdf.xacro"
            ]),
            " name:=panda",
            " prefix:=panda_",
            " gripper:=true",
            " collision_arm:=true",
            " collision_gripper:=true",
            " safety_limits:=true",
            " safety_position_margin:=0.15",
            " safety_k_position:=100.0",
            " safety_k_velocity:=40.0",
            " ros2_control:=false",  # Set to false since we're not using controllers here
        ])
    }

    # SRDF from panda_moveit_config with required arguments
    robot_description_semantic = {
        "robot_description_semantic": Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("panda_moveit_config"),
                "srdf",
                "panda.srdf.xacro"
            ]),
            " name:=panda",
            " prefix:=panda_",
        ])
    }

    toilet_surface_clean = Node(
        package="panda",
        executable="toilet_surface_clean",
        name="toilet_surface_clean",
        output="screen",
        parameters=[
            robot_description, 
            robot_description_semantic, 
            {"use_sim_time": True}
        ],
    )

    return LaunchDescription([toilet_surface_clean])