import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    
    bumperbot_controller_pkg = get_package_share_directory('bumperbot_controller')

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
                                      description="Use simulated time"
    )

    use_python_arg = DeclareLaunchArgument(
        "use_python", default_value="False"
    )

    use_python = LaunchConfiguration("use_python")


    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(get_package_share_directory("bumperbot_controller"), "config", "joy_teleop.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("bumperbot_controller"), "config", "joy_config.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    key_teleop_cpp = Node(
        package="bumperbot_controller",
        executable="key_teleop",
        output="screen",
        prefix="xterm -e",  
        condition=UnlessCondition(use_python),
    )

    key_teleop_py = Node(
        package="bumperbot_controller",
        executable="key_teleop.py",
        output="screen",
        prefix="xterm -e",  # or use "gnome-terminal --" if using GNOME
        condition=IfCondition(use_python),
    )
    
    twist_mux_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("twist_mux"),
            "launch",
            "twist_mux_launch.py"
        ),
        launch_arguments={
            "cmd_vel_out": "bumperbot_controller/cmd_vel_unstamped",
            "config_locks": os.path.join(bumperbot_controller_pkg, "config", "twist_mux_locks.yaml"),
            "config_topics": os.path.join(bumperbot_controller_pkg, "config", "twist_mux_topics.yaml"),
            "config_joy": os.path.join(bumperbot_controller_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    twist_relay_node = Node(
        package="bumperbot_controller",
        executable="twist_relay",
        name="twist_relay",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            use_python_arg,
            joy_teleop,
            joy_node,
            twist_mux_launch,
            twist_relay_node,
            key_teleop_cpp,
            key_teleop_py,
        ]
    )