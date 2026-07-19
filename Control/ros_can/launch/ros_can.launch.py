from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_interface_arg = LaunchConfiguration('can_interface', default='can0')

    node = Node(
        package="ros_can",
        executable="ros_can_node",
        name="ros_can",
        parameters=[
            # False on the real car: nothing publishes /clock there, and True
            # would stamp all sensor msgs with time 0 (breaks SLAM) and
            # disable the /cmd watchdog. Sim launches override this.
            {"use_sim_time": False},
            {"can_debug": 0},
            {"simulate_can": 0},
            {"can_interface": can_interface_arg},
            {"loop_rate": 100},
            {"rpm_limit": 4000},
            # the node declares max_dec (not max_acc/max_braking); 10.0 is
            # the default it was already running with, now explicit
            {"max_dec": 10.0},
            {"cmd_timeout": 0.5}
        ],
        arguments=['--ros-args', '--log-level', 'debug'],
    )

    ld = LaunchDescription()
    # expose can_interface as a launch argument so runtime overrides work
    ld.add_action(DeclareLaunchArgument('can_interface', default_value='can0'))
    ld.add_action(node)
    return ld
