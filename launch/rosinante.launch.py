# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import TimerAction


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Start robot in Gazebo simulation.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "teleop_joy",
            default_value="false",
            description="Configure robot for joystick teleop.",
        )
    )



    prefix = LaunchConfiguration("prefix")
    use_sim = LaunchConfiguration("use_sim")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    start_rviz = LaunchConfiguration("start_rviz")
    teleop_joy = LaunchConfiguration("teleop_joy")



    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rosinante_description"), "urdf", "rosinante.urdf.xacro"]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_sim:=",
            use_sim,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rosinante_bringup"),
            "config",
            "rosinante_controllers.yaml",
        ]
    )

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("rosinante_description"),
            "config",
            "rosinante.rviz",
        ]
    )

    odometry_config = PathJoinSubstitution(
        [
            FindPackageShare("rosinante_bringup"),
            "config",
            "odometry.yaml",
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(start_rviz),
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    
    # robot_state_pub_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="both",
    #     parameters=[robot_description],
    #     remappings=[
    #         # ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
    #     ],
    # )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "30"],
    )

    joint_state_broadcaster_spawner_2 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "30"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_velocity_controller", "-c", "/controller_manager"],
    )


    # Delay start of robot_controller after `joint_state_broadcaster`
    # delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=robot_controller_spawner,
    #         on_exit=[joint_state_broadcaster_spawner_2],
    #     )
    # )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delayed_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )



    # NON-ROS2-CONTROL-NODES

    # joystick_node = Node(
    #     package="joy",
    #     executable="joy_node",
    #     output={
    #         "stdout": "screen",
    #         "stderr": "screen",
    #     },
    # )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    odometry_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='odometry_node',
        parameters=[odometry_config]
    )

    map_odometry_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='mocap_odometry_node',
        parameters=[odometry_config]
    )

    ## Teleop config

    teleop_joy_config = PathJoinSubstitution(
        [
            FindPackageShare("rosinante_bringup"),
            "config",
            "teleop_joystick.yaml",
        ]
    )


    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="log",
        arguments=[],
        condition=IfCondition(teleop_joy),
    )

    teleop_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="log",
        parameters=[teleop_joy_config],
        condition=IfCondition(teleop_joy),
    )


    nodes = [
        control_node,
        # robot_state_pub_node,
        robot_controller_spawner,
        # delayed_joint_state_broadcaster_spawner
        joint_state_broadcaster_spawner,
        # delayed_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        # delayed_joint_state_broadcaster_spawner,
        #delayed_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        #TimerAction(period=20.0,
        #    actions=[joint_state_broadcaster_spawner]),
        robot_state_publisher_node,
        rviz_node,
        odometry_node,
        map_odometry_node,
        joy_node,
        teleop_joy_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
