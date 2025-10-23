"""Display the turtlebot in rviz."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command, 
    LaunchConfiguration, 
    PathJoinSubstitution, 
    PythonExpression
)

from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Generate Launch File to display turtlebot"""
    
    use_jsp = LaunchConfiguration('use_jsp')
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_jsp",
            default_value="gui"
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": Command([
                    "xacro ",
                    PathJoinSubstitution([
                        FindPackageShare("turtle_brick"),
                        "urdf",
                        "turtle_bot.urdf.xacro"
                    ])
                ])
            }]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(PythonExpression(["'", use_jsp, "' == 'gui'"]))
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=IfCondition(PythonExpression(["'", use_jsp, "' == 'jsp'"]))
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                PathJoinSubstitution([
                    FindPackageShare("turtle_brick"),
                    "config",
                    "view_robot.rviz"
                ])
            ]
        )
    ])