"""Run the Turtle Robot"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('turtle_brick'),
                'launch',
                'run_turtle.launch.py'
            ]),
        ),
        Node(
            package="turtle_brick",
            executable="arena",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("turtle_brick"),
                    "config",
                    "turtle.yaml"
                ]),
                {"frequency": 250.0}
            ],
            remappings=[
                ("/pose", "/turtle1/pose")
            ]
        ),
        Node(
            package="turtle_brick",
            executable="catcher",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("turtle_brick"),
                    "config",
                    "turtle.yaml"
                ])
            ]
        )
    ])