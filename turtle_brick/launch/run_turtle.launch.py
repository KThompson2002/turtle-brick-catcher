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
                'show_turtle.launch.py'
            ]),
            launch_arguments={
                'use_jsp': 'jsp'
            }.items()
        ),
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            parameters=[
                {"holonomic": True}
            ]
        ),
        Node(
            package="turtle_brick",
            executable="turtle_robot",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("turtle_brick"),
                    "config",
                    "turtle.yaml"
                ]),
                {"frequency": 100.0}
            ],
            remappings=[
                ("/cmd_vel", "/turtle1/cmd_vel"),
                ("/pose", "/turtle1/pose")
                
            ]
        ),
        
    ])