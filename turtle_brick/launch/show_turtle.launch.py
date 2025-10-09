"""Display the turtlebot in rviz."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, EqualsSubstitution, IfElseSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value='turtle', description="The xacro file to load, can be either 'slide' or 'chained'"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description" :
                 Command([ExecutableInPackage("xacro", "xacro"), " ",
                          PathJoinSubstitution(
                              #A plain urdf file is also a valid xacro file so we use xacro here for convenience only
                              [FindPackageShare("turtle_brick"),"urdf","turtle_bot.urdf.xacro"
                               ])])}
            ]
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui"
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution([FindPackageShare("turtle_brick"), "view_robot.rviz"])]
        )
        ])