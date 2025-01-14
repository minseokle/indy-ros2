from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    
    # description_package = FindPackageShare('indy_description')
    gazebo_package = FindPackageShare('indy_gazebo')
    moveit_config_package = FindPackageShare('indy_moveit')

    # Initialize Arguments
    name = LaunchConfiguration("name")
    indy_type = LaunchConfiguration("indy_type")
    indy_eye = LaunchConfiguration("indy_eye")
    servo_mode = LaunchConfiguration("servo_mode")
    prefix = LaunchConfiguration("prefix")

    indy_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [gazebo_package, "/launch", "/indy_gazebo.launch.py"]
        ),
        launch_arguments={
            "name": name,
            "indy_type": indy_type,
            "indy_eye": indy_eye,
            "prefix": prefix,
            "launch_rviz": "false",
        }.items(),
    )

    indy_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [moveit_config_package, "/launch", "/moveit.launch.py"]
        ),
        launch_arguments={
            "name": name,
            "indy_type": indy_type,
            "indy_eye": indy_eye,
            "servo_mode": servo_mode,
            "prefix": prefix,
            "use_sim_time": "true",
            "launch_rviz_moveit": "true", # if name == "launch_rviz" => spawn 2 rviz
        }.items(),
    )

    nodes_to_launch = [
        indy_gazebo_launch,
        indy_moveit_launch,
    ]

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "name",
            default_value="indy"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "indy_type",
            default_value="indy7",
            description="Type of Indy robot.",
            choices=["indy7", "indy7_v2" , "indy12", "indy12_v2", "indyrp2", "indyrp2_v2", "icon7l", "icon3", "nuri3s", "nuri4s", "nuri7c", "nuri20c", "opti5"]
        )
    )
 
    declared_arguments.append(
        DeclareLaunchArgument(
            "indy_eye",
            default_value="false",
            description="Work with Indy Eye",
        )
    )
 
    declared_arguments.append(
        DeclareLaunchArgument(
            "servo_mode",
            default_value="false",
            description="Servoing mode",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup. \
            If changed than also joint names in the controllers configuration have to be updated."
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])