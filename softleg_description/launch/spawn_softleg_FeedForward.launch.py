import imp
import os
from ament_index_python.packages import get_package_share_path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command,LaunchConfiguration,PathJoinSubstitution
from launch.conditions import IfCondition,UnlessCondition
from launch.actions import OpaqueFunction
from launch import LaunchDescription, LaunchContext

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def create_yaml(context: LaunchContext, robot_name,joystic_teleop):  #non serve tutto, ma parzialmente. Io uso unico robot
    robot_name_str = context.perform_substitution(robot_name)
    if(robot_name_str != ""):  
        path = os.path.join(get_package_share_path("softleg_description"),"config", "softleg_gazebo_FeedForwardPos.yaml")
        with open(path) as stream:
            param_str = stream.read()
            print(type(param_str))
            subs_par = param_str.replace("tf_prefix",robot_name_str)
        
        file = yaml.safe_load(subs_par)
        file_name = path = os.path.join(get_package_share_path("softleg_description"),"config", f"{robot_name_str}_controllers.yaml")
        with open(file_name,"w") as stream:
            yaml.dump(file, stream, default_flow_style=False)
     
    # new_file = {robot_name: file[] }


        teleop_node = Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            condition=IfCondition(joystic_teleop),
            parameters=[os.path.join(get_package_share_path("softleg_description"),"config", "teleop_conf.yaml")],
            remappings=[
                    ('/cmd_vel', "/"+ robot_name_str+"/diff_drive_control/cmd_vel")]
        )
        bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                [robot_name_str+"/", "imu@sensor_msgs/msg/Imu[ignition.msgs.IMU"],
                ["/gino","height_measure@nav_msgs/msg/Odometry@ignition.msgs.Odometry"],
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                [robot_name_str+"/","rgl_lidar@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked"],
                # "/rgbd_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
                # "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
                # "/rgbd_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image",
                # "/rgbd_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked"
            ],
            output="screen",
            )
    else:
        teleop_node = Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            condition=IfCondition(joystic_teleop),
            parameters=[os.path.join(get_package_share_path("softleg_description"),"config", "teleop_conf.yaml")],
            remappings=[
                    ('/cmd_vel', "/diff_drive_control/cmd_vel")]
        )
        bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                ["imu@sensor_msgs/msg/Imu[ignition.msgs.IMU"],
                # [robot_name,"gt_odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry"],
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                ["rgl_lidar@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked"],
                # "/rgbd_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
                # "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
                # "/rgbd_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image",
                # "/rgbd_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked"
            ],
            output="screen",
            )
    if(robot_name_str != ""):
        joint_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", f"/{robot_name_str}/controller_manager"],
            )  
        position_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["position_controller", "--controller-manager",f"/{robot_name_str}/controller_manager"],
            ) 
    else:
        joint_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            )  
        position_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["position_controller", "--controller-manager", "/controller_manager"],
            )  
    return [teleop_node,bridge,joint_state_broadcaster,position_controller]


def generate_launch_description():  #here delete opawe_fun 200

    ld = LaunchDescription()
    
    log_level = LaunchConfiguration("log_level")
    robot_name = LaunchConfiguration("robot_name")
    spawn_pos_x = LaunchConfiguration("spawn_pos_x")
    spawn_pos_y = LaunchConfiguration("spawn_pos_y")
    spawn_pos_z = LaunchConfiguration("spawn_pos_z")
    joystic_teleop = LaunchConfiguration("joystic_teleop")

    #MODIFIED
    softleg_robot_path = get_package_share_path("softleg_description")
    softleg_robot_path = os.path.join(softleg_robot_path, "urdf", "softleg_FC_fixedcart.urdf")
    print(softleg_robot_path)
    
    #MODIFIED
    softleg_model = DeclareLaunchArgument(
        name="softleg",
        default_value=str(softleg_robot_path)
    )
    
    log_lev_arg = DeclareLaunchArgument(
                name="log_level",
                default_value="warn",
                description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
            )

    robot_name_arg = DeclareLaunchArgument(
                name="robot_name",
                default_value="",
                description="Robot Name, Usefull to spawn different robots in the same simulation",
            )
    spawn_pos_x_arg = DeclareLaunchArgument(
            name="spawn_pos_x",
            default_value="0.",
            description="spawn x position",
        )
    spawn_pos_y_arg = DeclareLaunchArgument(
            name="spawn_pos_y",
            default_value="0.0",
            description="spawn y position",
        )
    spawn_pos_z_arg = DeclareLaunchArgument(
            name="spawn_pos_z",
            default_value="0.015",
            description="spawn z position",
        )
    joystic_teleop_arg = DeclareLaunchArgument(
            name="joystic_teleop",
            default_value="true",
            description="use joystic to teleoperate the robot ",
        )
    ld.add_action(softleg_model)
    ld.add_action(log_lev_arg)
    ld.add_action(robot_name_arg)
    ld.add_action(spawn_pos_x_arg)
    ld.add_action(spawn_pos_y_arg)
    ld.add_action(spawn_pos_z_arg)
    ld.add_action(joystic_teleop_arg)

    #MOFIFIED
    robot_description = ParameterValue(
        Command(["xacro ",LaunchConfiguration("softleg")]),
        value_type=str
    )

    #node declaration
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )
    ld.add_action(robot_state_pub)

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            robot_name,
            "-topic",
            "robot_description",
            "-z",
            spawn_pos_z,
            "-x",
            spawn_pos_x,
            "-y",
            spawn_pos_y,
            "--ros-args",
            "--log-level",
            log_level,
        ],
        parameters=[{"use_sim_time": True}],
    )
    ld.add_action(spawn_entity)


    joy_node = Node(
        package="joy",
        executable="joy_node",
        condition=IfCondition(joystic_teleop)
    )

    ld.add_action(joy_node)
    bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "height_measure@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                # "/rgbd_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
                # "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
                # "/rgbd_camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image",
                # "/rgbd_camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked"
            ],
            output="screen",
            )
    ld.add_action(bridge)
    
    # Add controller spawners with proper timing
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )
    
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(position_controller_spawner)
    
    # teleop_node = Node(
    #     package="teleop_twist_joy",
    #     executable="teleop_node",
    #     condition=IfCondition(joystic_teleop),
    #     parameters=[os.path.join(get_package_share_path("softleg_description"),"config", "teleop_conf.yaml")],
    #     remappings=[
    #             ('/cmd_vel', "/"+ +"/ackerman_control/reference")]
    # )
    # ld.add_action(teleop_node)
    
    
    
    return ld