import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
import launch_ros.descriptions
import launch


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf = os.path.join(
        get_package_share_directory('softleg_description'),
        'urdf',
        # 'softleg.urdf'
        'softlegisaacFC.urdf'
    )

    print("urdf_file_name : {}".format(urdf))

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
            ),
            launch_arguments={"pause": "false", "verbose": "false"}.items(),
    )

    default_dof = (
            -0.75,     
            -0.75,
            )
    joint_names = (
        'softleg_1_hip_joint',   
        'softleg_1_knee_joint',
    )
    default_dict = dict(zip(joint_names, default_dof))

    #format is "NAME:=VALUE "
    default_joint_args = ""
    for key, value in default_dict.items():
        default_joint_args += key + ":=" + str(value) + " "
        # print(default_joint_args)

    # Get URDF via xacro
    # softleg_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="urdf")]),
    #         " ", urdf, " use_gazebo:=true ",
    #         default_joint_args
    #     ]
    # )
        
    softleg_description_content = open(os.path.join(get_package_share_directory('softleg_description'),
                                                'urdf',
                                                # 'softleg.urdf'
                                                'softlegisaacFC.urdf'
                                                )).read()
    
    robot_description = {"robot_description": softleg_description_content}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "softleg",
                    "-x", "0.0", "-y", "0.0", "-z", "0.0",
                    ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    PD_jnt_control = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],  #CAMBIATO position invece di PD
    )
    
    jumping_node=Node(
                package = 'jumping_experiments',
                name = 'jumping_node',
                executable = 'jumping_node',
                parameters = [{'publication_rate': 200},
                        {'duration': 5.0},
                        {'start_delay': 1.0}]
        )
    
    policy = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        [PathJoinSubstitution([FindPackageShare("rlg_quad_controller"), "launch", "softleg_simulation.launch.py"])]
                ),
        )

    return LaunchDescription([
        
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),

        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value='true',
        #     description='Use simulation (Gazebo) clock if true'),

        gazebo,

        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[PD_jnt_control],
            )
        ),
        #jumping_node,
        #policy,
  ])