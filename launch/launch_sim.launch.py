import os
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

from scripts import GazeboRosPaths

from launch_ros.actions import Node



def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('omni_sim'))
    urdf_file = os.path.join(pkg_path,'description/urdf','omni_sim.urdf')
    
    yaml_controller = os.path.join(pkg_path,'config','controller.yaml')

    config = os.path.join(
        get_package_share_directory('omni_sim'),
        'config',
        'params.yaml'
        )
    
    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()

    env = {
        "GAZEBO_MODEL_PATH": model_path,
        "GAZEBO_PLUGIN_PATH": plugin_path,
        "GAZEBO_RESOURCE_PATH": media_path,
    }

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='omni_sim' #<--- CHANGE ME


    #TEST FOR OMNIWHEEL#########################################
    forward_velocity_controller = Node(
            package="controller_manager",
            executable="spawner",   
            arguments=["forward_velocity_controller", "-c", "/controller_manager"],
            )
    joint_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",   
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            )

    # forward_velocity_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'forward_velocity_controller'], output='screen'
    # )

    # joint_state_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'], output='screen'
    # )
    ############################################################



    descripition = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','description_launch.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
    
    jstate = Node(package='joint_state_publisher', executable='joint_state_publisher',
                        output='screen')

    control = Node(
            package='omni_sim',
            executable='control_omni_sim.py',
            name='omni_sim_control',
            parameters = [config]
        )
    
    forward_position_controller = Node(package="controller_manager",
        executable="ros2_control_node",
        parameters=[urdf_file, yaml_controller]
    )
    



    # Launch them all!
    return LaunchDescription([
        RegisterEventHandler(
          event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[forward_velocity_controller],
            )
        ),

        descripition,
        gazebo,
        spawn_entity,
        jstate,
        control,
        #forward_position_controller,
        # forward_velocity_controller,
        # joint_state_broadcaster,
    ])
