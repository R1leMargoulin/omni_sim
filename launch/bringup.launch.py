import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        
    config = os.path.join(
        get_package_share_directory('omni_sim'),
        'config',
        'params.yaml'
        )

    return LaunchDescription([
        Node(
            package='omni_sim',
            executable='control_omni_sim.py',
            name='omni_sim_control',
            parameters = [config]
        ),
        Node(
            package='omni_sim',
            executable='motors_manager_node.py',
            name='motors_control',
            parameters = [config],
        ),
        Node(
            package='omni_sim',
            executable='imu_node.py',
            name='imu_node',
            prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
            shell=True,
        )
    ])
