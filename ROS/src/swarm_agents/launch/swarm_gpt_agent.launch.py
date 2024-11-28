import os

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration

    
def generate_launch_description():
    
    
    param_agent_dir = os.path.join(get_package_share_directory('swarm_agents'),  'config', 'chatgpt.yaml')
    #param_detect_dir = os.path.join(get_package_share_directory('loop'),  'config', 'euroc/vins_config.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory('swarm_agents'), 'config/rviz', 'swarm_agent_config.rviz')
    #print(param_dir)

    


    
    show_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rvizvisualisation',
        output='log',
        arguments=['-d', rviz_config_dir]
        )

    swarm = Node(
            namespace=[LaunchConfiguration('namespace')],
            package='swarm_agents',
            executable='swarm',
            name='swarm_agent',            
            output='screen',
            emulate_tty=True,
            #prefix=['xterm -e gdb -ex run --args'],
            prefix=['stdbuf -o L'],
            
            parameters=[
                param_agent_dir,
              
                {"image_hz": 30.},
                {"agent_id": [LaunchConfiguration('agentid')]},
                {"max_nb_agents": 5}
            ]
        )
    
    

   
   
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/r0', description='Namespace'),

        DeclareLaunchArgument('agentid', default_value='0', description='robot id'),
        
        #loop 
        
        #,
        swarm
        #,              
        #show_rviz       
    ])