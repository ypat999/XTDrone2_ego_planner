from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable,
                            IncludeLaunchDescription, SetLaunchConfiguration, LogInfo,
                            ExecuteProcess)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():

    os.environ['GAZEBO_MODEL_DATABASE_URI'] = ""
    # 添加GPU渲染优化环境变量
    mesa_adapter = SetEnvironmentVariable(
        name='MESA_D3D12_DEFAULT_ADAPTER_NAME',
        value='NVIDIA'
    )
    
    gazebo_gpu_rendering = SetEnvironmentVariable(
        name='GAZEBO_GPU_RENDERING',
        value='1'
    )



    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_xtdrone2_gz_sim = get_package_share_directory('xtd2_gz_sim')
    # gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gz_model_path = '/home/ywj/git/xtd2_ws/XTDrone2_ego_planner/xtd2_gz_sim/models'  #PathJoinSubstitution([pkg_xtdrone2_gz_sim, 'models'])
    gz_model_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '') + ':' + gz_model_path
    
    return LaunchDescription([
        mesa_adapter,
        gazebo_gpu_rendering,
        DeclareLaunchArgument(
            'world',
            # default_value='aruco',
            default_value='tugbot_warehouse',
            description='World to load into Gazebo'
        ),
        SetLaunchConfiguration(name='world_file', 
                               value=[LaunchConfiguration('world'), 
                                      TextSubstitution(text='.sdf')]),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_model_path),
        SetEnvironmentVariable('DISPLAY', ':0'),
        LogInfo(msg=[TextSubstitution(text='Gazebo simulation resources path: '), gz_model_path]),
        ExecuteProcess(
            cmd=['gz', 'sim' , '--force-version', '7', '-r', PathJoinSubstitution([pkg_xtdrone2_gz_sim, 'worlds', LaunchConfiguration('world_file')])],
        ),
    ])