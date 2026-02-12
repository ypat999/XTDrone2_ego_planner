import os
import yaml
import argparse
import subprocess

from string import Template

from launch_ros.substitutions import FindPackageShare


def main():
    parser = argparse.ArgumentParser(description='Launch Gazebo ROS bridge in XTDrone2')

    parser.add_argument('--ros_ns', type=str, help='ROS namespace', required=True)
    parser.add_argument('--gz_ns', type=str, help='Gazebo namespace', required=True)
    parser.add_argument('--worldname', type=str, help='World name', required=True)

    args, unknown = parser.parse_known_args()

    # 通过YAML模板设定每个vehicle的桥接设置
    yaml_template = os.path.join(FindPackageShare("xtd2_launch").find('xtd2_launch'), 'launch', 'launch_config', 'ros_gz_bridge.yaml')
    # 处理ros_ns中的斜杠，确保生成有效的文件名
    safe_ros_ns = args.ros_ns.strip('/')
    yaml_tmp_output = os.path.join(FindPackageShare("xtd2_launch").find('xtd2_launch'), 'launch', 'launch_config', f'tmp_ros_gz_bridge_{safe_ros_ns}.yaml')
    with open(yaml_template, 'r') as file:
        yaml_template_content = file.read()
    template = Template(yaml_template_content).safe_substitute({'gz_ns': safe_ros_ns, 'ros_ns': safe_ros_ns, 'worldname': args.worldname})
    with open(yaml_tmp_output, 'w') as file:
        yaml.safe_dump(yaml.safe_load(template), file)


    # 使用ros_gz_bridge节点启动桥接
    pose_bridge_cmd = f'ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:={yaml_tmp_output}'

    _handle = subprocess.Popen(['bash', '-c', pose_bridge_cmd])

    try:
        while True:
            if _handle.poll() is not None:
                break
            else:
                _handle.wait()
    except KeyboardInterrupt:
        _handle.terminate()
        exit(0)



if __name__ == '__main__':
    main()