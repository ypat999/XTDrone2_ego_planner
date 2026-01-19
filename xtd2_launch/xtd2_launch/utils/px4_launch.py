import argparse
import subprocess
import sys
import logging

# Configure logging to stdout so ros2 launch can capture it reliably
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(name)s: %(message)s', stream=sys.stdout)
logger = logging.getLogger('px4_launch')

def main():
    parser = argparse.ArgumentParser(description='Launch px4 sitl in XTDrone2')

    parser.add_argument('--model', type=str, help='Model name', required=True)
    parser.add_argument('--id', type=int, help='Vehicle id', required=True)
    parser.add_argument('--x', type=float, help='X position', required=False, default=0)
    parser.add_argument('--y', type=float, help='Y position', required=False, default=0)
    parser.add_argument('--z', type=float, help='Z position', required=False, default=0)
    parser.add_argument('--roll', type=float, help='Roll', required=False, default=0)
    parser.add_argument('--pitch', type=float, help='Pitch', required=False, default=0)
    parser.add_argument('--yaw', type=float, help='Yaw', required=False, default=0)
    parser.add_argument('--world', type=str, help='World name', required=False, default="default")
    parser.add_argument('--px4-dir', type=str, help='PX4 directory', required=False, default="~/git/PX4-Autopilot")
    parser.add_argument('--namespace', type=str, help='ROS namespace, {{model}}_{{id}} by default', required=False, default="")

    args, unknown = parser.parse_known_args()

    autostart_map = {"gz_x500": 4001, "gz_x500_depth":4002}
    sys_autostart = autostart_map[args.model]

    mav_sys_id = args.id + 1

    if args.namespace:
        ns = args.namespace
    else:
        if args.model.startswith("gz_"):
            ns = f"{args.model[3:]}_{args.id}"
        else:
            ns = f"{args.model}_{args.id}"


    # Extract base model name (remove 'gz_' prefix if present)
    if args.model.startswith("gz_"):
        sim_model = args.model[3:]  # Remove 'gz_' prefix
    else:
        sim_model = args.model
    
    # Get XTDrone2 Gazebo simulation resource path
    import subprocess
    result = subprocess.run(['ros2', 'pkg', 'prefix', 'xtd2_gz_sim'], capture_output=True, text=True)
    if result.returncode == 0:
        xtd2_gz_sim_path = result.stdout.strip()
        xtd2_gz_models_path = f"{xtd2_gz_sim_path}/share/xtd2_gz_sim/models"
    else:
        # Fallback path
        xtd2_gz_models_path = "/home/ywj/git/xtd2_ws/install/xtd2_gz_sim/share/xtd2_gz_sim/models"
    
    # Set XTDrone2 Gazebo model variables similar to PX4 style
    px4_gz_models_path = f"{args.px4_dir}/Tools/simulation/gz/models"
    px4_gz_worlds_path = f"{args.px4_dir}/Tools/simulation/gz/worlds"
    gz_resource_path = f"$GZ_SIM_RESOURCE_PATH:$PX4_GZ_MODELS:$PX4_GZ_WORLDS:{xtd2_gz_models_path}"
    
    px4_cmd = f"PX4_UXRCE_DDS_NS={ns} PX4_GZ_WORLD={args.world} PX4_SYS_AUTOSTART={sys_autostart} PX4_SIM_MODEL={sim_model} PX4_GZ_MODEL_POSE='{args.x},{args.y},{args.z},{args.roll},{args.pitch},{args.yaw}' PX4_GZ_MODELS={px4_gz_models_path} PX4_GZ_WORLDS={px4_gz_worlds_path} XTD2_GZ_MODELS={xtd2_gz_models_path} GZ_SIM_RESOURCE_PATH={gz_resource_path} {args.px4_dir}/build/px4_sitl_default/bin/px4 -d -s {args.px4_dir}/build/px4_sitl_default/etc/init.d-posix/rcS {args.px4_dir}/ROMFS/px4fmu_common -i {args.id} -w {args.px4_dir}/build/px4_sitl_default"
    # px4_cmd = f"PX4_UXRCE_DDS_NS={ns} PX4_GZ_WORLD={args.world} PX4_SYS_AUTOSTART={sys_autostart} PX4_SIM_MODEL={sim_model} PX4_GZ_MODEL_POSE='{args.x},{args.y},{args.z},{args.roll},{args.pitch},{args.yaw}' {args.px4_dir}/build/px4_sitl_default/bin/px4 -d -s {args.px4_dir}/build/px4_sitl_default/etc/init.d-posix/rcS {args.px4_dir}/ROMFS/px4fmu_common -i {args.id} -w {args.px4_dir}/build/px4_sitl_default"

    logger.info("Launching PX4 with command: %s", px4_cmd)
    _handle = subprocess.Popen(['bash', '-c', px4_cmd])

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