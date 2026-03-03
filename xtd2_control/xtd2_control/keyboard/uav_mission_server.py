import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, VehicleLocalPosition
from rcl_interfaces.msg import SetParametersResult
import math


class UAVMissionServer(Node):
    """
    无人机自主飞行任务服务器节点
    使用话题而非服务来控制无人机
    """

    def __init__(self):
        super().__init__('uav_mission_server')

        # 创建兼容PX4的QoS配置
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 创建话题订阅器，接收命令
        self.command_sub = self.create_subscription(
            String,
            'uav_command',
            self.command_callback,
            10
        )

        # 创建发布者
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # 创建订阅者，使用与PX4兼容的QoS
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )

        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile
        )

        # 初始化参数
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()
        
        # 设置定时器，每0.1秒触发一次回调函数
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 初始位置和目标位置
        self.initial_x = None
        self.initial_y = None
        self.target_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # 状态管理
        self.current_state = 'IDLE'  # 可能的状态: IDLE, ARMING, TAKING_OFF, FLYING_TO_POINT, HOVERING, LANDING, DISARMING
        self.flying_state = 'TAKEOFF'  # 飞行阶段: TAKEOFF, CRUISE, RETURN
        self.setpoint_send_count = 0
        self.is_flying = False
        self.takeoff_altitude = 1.0  # 默认起飞高度1米，可以通过命令修改

        self.get_logger().info(f'无人机自主飞行任务服务器已启动，current_state: {self.current_state}')

    def command_callback(self, msg):
        """处理无人机任务命令"""
        command = msg.data
        self.get_logger().info(f'收到命令: {command}')
        
        if command == 'TAKEOFF':
            self.get_logger().info('开始执行起飞任务，默认高度1米')
            self.current_state = 'ARMING'
            self.flying_state = 'TAKEOFF'
        elif command.startswith('TAKEOFF:'):
            # 解析高度参数，格式为"TAKEOFF:height"
            try:
                height_str = command.split(':')[1]
                height = float(height_str)
                self.takeoff_altitude = height
                self.get_logger().info(f'开始执行起飞任务，高度设置为: {height}米')
                self.current_state = 'ARMING'
                self.flying_state = 'TAKEOFF'
            except ValueError:
                self.get_logger().error(f'无法解析高度参数: {command}')
                self.get_logger().info('使用默认高度1米执行起飞任务')
                self.current_state = 'ARMING'
                self.flying_state = 'TAKEOFF'
        elif command == 'LAND':
            self.get_logger().info('开始执行降落任务')
            self.land()
            self.current_state = 'LANDING'
            self.flying_state = 'RETURN'
        elif command.startswith('GOTO:'):
            # 解析坐标，格式为"GOTO:x,y,z"
            try:
                coords_str = command.split(':')[1]
                x, y, z = map(float, coords_str.split(','))
                self.get_logger().info(f'开始执行巡航任务，目标点: ({x}, {y}, {z})')
                self.target_position['x'] = x
                self.target_position['y'] = y
                self.target_position['z'] = -abs(z)  # 转换为NED坐标系
                self.current_state = 'FLYING_TO_POINT'
                self.flying_state = 'CRUISE'
            except ValueError:
                self.get_logger().error(f'无法解析坐标: {command}')
        else:
            self.get_logger().warn(f'未知命令: {command}')

    def vehicle_status_callback(self, msg):
        """车辆状态回调函数"""
        self.vehicle_status = msg

    def vehicle_local_position_callback(self, msg):
        """车辆本地位置回调函数"""
        self.vehicle_local_position = msg

    def timer_callback(self):
        """定时器回调函数，控制无人机状态转换"""
        # 发布offboard控制模式信号 - 必须持续发送
        self.publish_offboard_control_heartbeat_signal()

        # 根据当前状态发布轨迹设定点
        if self.current_state == 'ARMING':
            self.handle_arming_state()
        elif self.current_state == 'TAKING_OFF':
            self.handle_taking_off_state()
        elif self.current_state == 'FLYING_TO_POINT':
            self.handle_flying_to_point_state()
        elif self.current_state == 'HOVERING':
            self.handle_hovering_state()
        elif self.current_state == 'LANDING':
            self.handle_landing_state()
        elif self.current_state == 'DISARMING':
            self.handle_disarming_state()
        else:
            # IDLE状态或其他状态，发布安全设定点
            self.publish_setpoint(x=float('nan'), y=float('nan'), z=float('nan'))

        self.setpoint_send_count += 1

    def handle_arming_state(self):
        """处理解锁状态"""
        # 记录初始位置
        if self.initial_x is None or self.initial_y is None:
            self.initial_x = self.vehicle_local_position.x
            self.initial_y = self.vehicle_local_position.y
            self.get_logger().info(f'记录初始位置: X={self.initial_x:.2f}, Y={self.initial_y:.2f}')

        # 启用offboard模式
        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.engage_offboard_mode()

        # 持续发送解锁命令直到成功
        if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info('无人机已解锁，切换到起飞状态')
            self.current_state = 'TAKING_OFF'
        else:
            self.arm()

        # 发布基础设定点
        self.publish_setpoint(x=self.initial_x, y=self.initial_y, z=-0.1)

    def  handle_taking_off_state(self):
        """处理起飞状态"""
        takeoff_altitude = self.takeoff_altitude  # 起飞高度
        
        # 发布起飞设定点
        self.publish_setpoint(x=self.initial_x, y=self.initial_y, z=-takeoff_altitude)

        # 检查是否达到目标高度
        if self.vehicle_local_position.z_valid:
            current_altitude = abs(self.vehicle_local_position.z)
            target_altitude = takeoff_altitude * 0.9  # 达到90%的目标高度

            self.get_logger().info(f'当前X: {self.vehicle_local_position.x:.2f}, 当前Y: {self.vehicle_local_position.y:.2f}, 当前高度: {current_altitude:.2f}, 目标: {target_altitude:.2f}')

            if current_altitude >= target_altitude:
                self.get_logger().info(f'达到目标高度 {takeoff_altitude}m')
                
                # 如果是巡航请求，则继续前往目标点
                if self.flying_state == 'CRUISE':
                    self.current_state = 'FLYING_TO_POINT'
                # 如果是返航请求，则进入悬停状态
                elif self.flying_state == 'RETURN':
                    self.current_state = 'HOVERING'
                else:
                    self.current_state = 'HOVERING'

    def handle_flying_to_point_state(self):
        """处理飞向目标点状态"""
        # 发布目标点设定点
        self.publish_setpoint(
            x=self.target_position['x'], 
            y=self.target_position['y'], 
            z=self.target_position['z']
        )

        # 检查是否到达目标点（距离小于0.1米）
        distance_to_target = math.sqrt(
            (self.vehicle_local_position.x - self.target_position['x'])**2 +
            (self.vehicle_local_position.y - self.target_position['y'])**2 +
            (self.vehicle_local_position.z - self.target_position['z'])**2
        )
        
        # 打印目标点位置信息和到目标点的距离
        self.get_logger().info(f'目标点位置: ({self.target_position["x"]:.2f}, {self.target_position["y"]:.2f}, {self.target_position["z"]:.2f}), \
                               当前点位置: ({self.vehicle_local_position.x:.2f}, {self.vehicle_local_position.y:.2f}, {self.vehicle_local_position.z:.2f}), \
                               距离: {distance_to_target:.2f}m')

        if distance_to_target < 0.5:  # 距离小于0.5米认为到达
            self.get_logger().info(f'已到达目标点: ({self.target_position["x"]:.2f}, {self.target_position["y"]:.2f}, {self.target_position["z"]:.2f})')
            self.current_state = 'HOVERING'

    def handle_hovering_state(self):
        """处理悬停状态"""
        # 在目标点悬停
        if self.current_state == 'FLYING_TO_POINT':
            hover_pos = self.target_position
        else:
            # 在当前位置悬停
            hover_pos = {
                'x': self.vehicle_local_position.x,
                'y': self.vehicle_local_position.y,
                'z': self.vehicle_local_position.z
            }

        self.publish_setpoint(
            x=hover_pos['x'],
            y=hover_pos['y'],
            z=hover_pos['z']
        )

    def handle_landing_state(self):
        """处理降落状态"""
        # 发布降设定点（在当前位置缓慢下降）
        self.publish_setpoint(
            x=self.vehicle_local_position.x,
            y=self.vehicle_local_position.y,
            z=0.0  # 降落到地面
        )
        self.get_logger().info(f'执行降落任务，当前点位置: ({self.vehicle_local_position.x:.2f}, {self.vehicle_local_position.y:.2f}, {self.vehicle_local_position.z:.2f})')
        # 检查是否已降落到地面
        if self.vehicle_local_position.z_valid:
            current_altitude = abs(self.vehicle_local_position.z)

            if current_altitude <= 0.1:  # 高度小于0.1米认为已着陆
                self.get_logger().info('已着陆')
                self.current_state = 'DISARMING'

    def handle_disarming_state(self):
        """处理上锁状态"""
        if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            self.get_logger().info('无人机已上锁，任务完成')
            self.current_state = 'IDLE'
        else:
            # 持续发送上锁命令直到成功
            self.disarm()

    def arm(self):
        """发送命令使无人机解锁并准备飞行"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0
        )
        self.get_logger().info("发送解锁命令")

    def disarm(self):
        """发送命令使无人机上锁"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0
        )
        self.get_logger().info("发送上锁命令")
    def land(self):
        """发送降落命令"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )
        self.get_logger().info("发送降落命令:Land command sent")
    def engage_offboard_mode(self):
        """启用offboard模式"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,  # 主模式
            6.0  # OFFBOARD模式
        )
        self.get_logger().info("启用offboard模式")

    def publish_offboard_control_heartbeat_signal(self):
        """发布offboard控制模式信号"""
        msg = OffboardControlMode()
        msg.position = True  # 启用位置控制
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # 时间戳（微秒）
        self.offboard_control_mode_publisher.publish(msg)

    def publish_setpoint(self, x=0.0, y=0.0, z=None):
        """发布轨迹设定点"""
        if z is None:
            z = -2.5  # 默认高度2.5米

        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [float('nan'), float('nan'), float('nan')]  # 使用NaN表示不控制速度
        msg.acceleration = [float('nan'), float('nan'), float('nan')]  # 使用NaN表示不控制加速度
        msg.jerk = [float('nan'), float('nan'), float('nan')]  # 使用NaN表示不控制急动度
        msg.yaw = float('nan')  # 不控制偏航角
        msg.yawspeed = float('nan')  # 不控制偏航速率
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # 时间戳（微秒）
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """发布车辆命令"""
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # 时间戳（微秒）
        self.vehicle_command_publisher.publish(msg)


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    # 创建节点实例
    uav_mission_server = UAVMissionServer()

    try:
        # 开始处理回调
        rclpy.spin(uav_mission_server)
    except KeyboardInterrupt:
        uav_mission_server.get_logger().info('被用户中断')
    finally:
        # 清理资源
        uav_mission_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()