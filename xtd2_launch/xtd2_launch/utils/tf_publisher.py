#!/usr/bin/env python3

import platform
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose
from nav_msgs.msg import Odometry


class TfPublisher(Node):

    def __init__(self):
        # 检查主机名，设置默认namespace
        hostname = platform.node()
        if hostname == 'ywj-B250-D3A' or hostname == 'DESKTOP-ypat':
            default_namespace = '/x500_depth_0/'
            self.is_sim = True
        else:
            default_namespace = '/'
            self.is_sim = False
        
        super().__init__(
            'tf_publisher',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, self.is_sim),
                Parameter('namespace', Parameter.Type.STRING, default_namespace)
            ]
        )
        
        # 获取namespace参数
        self.declare_parameter('namespace', default_namespace)
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value

        # 动态 TF（odom 真值）
        self.tf_broadcaster = TransformBroadcaster(self)

        # 静态 TF（结构关系）
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # PX4 visual odometry发布器
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # # 补偿后的轨迹发布器
        # self.compensated_traj_pub = self.create_publisher(
        #     Pose,
        #     '/xtdrone2' + self.namespace + 'cmd_pose_local_ned',
        #     50
        # )

        # 订阅 Gazebo 发布的 odometry（真值）
        self.create_subscription(
            Odometry,
            self.namespace + 'odometry',
            self.odom_callback,
            10
        )

        # # 订阅原始轨迹（Gazebo坐标系）
        # self.create_subscription(
        #     PoseStamped,
        #     '/xtdrone2/planning/raw_trajectory',
        #     self.raw_trajectory_callback,
        #     10
        # )

        # 创建相机位姿发布器
        if self.is_sim:
            self.camera_pose_publisher = self.create_publisher(
                PoseStamped,
                self.namespace + 'StereoOV7251/pose',
                10
            ) 
        else:
        # 真机环境下创建mid360/pose发布器
            self.mid360_pose_publisher = self.create_publisher(
                PoseStamped,
                self.namespace + 'mid360/pose',
                10
            )

        # 一次性发布所有静态 TF
        self.publish_static_transforms()

        self.get_logger().info('TF publisher started (static + dynamic TF + PX4 visual odometry)')

    # def raw_trajectory_callback(self, msg: PoseStamped):
    #     """处理原始轨迹（直接发布，不进行坐标转换）"""
    #     try:
    #         # 直接发布原始轨迹（Gazebo坐标系/ENU）
    #         trajectory_pose = Pose()
    #         trajectory_pose.position.x = msg.pose.position.x
    #         trajectory_pose.position.y = msg.pose.position.y
    #         trajectory_pose.position.z = msg.pose.position.z
            
    #         trajectory_pose.orientation.w = msg.pose.orientation.w
    #         trajectory_pose.orientation.x = msg.pose.orientation.x
    #         trajectory_pose.orientation.y = msg.pose.orientation.y
    #         trajectory_pose.orientation.z = msg.pose.orientation.z
            
    #         # 发布轨迹
    #         self.compensated_traj_pub.publish(trajectory_pose)
            
    #         self.get_logger().debug(
    #             f'Trajectory: position=({trajectory_pose.position.x:.3f}, '
    #             f'{trajectory_pose.position.y:.3f}, {trajectory_pose.position.z:.3f})',
    #             throttle_duration_sec=1.0
    #         )
                
    #     except Exception as e:
    #         self.get_logger().error(f'Error processing trajectory: {str(e)}')

    def odom_callback(self, msg: Odometry):
        """Gazebo odometry callback - 发布 world -> base_footprint tf (真值)"""
        # 发布 world -> base_footprint tf (使用Gazebo odometry作为真值)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'world'
        t.child_frame_id = self.namespace.lstrip('/') + 'base_footprint'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)
        
        # 模拟环境下发布相机位姿
        if self.is_sim:
            self.publish_camera_pose(msg)
        else: 
        # 真机环境下发布mid360/pose
            self.publish_mid360_pose(msg)

    def publish_camera_pose(self, odom_msg: Odometry):
        """从odom推算相机位姿并发布"""
        camera_pose = PoseStamped()
        camera_pose.header.stamp = odom_msg.header.stamp
        camera_pose.header.frame_id = self.namespace.lstrip('/') + 'odom'
        
        # 相机相对于base_footprint的固定偏移
        camera_offset_x = 0.12  # 前方偏移
        camera_offset_y = 0.03  # 侧向偏移
        camera_offset_z = 0.242  # 下方偏移
        
        # 计算相机在世界坐标系中的位置
        import math
        
        # 获取无人机的朝向（四元数）
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w
        
        # 简化的旋转计算
        yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
        
        # 旋转相机偏移
        rotated_offset_x = camera_offset_x * math.cos(yaw) - camera_offset_y * math.sin(yaw)
        rotated_offset_y = camera_offset_x * math.sin(yaw) + camera_offset_y * math.cos(yaw)
        rotated_offset_z = camera_offset_z
        
        # 计算相机在世界坐标系中的位置
        camera_pose.pose.position.x = odom_msg.pose.pose.position.x + rotated_offset_x
        camera_pose.pose.position.y = odom_msg.pose.pose.position.y + rotated_offset_y
        camera_pose.pose.position.z = odom_msg.pose.pose.position.z + rotated_offset_z
        
        # 相机朝向与无人机朝向一致
        camera_pose.pose.orientation = odom_msg.pose.pose.orientation
        
        # 发布相机位姿
        self.camera_pose_publisher.publish(camera_pose)
        
        # 发布相机TF
        camera_tf = TransformStamped()
        camera_tf.header.stamp = odom_msg.header.stamp
        camera_tf.header.frame_id = self.namespace.lstrip('/') + 'base_footprint'
        camera_tf.child_frame_id = self.namespace.lstrip('/') + 'OakD-Lite/base_link/StereoOV7251'
        
        camera_tf.transform.translation.x = camera_offset_x
        camera_tf.transform.translation.y = camera_offset_y
        camera_tf.transform.translation.z = camera_offset_z
        
        # 相机相对于 base_footprint 
        camera_tf.transform.rotation.w = 1.0
        camera_tf.transform.rotation.x = 0.0
        camera_tf.transform.rotation.y = 0.0
        camera_tf.transform.rotation.z = 0.0
        
        self.tf_broadcaster.sendTransform(camera_tf)

    def publish_mid360_pose(self, odom_msg: Odometry):
        """发布mid360/pose话题"""
        mid360_pose = PoseStamped()
        mid360_pose.header.stamp = odom_msg.header.stamp
        mid360_pose.header.frame_id = 'livox_frame'
        
        # mid360相对于base_link的固定偏移
        mid360_offset_x = -0.1  # 后方偏移
        mid360_offset_y = 0.0  # 侧向偏移
        mid360_offset_z = -0.1  # 下方偏移
        
        # 计算mid360在世界坐标系中的位置
        import math
        
        # 获取无人机的朝向（四元数）
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w
        
        # 简化的旋转计算
        yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
        
        # 旋转mid360偏移
        rotated_offset_x = mid360_offset_x * math.cos(yaw) - mid360_offset_y * math.sin(yaw)
        rotated_offset_y = mid360_offset_x * math.sin(yaw) + mid360_offset_y * math.cos(yaw)
        rotated_offset_z = mid360_offset_z
        
        # 计算mid360在世界坐标系中的位置
        mid360_pose.pose.position.x = odom_msg.pose.pose.position.x + rotated_offset_x
        mid360_pose.pose.position.y = odom_msg.pose.pose.position.y + rotated_offset_y
        mid360_pose.pose.position.z = odom_msg.pose.pose.position.z + rotated_offset_z
        
        # mid360朝向与无人机朝向一致
        mid360_pose.pose.orientation = odom_msg.pose.pose.orientation
        
        # 发布mid360位姿
        self.mid360_pose_publisher.publish(mid360_pose)

    def publish_static_transforms(self):
        """发布静态TF变换"""
        now = self.get_clock().now().to_msg() 
        while now.sec == 0:  # 等待 /clock 开始发布
            rclpy.spin_once(self, timeout_sec=1.0)
            now = self.get_clock().now().to_msg()
            self.get_logger().info(f'now:{now}')

        tfs = []

        # world -> map
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'world'
        t.child_frame_id = 'map'
        t.transform.rotation.w = 1.0
        tfs.append(t)

        # map -> odom
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        tfs.append(t)

        if self.namespace != '/':
            # odom -> namespace/odom
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = self.namespace.lstrip('/') + 'odom'
            t.transform.rotation.w = 1.0
            tfs.append(t)

        # base_footprint -> base_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.namespace.lstrip('/') + 'base_footprint'
        t.child_frame_id = self.namespace.lstrip('/') + 'base_link'
        t.transform.rotation.w = 1.0
        tfs.append(t)

        

        if self.is_sim:
            # base_link -> OakD-Lite base
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.namespace.lstrip('/') + 'base_link'
            t.child_frame_id = self.namespace.lstrip('/') + 'OakD-Lite/base_link'
            t.transform.translation.x = 0.12
            t.transform.translation.y = 0.03
            t.transform.translation.z = 0.242
            t.transform.rotation.w = 1.0
            tfs.append(t)
    
            # OakD-Lite -> StereoOV7251（点云）
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.namespace.lstrip('/') + 'OakD-Lite/base_link'
            t.child_frame_id = self.namespace.lstrip('/') + 'OakD-Lite/base_link/StereoOV7251'
            t.transform.translation.x = 0.01233
            t.transform.translation.y = -0.03
            t.transform.translation.z = 0.01878
            t.transform.rotation.x = 0.707
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.707
            t.transform.rotation.w = 0.0
            tfs.append(t)
    
            # OakD-Lite -> IMX214（RGB）
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.namespace.lstrip('/') + 'OakD-Lite/base_link'
            t.child_frame_id = self.namespace.lstrip('/') + 'IMX214'
            t.transform.translation.x = 0.01233
            t.transform.translation.y = -0.03
            t.transform.translation.z = 0.01878
            t.transform.rotation.w = 1.0
            tfs.append(t)
    
            # StereoOV7251 -> StereoOV7251
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.namespace.lstrip('/') + 'OakD-Lite/base_link/StereoOV7251'
            t.child_frame_id = self.namespace.lstrip('/') + 'StereoOV7251'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.w = 1.0
            tfs.append(t)

        # 真机环境下添加mid360到base_link的TF变换
        else:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'livox_frame'
            t.child_frame_id = self.namespace.lstrip('/') + 'base_link'
            t.transform.translation.x = -0.1
            t.transform.translation.y = 0.0
            t.transform.translation.z = -0.1
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = -0.87266
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            tfs.append(t)

        self.static_tf_broadcaster.sendTransform(tfs)

        self.get_logger().info('Static TFs published')


def main(args=None):
    rclpy.init(args=args)
    node = TfPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()