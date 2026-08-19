#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 mid360 点云转换为 PX4 ObstacleDistance 消息，供 PX4 原生 Collision Prevention 使用。

数据流:
    PointCloud2 (mid360) -> 机体系(FRD) -> 水平方位角分 bin -> px4_msgs/ObstacleDistance
    -> {namespace}fmu/in/obstacle_distance (uXRCE-DDS) -> PX4 uORB obstacle_distance

约定(与 PX4 CollisionPrevention 匹配):
    - frame = MAV_FRAME_BODY_FRD (12), 0 号 bin 指向机头, 角度顺时针(右)为正
    - distances 单位为 cm
    - 某 bin 内有点: 取该方位最小水平距离 (cm)
    - 某 bin 无回波: 填 max_distance+1, 表示该方向明确无障碍
    - 数据超时(>500ms)由 PX4 侧判定为 stale, 配合 CP_GO_NO_DATA=1 允许无数据时飞行
"""

import time as time_module

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration

from tf2_ros import Buffer, TransformListener

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from px4_msgs.msg import ObstacleDistance

UINT16_MAX = 65535
CLOUD_TOPIC_NAME = '/livox/lidar'


def quat_to_rotation_matrix(w, x, y, z):
    """单位四元数 (w,x,y,z) 转为旋转矩阵 (3x3)。"""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def pointcloud2_to_xyz(msg):
    """从 PointCloud2 提取 (N,3) 的 xyz 点(米)。返回 numpy float32 数组或 None。"""
    n_points = msg.width * msg.height
    if n_points == 0 or len(msg.fields) < 3:
        return None

    # 查找 x/y/z 字段偏移
    offsets = {}
    for f in msg.fields:
        offsets[f.name] = (f.offset, f.datatype)
    if not all(name in offsets for name in ('x', 'y', 'z')):
        return None

    step = msg.point_step
    if step <= 0:
        return None

    # 限制单帧处理点数, 防止超大点云拖慢节点
    n_read = min(int(n_points), 300000)
    raw = np.frombuffer(msg.data, dtype=np.uint8, count=n_read * step)
    raw = raw.reshape(n_read, step)

    pts = np.zeros((n_read, 3), dtype=np.float32)
    for i, name in enumerate(('x', 'y', 'z')):
        off, dt = offsets[name]
        if dt == PointField.FLOAT32 and off + 4 <= step:
            # copy 保证对齐后再 view
            col = raw[:, off:off + 4].copy().view(np.float32).reshape(-1)
            pts[:, i] = col
    return pts


class ObstacleDistancePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_distance_publisher')

        self.declare_parameter('cloud_topic', CLOUD_TOPIC_NAME)
        self.declare_parameter('namespace', '/x500_depth_0/')
        self.declare_parameter('bin_deg', 10.0)
        self.declare_parameter('pitch_fov_deg', 20.0)   # 参与避障的水平环带半俯仰角(度)
        self.declare_parameter('min_range', 0.3)        # 有效距离下限(米)
        self.declare_parameter('max_range', 30.0)       # 有效距离上限(米)
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('quat_wxyz', [1.0, 0.0, 0.0, 0.0])  # 雷达系->机体系(FRD) 旋转
        self.declare_parameter('trans_xyz', [0.0, 0.0, 0.0])       # 雷达系->机体系(FRD) 平移
        # 或从 /tf 获取外参: 配置 tf_parent_frame(机体frame)/tf_child_frame(点云frame) 后,
        # 自动 lookup 静态外参, 优先于上面的 quat/trans (雷达倾斜安装时用这个)
        self.declare_parameter('tf_parent_frame', '')
        self.declare_parameter('tf_child_frame', '')
        # 机体包络半尺寸 (FRD: x前/y右/z下), 位于包络内的点视为打到自己机身上的回波, 剔除
        self.declare_parameter('body_half_size', [0.5, 0.45, 0.4])
        # use_sim_time 由 rclpy 预声明, 直接读取即可

        cloud_topic = self.get_parameter('cloud_topic').value
        namespace = self.get_parameter('namespace').value
        self.bin_deg = float(self.get_parameter('bin_deg').value)
        self.pitch_fov_deg = float(self.get_parameter('pitch_fov_deg').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        tf_parent = self.get_parameter('tf_parent_frame').value
        tf_child = self.get_parameter('tf_child_frame').value

        self.n_bins = int(round(360.0 / self.bin_deg))  # 36
        self.max_cm = int(round(self.max_range * 100.0))
        self.clear_value = self.max_cm + 1              # 无回波 -> 明确无障碍

        if tf_child:
            # 从 /tf(/tf_static) 自动获取雷达系 -> 机体系 外参
            R_flu, t_flu = self._lookup_extrinsic(tf_parent, tf_child)
            # 机体系从 ROS(FLU, z上) 转到 PX4 FRD(z下): Rx(180°)
            r_flu2frd = np.array([[1.0, 0.0, 0.0],
                                  [0.0, -1.0, 0.0],
                                  [0.0, 0.0, -1.0]])
            self.R = r_flu2frd @ R_flu
            self.t = r_flu2frd @ t_flu
            self.get_logger().info(f'外参来自 /tf: {tf_child} -> {tf_parent}')
            self.get_logger().info('R(FRD)=\n' + np.array2string(self.R, precision=4) +
                                   f'\nt(FRD)={np.round(self.t, 4).tolist()}')
        else:
            q = self.get_parameter('quat_wxyz').value
            t = self.get_parameter('trans_xyz').value
            self.R = quat_to_rotation_matrix(*[float(v) for v in q])
            self.t = np.array([float(v) for v in t], dtype=np.float64)
        self.body_half = np.array([float(v) for v in self.get_parameter('body_half_size').value])

        # 点云订阅: 传感器数据一般 best_effort
        cloud_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, cloud_topic, self.cloud_cb, cloud_qos)

        # 发布到 uXRCE-DDS 输入话题, 直达 PX4 uORB obstacle_distance
        out_topic = namespace.rstrip('/') + '/fmu/in/obstacle_distance'
        self.pub = self.create_publisher(ObstacleDistance, out_topic, 10)

        self._latest = None          # (stamp_ns, distances list)
        self._timeout_ns = int(0.5 * 1e9)
        self.create_timer(1.0 / publish_hz, self.timer_cb)

        self.get_logger().info(
            f'订阅点云: {cloud_topic} -> 发布: {out_topic}, bins={self.n_bins} '
            f'({self.bin_deg}deg), pitch_fov=±{self.pitch_fov_deg}deg, '
            f'range=[{self.min_range}, {self.max_range}]m, '
            f'body_mask(FRD half)={self.body_half.tolist()}')

    def _lookup_extrinsic(self, parent, child):
        """从 /tf(/tf_static) 获取 child->parent 静态外参, 返回 (R_3x3, t_3)。

        R/t 满足 p_parent = R * p_child + t (ROS FLU 系)。
        重试最多 10s, 失败则抛异常(避免用错误外参静默飞行)。
        """
        if not parent:
            raise RuntimeError('tf_parent_frame 未配置')
        buffer = Buffer()
        TransformListener(buffer, self, spin_thread=True)
        deadline = time_module.monotonic() + 10.0
        last_err = None
        while time_module.monotonic() < deadline:
            try:
                ts = buffer.lookup_transform(parent, child, Time(), Duration(seconds=1.0))
                q = ts.transform.rotation
                t = ts.transform.translation
                R = quat_to_rotation_matrix(q.w, q.x, q.y, q.z)
                return R, np.array([t.x, t.y, t.z], dtype=np.float64)
            except Exception as e:  # noqa: BLE001 - tf2 异常类型多且派生自同一基类
                last_err = e
                time_module.sleep(0.5)
        raise RuntimeError(f'获取静态外参失败 {child}->{parent}: {last_err}')

    def cloud_cb(self, msg):
        pts = pointcloud2_to_xyz(msg)
        if pts is None or len(pts) == 0:
            return

        # 雷达系 -> 机体系 FRD (x 前, y 右, z 下)
        p = pts @ self.R.T + self.t

        x = p[:, 0]
        y = p[:, 1]
        z = p[:, 2]

        d_h = np.hypot(x, y)
        # 方位角: 0 度=机头, +90 度=机右(FRD)
        az = np.degrees(np.arctan2(y, x))
        # 俯仰角: 相对机体水平面, 上为正
        pitch = np.degrees(np.arctan2(-z, d_h))

        # 机体包络剔除: 落在机体尺寸内的回波视为打到机体自身(机臂/机身), 不参与避障
        in_body = (np.abs(x) < self.body_half[0]) & \
                  (np.abs(y) < self.body_half[1]) & \
                  (np.abs(z) < self.body_half[2])

        valid = (
            (d_h >= self.min_range)
            & (d_h <= self.max_range)
            & (np.abs(pitch) <= self.pitch_fov_deg)
            & ~in_body
        )
        if not np.any(valid):
            return

        az_v = az[valid]
        d_v = d_h[valid]

        # 分 bin, 每个 bin 取最小水平距离
        bin_idx = np.floor(az_v / self.bin_deg).astype(np.int64) % self.n_bins
        bin_min = np.full(self.n_bins, np.inf, dtype=np.float64)
        np.minimum.at(bin_min, bin_idx, d_v)

        distances = [UINT16_MAX] * 72
        for i in range(self.n_bins):
            if np.isfinite(bin_min[i]):
                cm = int(round(bin_min[i] * 100.0))
                distances[i] = min(cm, self.max_cm)
            else:
                distances[i] = self.clear_value

        # 注意: 时间戳必须换算成完整纳秒(sec+nanosec), 否则与 get_clock().now() 比较会永远超时
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self._latest = (stamp_ns, distances)

    def timer_cb(self):
        if self._latest is None:
            return
        stamp_ns, distances = self._latest
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - stamp_ns > self._timeout_ns:
            # 数据过期, 停止发布, 让 PX4 判定数据失效 (配合 CP_GO_NO_DATA 决策)
            return

        msg = ObstacleDistance()
        msg.timestamp = int(now_ns // 1000)          # us, PX4 用其判断数据新鲜度
        msg.frame = ObstacleDistance.MAV_FRAME_BODY_FRD
        msg.sensor_type = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
        msg.increment = float(self.bin_deg)
        msg.min_distance = int(round(self.min_range * 100.0))
        msg.max_distance = self.max_cm
        msg.angle_offset = 0.0
        msg.distances = distances
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDistancePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
