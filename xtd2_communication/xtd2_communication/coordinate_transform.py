"""
坐标系转换工具函数

坐标系定义:
- NED (North-East-Down): x=北, y=东, z=下 (PX4世界系, 右手系)
- ENU (East-North-Up): x=东, y=北, z=上 (ROS标准世界系, 右手系)
- FLU (Forward-Left-Up): x=前, y=左, z=上 (ROS机体系, 右手系)
- FRD (Forward-Right-Down): x=前, y=右, z=下 (PX4机体系, 右手系)

重要说明:
- FLU、FRD、NED、ENU 都是右手坐标系
- PX4 vehicle_odometry 使用 FRD (机体系) 和 NED (世界系)
- ROS 使用 FLU (机体系) 和 ENU (世界系)
- Heading定义: PX4输出的是body_x与正北的夹角，顺时针为正

标准转换矩阵:
- NED → ENU: [[0,1,0], [1,0,0], [0,0,-1]]
- FRD → FLU: [[1,0,0], [0,-1,0], [0,0,-1]]

关键注意事项:
1. vehicle_odometry.velocity 是 FRD (机体系)，不是 NED！
2. vehicle_odometry.q 是 FRD-relative-to-NED，需要同时转换:
   - FRD → FLU (机体系)
   - NED → ENU (世界系)

Author: Andy Zhuo
Email: zhuoan@stu.pku.edu.cn
"""

import math
import numpy as np


class CoordinateTransform:
    """坐标系转换工具类"""

    # =========================================================================
    # NED <-> ENU 转换 (右手系之间的转换)
    # =========================================================================

    @staticmethod
    def ned_to_enu_position(x: float, y: float, z: float) -> list:
        """NED位置 -> ENU位置"""
        return [y, x, -z]

    @staticmethod
    def enu_to_ned_position(x: float, y: float, z: float) -> list:
        """ENU位置 -> NED位置"""
        return [y, x, -z]

    @staticmethod
    def ned_to_enu_velocity(vx: float, vy: float, vz: float) -> list:
        """NED速度 -> ENU速度"""
        return [vy, vx, -vz]

    @staticmethod
    def enu_to_ned_velocity(vx: float, vy: float, vz: float) -> list:
        """ENU速度 -> NED速度"""
        return [vy, vx, -vz]

    @staticmethod
    def ned_to_enu_acceleration(ax: float, ay: float, az: float) -> list:
        """NED加速度 -> ENU加速度"""
        return [ay, ax, -az]

    @staticmethod
    def enu_to_ned_acceleration(ax: float, ay: float, az: float) -> list:
        """ENU加速度 -> NED加速度"""
        return [ay, ax, -az]

    @staticmethod
    def ned_to_enu_quaternion(qw: float, qx: float, qy: float, qz: float) -> list:
        """NED四元数 -> ENU四元数"""
        R_ned = CoordinateTransform._quat_to_rot(qw, qx, qy, qz)
        T = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        R_enu = T @ R_ned @ T.T
        return CoordinateTransform._rot_to_quat(R_enu)

    @staticmethod
    def enu_to_ned_quaternion(qw: float, qx: float, qy: float, qz: float) -> list:
        """ENU四元数 -> NED四元数"""
        R_enu = CoordinateTransform._quat_to_rot(qw, qx, qy, qz)
        T = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        R_ned = T.T @ R_enu @ T
        return CoordinateTransform._rot_to_quat(R_ned)

    # =========================================================================
    # FRD <-> FLU 转换 (PX4机体系 <-> ROS机体系)
    # =========================================================================

    @staticmethod
    def frd_to_flu_position(x: float, y: float, z: float) -> list:
        """FRD位置 -> FLU位置
        
        FRD: x=前, y=右, z=下
        FLU: x=前, y=左, z=上
        
        转换: [x, y, z] -> [x, -y, -z]
        """
        return [x, -y, -z]

    @staticmethod
    def flu_to_frd_position(x: float, y: float, z: float) -> list:
        """FLU位置 -> FRD位置"""
        return [x, -y, -z]

    @staticmethod
    def frd_to_flu_velocity(vx: float, vy: float, vz: float) -> list:
        """FRD速度 -> FLU速度"""
        return [vx, -vy, -vz]

    @staticmethod
    def flu_to_frd_velocity(vx: float, vy: float, vz: float) -> list:
        """FLU速度 -> FRD速度"""
        return [vx, -vy, -vz]

    @staticmethod
    def frd_to_flu_acceleration(ax: float, ay: float, az: float) -> list:
        """FRD加速度 -> FLU加速度"""
        return [ax, -ay, -az]

    @staticmethod
    def flu_to_frd_acceleration(ax: float, ay: float, az: float) -> list:
        """FLU加速度 -> FRD加速度"""
        return [ax, -ay, -az]

    @staticmethod
    def frd_to_flu_angular_velocity(wx: float, wy: float, wz: float) -> list:
        """FRD角速度 -> FLU角速度"""
        return [wx, -wy, -wz]

    @staticmethod
    def flu_to_frd_angular_velocity(wx: float, wy: float, wz: float) -> list:
        """FLU角速度 -> FRD角速度"""
        return [wx, -wy, -wz]

    @staticmethod
    def frd_to_flu_quaternion(qw: float, qx: float, qy: float, qz: float) -> list:
        """
        FRD四元数 -> FLU四元数
        
        FRD和FLU都是右手系，只是y和z轴反向。
        转换矩阵: [[1,0,0], [0,-1,0], [0,0,-1]]
        
        对于纯旋转（如姿态），这个转换相当于绕x轴旋转180度。
        """
        # FRD->FLU 转换矩阵
        T = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        R_frd = CoordinateTransform._quat_to_rot(qw, qx, qy, qz)
        R_flu = T @ R_frd @ T.T
        return CoordinateTransform._rot_to_quat(R_flu)

    @staticmethod
    def flu_to_frd_quaternion(qw: float, qx: float, qy: float, qz: float) -> list:
        """FLU四元数 -> FRD四元数"""
        T = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        R_flu = CoordinateTransform._quat_to_rot(qw, qx, qy, qz)
        R_frd = T @ R_flu @ T.T
        return CoordinateTransform._rot_to_quat(R_frd)

    # =========================================================================
    # BODY (FLU) <-> NED 转换 (都是右手系，使用标准旋转矩阵)
    # =========================================================================

    @staticmethod
    def body_to_ned_matrix(heading: float) -> np.ndarray:
        """
        创建从 FLU 到 NED 的 2D 旋转矩阵 (标准旋转，行列式=+1)

        几何关系:
        n = f * cos(ψ) - l * sin(ψ)
        e = f * sin(ψ) + l * cos(ψ)

        矩阵:
        [n]   [cos(ψ)  -sin(ψ)] [f]
        [e] = [sin(ψ)   cos(ψ)] [l]

        Args:
            heading: 航向角 ψ (弧度)，顺时针为正
        Returns:
            2x2标准旋转矩阵，行列式=+1
        """
        c = math.cos(heading)
        s = math.sin(heading)
        return np.array([[c, -s], [s, c]])

    @staticmethod
    def ned_to_body_matrix(heading: float) -> np.ndarray:
        """
        创建从 NED 到 FLU 的 2D 旋转矩阵 (转置即逆)

        Args:
            heading: 航向角 ψ (弧度)
        Returns:
            2x2标准旋转矩阵
        """
        c = math.cos(heading)
        s = math.sin(heading)
        return np.array([[c, s], [-s, c]])

    @staticmethod
    def body_to_ned_3d_matrix(heading: float) -> np.ndarray:
        """
        创建从 FLU 到 NED 的 3D 旋转矩阵

        完整变换:
        [n]   [cos(ψ)  -sin(ψ)   0] [f]
        [e] = [sin(ψ)   cos(ψ)   0] [l]
        [d]   [   0        0    -1] [u]

        Args:
            heading: 航向角 ψ (弧度)
        Returns:
            3x3矩阵 (标准旋转 + Z轴翻转)
        """
        c = math.cos(heading)
        s = math.sin(heading)
        return np.array([
            [c, -s, 0],
            [s,  c, 0],
            [0,  0, -1]
        ])

    @staticmethod
    def ned_to_body_3d_matrix(heading: float) -> np.ndarray:
        """
        创建从 NED 到 FLU 的 3D 旋转矩阵

        Args:
            heading: 航向角 ψ (弧度)
        Returns:
            3x3矩阵
        """
        c = math.cos(heading)
        s = math.sin(heading)
        return np.array([
            [ c, s, 0],
            [-s, c, 0],
            [ 0, 0, -1]
        ])

    @staticmethod
    def flu_to_ned_position(flu_x: float, flu_y: float, flu_z: float, heading: float) -> list:
        """FLU位置 -> NED位置"""
        R = CoordinateTransform.body_to_ned_3d_matrix(heading)
        ned_vec = R @ np.array([flu_x, flu_y, flu_z])
        return [ned_vec[0], ned_vec[1], ned_vec[2]]

    @staticmethod
    def ned_to_flu_position(ned_x: float, ned_y: float, ned_z: float, heading: float) -> list:
        """NED位置 -> FLU位置"""
        R = CoordinateTransform.ned_to_body_3d_matrix(heading)
        flu_vec = R @ np.array([ned_x, ned_y, ned_z])
        return [flu_vec[0], flu_vec[1], flu_vec[2]]

    @staticmethod
    def flu_to_ned_velocity(flu_vx: float, flu_vy: float, flu_vz: float, heading: float) -> list:
        """FLU速度 -> NED速度"""
        R = CoordinateTransform.body_to_ned_3d_matrix(heading)
        ned_vec = R @ np.array([flu_vx, flu_vy, flu_vz])
        return [ned_vec[0], ned_vec[1], ned_vec[2]]

    @staticmethod
    def ned_to_flu_velocity(ned_vx: float, ned_vy: float, ned_vz: float, heading: float) -> list:
        """NED速度 -> FLU速度"""
        R = CoordinateTransform.ned_to_body_3d_matrix(heading)
        flu_vec = R @ np.array([ned_vx, ned_vy, ned_vz])
        return [flu_vec[0], flu_vec[1], flu_vec[2]]

    @staticmethod
    def flu_to_ned_acceleration(flu_ax: float, flu_ay: float, flu_az: float, heading: float) -> list:
        """FLU加速度 -> NED加速度"""
        R = CoordinateTransform.body_to_ned_3d_matrix(heading)
        ned_vec = R @ np.array([flu_ax, flu_ay, flu_az])
        return [ned_vec[0], ned_vec[1], ned_vec[2]]

    @staticmethod
    def ned_to_flu_acceleration(ned_ax: float, ned_ay: float, ned_az: float, heading: float) -> list:
        """NED加速度 -> FLU加速度"""
        R = CoordinateTransform.ned_to_body_3d_matrix(heading)
        flu_vec = R @ np.array([ned_ax, ned_ay, ned_az])
        return [flu_vec[0], flu_vec[1], flu_vec[2]]

    # =========================================================================
    # 角速度转换 (不涉及世界坐标，不需要 heading)
    # =========================================================================

    @staticmethod
    def flu_to_ned_angular_velocity(flu_wx: float, flu_wy: float, flu_wz: float) -> list:
        """
        FLU角速度 -> NED角速度

        注意: 角速度是机体坐标量，不涉及世界坐标
        转换只是坐标轴重新排列:
        ω_ned = [wy, wx, -wz]

        Args:
            flu_wx, flu_wy, flu_wz: FLU坐标系下的角速度
        Returns:
            [w_north, w_east, w_down]: NED坐标系下的角速度
        """
        return [flu_wy, flu_wx, -flu_wz]

    @staticmethod
    def ned_to_flu_angular_velocity(ned_wx: float, ned_wy: float, ned_wz: float) -> list:
        """
        NED角速度 -> FLU角速度
        """
        return [ned_wy, ned_wx, -ned_wz]

    # =========================================================================
    # FRD <-> NED 转换 (PX4机体系 <-> PX4世界系)
    # =========================================================================

    @staticmethod
    def frd_to_ned_velocity(vx: float, vy: float, vz: float, heading: float) -> list:
        """
        FRD速度 -> NED速度
        
        vehicle_odometry.velocity 是 FRD 坐标系！
        需要先 FRD->FLU，再 FLU->NED
        """
        # FRD -> FLU
        flu_vx, flu_vy, flu_vz = vx, -vy, -vz
        # FLU -> NED
        return CoordinateTransform.flu_to_ned_velocity(flu_vx, flu_vy, flu_vz, heading)

    @staticmethod
    def ned_to_frd_velocity(vx: float, vy: float, vz: float, heading: float) -> list:
        """
        NED速度 -> FRD速度
        """
        # NED -> FLU
        flu_vel = CoordinateTransform.ned_to_flu_velocity(vx, vy, vz, heading)
        # FLU -> FRD
        return [flu_vel[0], -flu_vel[1], -flu_vel[2]]

    @staticmethod
    def frd_to_ned_angular_velocity(wx: float, wy: float, wz: float) -> list:
        """
        FRD角速度 -> NED角速度
        
        vehicle_odometry.angular_velocity 是 FRD 坐标系！
        """
        # FRD -> FLU
        flu_wx, flu_wy, flu_wz = wx, -wy, -wz
        # FLU -> NED (只是坐标轴重排)
        return [flu_wy, flu_wx, -flu_wz]

    @staticmethod
    def ned_to_frd_angular_velocity(wx: float, wy: float, wz: float) -> list:
        """
        NED角速度 -> FRD角速度
        """
        # NED -> FLU
        flu_wx, flu_wy, flu_wz = wy, wx, -wz
        # FLU -> FRD
        return [flu_wx, -flu_wy, -flu_wz]

    # =========================================================================
    # VehicleOdometry 专用转换 (FRD/NED <-> FLU/ENU)
    # =========================================================================

    @staticmethod
    def frd_ned_to_flu_enu_velocity(frd_vx: float, frd_vy: float, frd_vz: float, 
                                     heading: float) -> list:
        """
        vehicle_odometry 速度: FRD -> FLU -> ENU
        
        这是从 PX4 接收 velocity 时的完整转换链
        """
        # FRD -> FLU
        flu_vx, flu_vy, flu_vz = frd_vx, -frd_vy, -frd_vz
        # FLU -> NED (使用heading)
        ned_vel = CoordinateTransform.flu_to_ned_velocity(flu_vx, flu_vy, flu_vz, heading)
        # NED -> ENU
        return CoordinateTransform.ned_to_enu_velocity(ned_vel[0], ned_vel[1], ned_vel[2])

    @staticmethod
    def flu_enu_to_frd_ned_velocity(flu_vx: float, flu_vy: float, flu_vz: float,
                                     heading: float) -> list:
        """
        速度: ENU -> NED -> FLU -> FRD
        
        这是发送 velocity 到 PX4 时的完整转换链
        """
        # ENU -> NED
        ned_vel = CoordinateTransform.enu_to_ned_velocity(flu_vx, flu_vy, flu_vz)
        # NED -> FLU
        flu_vel = CoordinateTransform.ned_to_flu_velocity(ned_vel[0], ned_vel[1], ned_vel[2], heading)
        # FLU -> FRD
        return [flu_vel[0], -flu_vel[1], -flu_vel[2]]

    @staticmethod
    def frd_ned_to_flu_enu_quaternion(frd_qw: float, frd_qx: float, frd_qy: float, frd_qz: float) -> list:
        """
        vehicle_odometry 姿态四元数: FRD/NED -> FLU/ENU
        
        vehicle_odometry.q 表示 FRD-relative-to-NED
        需要同时转换:
        1. FRD -> FLU (机体系转换)
        2. NED -> ENU (世界系转换)
        
        数学上: q_flu_enu = q_ned_enu * q_frd_ned * q_frd_flu
        """
        # 1. FRD -> FLU (机体系转换)
        T_body = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        R_frd_ned = CoordinateTransform._quat_to_rot(frd_qw, frd_qx, frd_qy, frd_qz)
        R_flu_ned = T_body @ R_frd_ned @ T_body.T
        
        # 2. NED -> ENU (世界系转换)
        T_world = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        R_flu_enu = T_world @ R_flu_ned @ T_world.T
        
        return CoordinateTransform._rot_to_quat(R_flu_enu)

    @staticmethod
    def flu_enu_to_frd_ned_quaternion(flu_qw: float, flu_qx: float, flu_qy: float, flu_qz: float) -> list:
        """
        姿态四元数: FLU/ENU -> FRD/NED
        """
        # 1. ENU -> NED (世界系转换)
        T_world = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        R_flu_enu = CoordinateTransform._quat_to_rot(flu_qw, flu_qx, flu_qy, flu_qz)
        R_flu_ned = T_world.T @ R_flu_enu @ T_world
        
        # 2. FLU -> FRD (机体系转换)
        T_body = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        R_frd_ned = T_body @ R_flu_ned @ T_body.T
        
        return CoordinateTransform._rot_to_quat(R_frd_ned)

    @staticmethod
    def flu_to_ned_yawspeed(flu_wz: float) -> float:
        """FLU yawspeed -> NED yawspeed"""
        return -flu_wz

    @staticmethod
    def ned_to_flu_yawspeed(ned_wz: float) -> float:
        """NED yawspeed -> FLU yawspeed"""
        return -ned_wz

    # =========================================================================
    # BODY (FLU) <-> ENU 转换 (通过 NED 中转)
    # =========================================================================

    @staticmethod
    def flu_to_enu_position(flu_x: float, flu_y: float, flu_z: float, heading: float) -> list:
        """FLU位置 -> ENU位置"""
        ned_pos = CoordinateTransform.flu_to_ned_position(flu_x, flu_y, flu_z, heading)
        return CoordinateTransform.ned_to_enu_position(ned_pos[0], ned_pos[1], ned_pos[2])

    @staticmethod
    def enu_to_flu_position(enu_x: float, enu_y: float, enu_z: float, heading: float) -> list:
        """ENU位置 -> FLU位置"""
        ned_pos = CoordinateTransform.enu_to_ned_position(enu_x, enu_y, enu_z)
        return CoordinateTransform.ned_to_flu_position(ned_pos[0], ned_pos[1], ned_pos[2], heading)

    @staticmethod
    def flu_to_enu_velocity(flu_vx: float, flu_vy: float, flu_vz: float, heading: float) -> list:
        """FLU速度 -> ENU速度"""
        ned_vel = CoordinateTransform.flu_to_ned_velocity(flu_vx, flu_vy, flu_vz, heading)
        return CoordinateTransform.ned_to_enu_velocity(ned_vel[0], ned_vel[1], ned_vel[2])

    @staticmethod
    def enu_to_flu_velocity(enu_vx: float, enu_vy: float, enu_vz: float, heading: float) -> list:
        """ENU速度 -> FLU速度"""
        ned_vel = CoordinateTransform.enu_to_ned_velocity(enu_vx, enu_vy, enu_vz)
        return CoordinateTransform.ned_to_flu_velocity(ned_vel[0], ned_vel[1], ned_vel[2], heading)

    @staticmethod
    def flu_to_enu_acceleration(flu_ax: float, flu_ay: float, flu_az: float, heading: float) -> list:
        """FLU加速度 -> ENU加速度"""
        ned_accel = CoordinateTransform.flu_to_ned_acceleration(flu_ax, flu_ay, flu_az, heading)
        return CoordinateTransform.ned_to_enu_acceleration(ned_accel[0], ned_accel[1], ned_accel[2])

    @staticmethod
    def enu_to_flu_acceleration(enu_ax: float, enu_ay: float, enu_az: float, heading: float) -> list:
        """ENU加速度 -> FLU加速度"""
        ned_accel = CoordinateTransform.enu_to_ned_acceleration(enu_ax, enu_ay, enu_az)
        return CoordinateTransform.ned_to_flu_acceleration(ned_accel[0], ned_accel[1], ned_accel[2], heading)

    # =========================================================================
    # 使用完整旋转矩阵的转换 (用于 TF 直接转换)
    # =========================================================================

    @staticmethod
    def flu_to_enu_vector_by_rotation_matrix(flu_vec: np.ndarray, rotation_matrix: np.ndarray) -> np.ndarray:
        """
        使用完整旋转矩阵将 FLU 向量转换到 ENU

        Args:
            flu_vec: FLU坐标系下的向量 [x, y, z]
            rotation_matrix: 3x3 旋转矩阵 (FLU -> ENU)
        Returns:
            ENU坐标系下的向量
        """
        return rotation_matrix @ flu_vec

    @staticmethod
    def enu_to_flu_vector_by_rotation_matrix(enu_vec: np.ndarray, rotation_matrix: np.ndarray) -> np.ndarray:
        """
        使用完整旋转矩阵将 ENU 向量转换到 FLU

        Args:
            enu_vec: ENU坐标系下的向量 [x, y, z]
            rotation_matrix: 3x3 旋转矩阵 (FLU -> ENU)
        Returns:
            FLU坐标系下的向量
        """
        return rotation_matrix.T @ enu_vec

    @staticmethod
    def flu_to_ned_vector_by_rotation_matrix(flu_vec: np.ndarray, rotation_matrix: np.ndarray) -> np.ndarray:
        """
        使用完整旋转矩阵将 FLU 向量转换到 NED

        转换链: FLU -> ENU -> NED

        Args:
            flu_vec: FLU坐标系下的向量 [x, y, z]
            rotation_matrix: 3x3 旋转矩阵 (FLU -> ENU)
        Returns:
            NED坐标系下的向量 [north, east, down]
        """
        enu_vec = rotation_matrix @ flu_vec
        return np.array(CoordinateTransform.enu_to_ned_velocity(
            enu_vec[0], enu_vec[1], enu_vec[2]))

    @staticmethod
    def ned_to_flu_vector_by_rotation_matrix(ned_vec: np.ndarray, rotation_matrix: np.ndarray) -> np.ndarray:
        """
        使用完整旋转矩阵将 NED 向量转换到 FLU

        转换链: NED -> ENU -> FLU

        Args:
            ned_vec: NED坐标系下的向量 [north, east, down]
            rotation_matrix: 3x3 旋转矩阵 (FLU -> ENU)
        Returns:
            FLU坐标系下的向量
        """
        enu_vec = np.array(CoordinateTransform.ned_to_enu_velocity(
            ned_vec[0], ned_vec[1], ned_vec[2]))
        return rotation_matrix.T @ enu_vec

    # =========================================================================
    # 工具函数
    # =========================================================================

    @staticmethod
    def heading_from_quaternion(qw: float, qx: float, qy: float, qz: float) -> float:
        """从四元数提取航向角 (yaw)"""
        return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

    @staticmethod
    def normalize_quaternion(qw: float, qx: float, qy: float, qz: float) -> list:
        """归一化四元数"""
        norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if norm < 1e-10:
            return [1.0, 0.0, 0.0, 0.0]
        return [qw/norm, qx/norm, qy/norm, qz/norm]

    @staticmethod
    def quaternion_conjugate(qw: float, qx: float, qy: float, qz: float) -> list:
        """四元数共轭 (逆旋转)"""
        return [qw, -qx, -qy, -qz]

    @staticmethod
    def create_yaw_rotation_matrix(yaw: float) -> np.ndarray:
        """创建绕Z轴旋转的3D旋转矩阵"""
        c = math.cos(yaw)
        s = math.sin(yaw)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    @staticmethod
    def create_rotation_matrix_from_quaternion(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
        """从四元数创建3D旋转矩阵"""
        return CoordinateTransform._quat_to_rot(qw, qx, qy, qz)

    @staticmethod
    def inverse_transform(position: list, orientation: list) -> tuple:
        """
        计算变换的逆

        给定一个变换 T = (position, orientation)，计算其逆变换 T_inv。
        如果 T 表示 frame_A -> frame_B，则 T_inv 表示 frame_B -> frame_A。

        数学上:
        - 逆旋转: q_inv = q_conj (四元数共轭)
        - 逆位置: p_inv = -R_inv @ p

        Args:
            position: [x, y, z] 位置向量
            orientation: [qw, qx, qy, qz] 四元数 (w-first)
        Returns:
            (inv_position, inv_orientation): 逆变换的位置和姿态
            inv_position: [x, y, z]
            inv_orientation: [qw, qx, qy, qz] (w-first)

        Example:
            # world -> px4_odom 的变换
            pos = [1.0, 2.0, 3.0]
            quat = [1.0, 0.0, 0.0, 0.0]  # 单位四元数
            # 计算 px4_odom -> world 的逆变换
            inv_pos, inv_quat = CoordinateTransform.inverse_transform(pos, quat)
        """
        qw, qx, qy, qz = orientation

        # 逆旋转: 四元数共轭
        inv_orientation = [qw, -qx, -qy, -qz]

        # 逆位置: p_inv = -R_inv @ p
        # R_inv 是逆旋转的旋转矩阵，即共轭四元数对应的旋转矩阵
        R_inv = CoordinateTransform._quat_to_rot(qw, -qx, -qy, -qz)
        p = np.array(position)
        inv_position = -R_inv @ p

        return inv_position.tolist(), inv_orientation

    @staticmethod
    def compose_transforms(pos1: list, quat1: list, pos2: list, quat2: list) -> tuple:
        """
        组合两个变换: T = T1 @ T2

        先应用 T2，再应用 T1。
        如果 T1: A->B, T2: B->C，则结果 T: A->C

        Args:
            pos1, quat1: 第一个变换的位置和四元数 (w-first)
            pos2, quat2: 第二个变换的位置和四元数 (w-first)
        Returns:
            (composed_pos, composed_quat): 组合后的变换
        """
        # 旋转组合: q = q1 * q2
        composed_quat = CoordinateTransform.quaternion_multiply(quat1, quat2)

        # 位置组合: p = p1 + R1 @ p2
        R1 = CoordinateTransform._quat_to_rot(*quat1)
        p1 = np.array(pos1)
        p2 = np.array(pos2)
        composed_pos = p1 + R1 @ p2

        return composed_pos.tolist(), composed_quat

    @staticmethod
    def quaternion_from_rotation_matrix(R: np.ndarray) -> list:
        """从3D旋转矩阵创建四元数"""
        return CoordinateTransform._rot_to_quat(R)

    @staticmethod
    def quaternion_multiply(q1: list, q2: list) -> list:
        """四元数乘法 q1 * q2"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        return [w, x, y, z]

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> list:
        """航向角 -> 四元数 (仅绕Z轴旋转)"""
        half_yaw = yaw / 2.0
        return [math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw)]

    @staticmethod
    def quaternion_to_yaw(qw: float, qx: float, qy: float, qz: float) -> float:
        """四元数 -> 航向角"""
        return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

    # =========================================================================
    # 内部辅助函数
    # =========================================================================

    @staticmethod
    def _quat_to_rot(w: float, x: float, y: float, z: float) -> np.ndarray:
        """四元数转旋转矩阵"""
        return np.array([
            [1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
            [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
            [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]
        ])

    @staticmethod
    def _rot_to_quat(R: np.ndarray) -> list:
        """旋转矩阵转四元数 (Shepperd方法)"""
        tr = np.trace(R)
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            w = 0.25 * S
            x = (R[2,1] - R[1,2]) / S
            y = (R[0,2] - R[2,0]) / S
            z = (R[1,0] - R[0,1]) / S
        elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            w = (R[2,1] - R[1,2]) / S
            x = 0.25 * S
            y = (R[0,1] + R[1,0]) / S
            z = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            w = (R[0,2] - R[2,0]) / S
            x = (R[0,1] + R[1,0]) / S
            y = 0.25 * S
            z = (R[1,2] + R[2,1]) / S
        else:
            S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            w = (R[1,0] - R[0,1]) / S
            x = (R[0,2] + R[2,0]) / S
            y = (R[1,2] + R[2,1]) / S
            z = 0.25 * S
        return [float(w), float(x), float(y), float(z)]

    # =========================================================================
    # px4_odom 专用转换函数 (用于视觉里程计)
    # =========================================================================

    @staticmethod
    def flu_to_ned_position_px4_odom(flu_x: float, flu_y: float, flu_z: float,
                                      init_n: float, init_e: float, init_d: float,
                                      init_heading: float) -> tuple:
        """
        px4_odom 专用的 FLU -> NED 位置转换
        
        这个转换考虑了初始位置和 heading 的补偿，
        用于将 Gazebo 的 FLU 坐标转换为 PX4 的 NED 坐标。
        
        Args:
            flu_x, flu_y, flu_z: FLU 坐标系下的位置
            init_n, init_e, init_d: 初始 NED 位置
            init_heading: 初始航向角 (弧度)
        Returns:
            (compensated_n, compensated_e, compensated_d): 补偿后的 NED 位置
        """
        # 将 FLU 偏移转换为 NED 偏移
        ned_offset = CoordinateTransform.flu_to_ned_position(flu_x, flu_y, flu_z, init_heading)
        
        # 加上初始位置
        compensated_n = init_n + ned_offset[0]
        compensated_e = init_e + ned_offset[1]
        compensated_d = init_d + ned_offset[2]
        
        return (compensated_n, compensated_e, compensated_d)

    @staticmethod
    def flu_to_ned_quaternion_px4_odom(flu_qw: float, flu_qx: float, flu_qy: float, flu_qz: float,
                                        init_heading: float) -> list:
        """
        px4_odom 专用的 FLU -> NED 四元数转换
        
        将 FLU/ENU 姿态转换为 FRD/NED 姿态，
        这是发送给 vehicle_odometry 消息的正确格式。
        
        Args:
            flu_qw, flu_qx, flu_qy, flu_qz: FLU/ENU 四元数
            init_heading: 初始航向角 (弧度，用于验证)
        Returns:
            [qw, qx, qy, qz]: FRD/NED 四元数
        """
        # FLU/ENU -> FRD/NED
        return CoordinateTransform.flu_enu_to_frd_ned_quaternion(flu_qw, flu_qx, flu_qy, flu_qz)
