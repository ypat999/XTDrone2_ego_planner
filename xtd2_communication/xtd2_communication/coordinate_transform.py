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

try:
    from .coordinate_transform_cpp import CoordinateTransform as CoordinateTransformCPP
    USE_CPP_IMPL = True
except ImportError:
    USE_CPP_IMPL = False
    print("Warning: C++ implementation not available, falling back to Python implementation")


class CoordinateTransform:
    """坐标系转换工具类 - Python包装器"""

    _cpp_impl = None
    
    @classmethod
    def _get_cpp_impl(cls):
        if cls._cpp_impl is None and USE_CPP_IMPL:
            cls._cpp_impl = CoordinateTransformCPP()
        return cls._cpp_impl

    @staticmethod
    def ned_to_enu_position(x: float, y: float, z: float) -> list:
        """NED位置 -> ENU位置"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().ned_to_enu_position(x, y, z)
        return [y, x, -z]

    @staticmethod
    def enu_to_ned_position(x: float, y: float, z: float) -> list:
        """ENU位置 -> NED位置"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().enu_to_ned_position(x, y, z)
        return [y, x, -z]

    @staticmethod
    def ned_to_enu_velocity(vx: float, vy: float, vz: float) -> list:
        """NED速度 -> ENU速度"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().ned_to_enu_velocity(vx, vy, vz)
        return [vy, vx, -vz]

    @staticmethod
    def enu_to_ned_velocity(vx: float, vy: float, vz: float) -> list:
        """ENU速度 -> NED速度"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().enu_to_ned_velocity(vx, vy, vz)
        return [vy, vx, -vz]

    @staticmethod
    def ned_to_enu_acceleration(ax: float, ay: float, az: float) -> list:
        """NED加速度 -> ENU加速度"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().ned_to_enu_acceleration(ax, ay, az)
        return [ay, ax, -az]

    @staticmethod
    def enu_to_ned_acceleration(ax: float, ay: float, az: float) -> list:
        """ENU加速度 -> NED加速度"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().enu_to_ned_acceleration(ax, ay, az)
        return [ay, ax, -az]

    @staticmethod
    def ned_to_enu_quaternion(qw: float, qx: float, qy: float, qz: float) -> list:
        """NED四元数 -> ENU四元数"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().ned_to_enu_quaternion(qw, qx, qy, qz)
        R_ned = CoordinateTransform._quat_to_rot(qw, qx, qy, qz)
        T = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        R_enu = T @ R_ned
        return CoordinateTransform._rot_to_quat(R_enu)

    @staticmethod
    def enu_to_ned_quaternion(qw: float, qx: float, qy: float, qz: float) -> list:
        """ENU四元数 -> NED四元数"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().enu_to_ned_quaternion(qw, qx, qy, qz)
        R_enu = CoordinateTransform._quat_to_rot(qw, qx, qy, qz)
        T = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        R_ned = T.T @ R_enu
        return CoordinateTransform._rot_to_quat(R_ned)

    @staticmethod
    def frd_to_flu_position(x: float, y: float, z: float) -> list:
        """FRD位置 -> FLU位置
        
        FRD: x=前, y=右, z=下
        FLU: x=前, y=左, z=上
        
        转换: [x, y, z] -> [x, -y, -z]
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().frd_to_flu_position(x, y, z)
        return [x, -y, -z]

    @staticmethod
    def flu_to_frd_position(x: float, y: float, z: float) -> list:
        """FLU位置 -> FRD位置"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().flu_to_frd_position(x, y, z)
        return [x, -y, -z]

    @staticmethod
    def frd_to_flu_velocity(vx: float, vy: float, vz: float) -> list:
        """FRD速度 -> FLU速度"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().frd_to_flu_velocity(vx, vy, vz)
        return [vx, -vy, -vz]

    @staticmethod
    def flu_to_frd_velocity(vx: float, vy: float, vz: float) -> list:
        """FLU速度 -> FRD速度"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().flu_to_frd_velocity(vx, vy, vz)
        return [vx, -vy, -vz]

    @staticmethod
    def frd_to_flu_acceleration(ax: float, ay: float, az: float) -> list:
        """FRD加速度 -> FLU加速度"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().frd_to_flu_acceleration(ax, ay, az)
        return [ax, -ay, -az]

    @staticmethod
    def flu_to_frd_acceleration(ax: float, ay: float, az: float) -> list:
        """FLU加速度 -> FRD加速度"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().flu_to_frd_acceleration(ax, ay, az)
        return [ax, -ay, -az]

    @staticmethod
    def frd_to_flu_quaternion(qw: float, qx: float, qy: float, qz: float) -> list:
        """
        FRD四元数 -> FLU四元数
        
        FRD: x=前, y=右, z=下
        FLU: x=前, y=左, z=上
        
        轴反射矩阵: T = diag(1, -1, -1)
        表示: x不变, y反向, z反向
        
        旋转矩阵转换: R_flu = T @ R_frd @ T
        (注意: 对于反射，使用 T 而不是 T.T)
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().frd_to_flu_quaternion(qw, qx, qy, qz)
        T = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        R_frd = CoordinateTransform._quat_to_rot(qw, qx, qy, qz)
        R_flu = T @ R_frd @ T
        return CoordinateTransform._rot_to_quat(R_flu)

    @staticmethod
    def flu_to_frd_quaternion(qw: float, qx: float, qy: float, qz: float) -> list:
        """
        FLU四元数 -> FRD四元数
        
        轴反射矩阵: T = diag(1, -1, -1)
        旋转矩阵转换: R_frd = T @ R_flu @ T
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().flu_to_frd_quaternion(qw, qx, qy, qz)
        T = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        R_flu = CoordinateTransform._quat_to_rot(qw, qx, qy, qz)
        R_frd = T @ R_flu @ T
        return CoordinateTransform._rot_to_quat(R_frd)

    @staticmethod
    def body_to_ned_matrix(heading: float) -> np.ndarray:
        """
        创建从 FLU 到 NED 的 2D 反射矩阵 (行列式=-1)

        重要: FLU (左手系) -> NED (右手系) 需要反射矩阵，不是纯旋转矩阵！

        几何关系:
        n = f * cos(ψ) + l * sin(ψ)
        e = f * sin(ψ) - l * cos(ψ)

        矩阵:
        [n]   [cos(ψ)   sin(ψ)] [f]
        [e] = [sin(ψ)  -cos(ψ)] [l]

        验证 det = cos*(-cos) - sin*sin = -cos² - sin² = -1 ✓

        Args:
            heading: 航向角 ψ (弧度)，顺时针为正
        Returns:
            2x2反射矩阵，行列式=-1
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().body_to_ned_matrix(heading)
        c = math.cos(heading)
        s = math.sin(heading)
        return np.array([[c, s], [s, -c]])

    @staticmethod
    def ned_to_body_matrix(heading: float) -> np.ndarray:
        """
        创建从 NED 到 FLU 的 2D 反射矩阵 (逆矩阵)

        注意: 反射矩阵的逆 = 自身 (因为 R @ R = I)

        Args:
            heading: 航向角 ψ (弧度)
        Returns:
            2x2反射矩阵
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().ned_to_body_matrix(heading)
        c = math.cos(heading)
        s = math.sin(heading)
        return np.array([[c, s], [s, -c]])

    @staticmethod
    def body_to_ned_3d_matrix(heading: float) -> np.ndarray:
        """
        创建从 FLU 到 NED 的 3D 变换矩阵

        这个矩阵用于将 FLU 坐标系下的向量转换到 NED 坐标系。
        它结合了:
        1. 航向旋转 (FLU -> ENU 的投影)
        2. Z轴翻转 (Up -> Down)

        矩阵形式:
        [n]   [cos(ψ)   sin(ψ)   0] [f]
        [e] = [sin(ψ)  -cos(ψ)   0] [l]
        [d]   [   0        0    -1] [u]

        性质（已验算，勿凭直觉改写）:
        本矩阵 = Rz(ψ) · diag(1, -1, -1)，即"水平面内朝向旋转"复合"机体系 FLU->FRD 轴反射"。
        2x2 水平子块 [[c, s], [s, -c]] 的 det = -c²-s² = -1（反射），
        但再乘上 z 轴翻转的 -1 之后，整体 det = +1 且恒满足 M · Mᵀ = I（任意 ψ）。
        ⇒ 它是一个【正交旋转矩阵】，不是退化矩阵，不存在 ψ->45°/135° 失效问题。
        历史版本此处曾误写 "det = cos(2ψ)、非正交"，导致有人据此"修正"这段正确的代码，
        故在此明确记录。参见 05_46_38_事故复核结论与参数清单.md §13.1(b)。

        Args:
            heading: 航向角 ψ (弧度)，顺时针为正
        Returns:
            3x3变换矩阵
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().body_to_ned_3d_matrix(heading)
        c = math.cos(heading)
        s = math.sin(heading)
        return np.array([
            [c,  s, 0],
            [s, -c, 0],
            [0,  0, -1]
        ])

    @staticmethod
    def ned_to_body_3d_matrix(heading: float) -> np.ndarray:
        """
        创建从 NED 到 FLU 的 3D 变换矩阵

        这是 body_to_ned_3d_matrix 的逆矩阵。
        注意: body_to_ned_3d_matrix 是【正交】矩阵（见其 docstring 的验算），
        故逆矩阵 = 转置；又因为它同时是对称且对合的（M = Mᵀ, M·M = I），
        所以逆矩阵在数值上【等于自身】—— 这也是下面直接复用同一组元素的原因。
        历史版本此处曾误写"原矩阵不是正交矩阵，逆矩阵不等于转置"，属错误说明。

        Args:
            heading: 航向角 ψ (弧度)
        Returns:
            3x3变换矩阵
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().ned_to_body_3d_matrix(heading)
        c = math.cos(heading)
        s = math.sin(heading)
        return np.array([
            [c, s, 0],
            [s, -c, 0],
            [0, 0, -1]
        ])

    @staticmethod
    def flu_to_ned_position(flu_x: float, flu_y: float, flu_z: float, heading: float) -> list:
        """FLU位置 -> NED位置"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().flu_to_ned_position(flu_x, flu_y, flu_z, heading)
        R = CoordinateTransform.body_to_ned_3d_matrix(heading)
        ned_vec = R @ np.array([flu_x, flu_y, flu_z])
        return [ned_vec[0], ned_vec[1], ned_vec[2]]

    @staticmethod
    def ned_to_flu_position(ned_x: float, ned_y: float, ned_z: float, heading: float) -> list:
        """NED位置 -> FLU位置"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().ned_to_flu_position(ned_x, ned_y, ned_z, heading)
        R = CoordinateTransform.ned_to_body_3d_matrix(heading)
        flu_vec = R @ np.array([ned_x, ned_y, ned_z])
        return [flu_vec[0], flu_vec[1], flu_vec[2]]

    @staticmethod
    def flu_to_ned_velocity(flu_vx: float, flu_vy: float, flu_vz: float, heading: float) -> list:
        """FLU速度 -> NED速度"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().flu_to_ned_velocity(flu_vx, flu_vy, flu_vz, heading)
        R = CoordinateTransform.body_to_ned_3d_matrix(heading)
        ned_vec = R @ np.array([flu_vx, flu_vy, flu_vz])
        return [ned_vec[0], ned_vec[1], ned_vec[2]]

    @staticmethod
    def ned_to_flu_velocity(ned_vx: float, ned_vy: float, ned_vz: float, heading: float) -> list:
        """NED速度 -> FLU速度"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().ned_to_flu_velocity(ned_vx, ned_vy, ned_vz, heading)
        R = CoordinateTransform.ned_to_body_3d_matrix(heading)
        flu_vec = R @ np.array([ned_vx, ned_vy, ned_vz])
        return [flu_vec[0], flu_vec[1], flu_vec[2]]

    @staticmethod
    def flu_to_ned_acceleration(flu_ax: float, flu_ay: float, flu_az: float, heading: float) -> list:
        """FLU加速度 -> NED加速度"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().flu_to_ned_acceleration(flu_ax, flu_ay, flu_az, heading)
        R = CoordinateTransform.body_to_ned_3d_matrix(heading)
        ned_vec = R @ np.array([flu_ax, flu_ay, flu_az])
        return [ned_vec[0], ned_vec[1], ned_vec[2]]

    @staticmethod
    def ned_to_flu_acceleration(ned_ax: float, ned_ay: float, ned_az: float, heading: float) -> list:
        """NED加速度 -> FLU加速度"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().ned_to_flu_acceleration(ned_ax, ned_ay, ned_az, heading)
        R = CoordinateTransform.ned_to_body_3d_matrix(heading)
        flu_vec = R @ np.array([ned_ax, ned_ay, ned_az])
        return [flu_vec[0], flu_vec[1], flu_vec[2]]

    @staticmethod
    def flu_to_frd_angular_velocity(flu_wx: float, flu_wy: float, flu_wz: float) -> list:
        """
        FLU角速度 -> FRD角速度

        重要: 角速度是机体系量，不需要世界坐标转换！
        ROS: angular velocity -> body frame (FLU)
        PX4: angular velocity -> body frame (FRD)
        只需要坐标轴反射: wx=wx, wy=-wy, wz=-wz

        Args:
            flu_wx, flu_wy, flu_wz: FLU坐标系下的角速度
        Returns:
            [w_forward, w_right, w_down]: FRD坐标系下的角速度
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().flu_to_frd_angular_velocity(flu_wx, flu_wy, flu_wz)
        return [flu_wx, -flu_wy, -flu_wz]

    @staticmethod
    def frd_to_flu_angular_velocity(frd_wx: float, frd_wy: float, frd_wz: float) -> list:
        """
        FRD角速度 -> FLU角速度

        只需要坐标轴反射: wx=wx, wy=-wy, wz=-wz

        Args:
            frd_wx, frd_wy, frd_wz: FRD坐标系下的角速度
        Returns:
            [w_forward, w_left, w_up]: FLU坐标系下的角速度
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().frd_to_flu_angular_velocity(frd_wx, frd_wy, frd_wz)
        return [frd_wx, -frd_wy, -frd_wz]

    @staticmethod
    def frd_to_ned_velocity(vx: float, vy: float, vz: float, heading: float) -> list:
        """
        FRD速度 -> NED速度
        
        vehicle_odometry.velocity 是 FRD 坐标系！
        需要先 FRD->FLU，再 FLU->NED
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().frd_to_ned_velocity(vx, vy, vz, heading)
        flu_vx, flu_vy, flu_vz = vx, -vy, -vz
        return CoordinateTransform.flu_to_ned_velocity(flu_vx, flu_vy, flu_vz, heading)

    @staticmethod
    def ned_to_frd_velocity(vx: float, vy: float, vz: float, heading: float) -> list:
        """
        NED速度 -> FRD速度
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().ned_to_frd_velocity(vx, vy, vz, heading)
        flu_vel = CoordinateTransform.ned_to_flu_velocity(vx, vy, vz, heading)
        return [flu_vel[0], -flu_vel[1], -flu_vel[2]]

    @staticmethod
    def frd_ned_to_flu_enu_velocity(frd_vx: float, frd_vy: float, frd_vz: float, 
                                     heading: float) -> list:
        """
        vehicle_odometry 速度: FRD -> FLU -> ENU
        
        这是从 PX4 接收 velocity 时的完整转换链
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().frd_ned_to_flu_enu_velocity(frd_vx, frd_vy, frd_vz, heading)
        flu_vx, flu_vy, flu_vz = frd_vx, -frd_vy, -frd_vz
        ned_vel = CoordinateTransform.flu_to_ned_velocity(flu_vx, flu_vy, flu_vz, heading)
        return CoordinateTransform.ned_to_enu_velocity(ned_vel[0], ned_vel[1], ned_vel[2])

    @staticmethod
    def flu_enu_to_frd_ned_velocity(flu_vx: float, flu_vy: float, flu_vz: float,
                                     heading: float) -> list:
        """
        速度: ENU -> NED -> FLU -> FRD
        
        这是发送 velocity 到 PX4 时的完整转换链
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().flu_enu_to_frd_ned_velocity(flu_vx, flu_vy, flu_vz, heading)
        ned_vel = CoordinateTransform.enu_to_ned_velocity(flu_vx, flu_vy, flu_vz)
        flu_vel = CoordinateTransform.ned_to_flu_velocity(ned_vel[0], ned_vel[1], ned_vel[2], heading)
        return [flu_vel[0], -flu_vel[1], -flu_vel[2]]

    @staticmethod
    def frd_ned_to_flu_enu_quaternion(frd_qw: float, frd_qx: float, frd_qy: float, frd_qz: float) -> list:
        """
        vehicle_odometry 姿态四元数: FRD/NED -> FLU/ENU

        PX4 quaternion: world=NED, body=FRD
        ROS quaternion: world=ENU, body=FLU

        需要变换: R_flu_enu = R_ned_to_enu * R_frd_ned * R_frd_to_flu

        其中:
        - R_ned_to_enu = [[0,1,0], [1,0,0], [0,0,-1]] (NED->ENU)
        - R_frd_to_flu = [[1,0,0], [0,-1,0], [0,0,-1]] (FRD->FLU, 轴反射)
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().frd_ned_to_flu_enu_quaternion(frd_qw, frd_qx, frd_qy, frd_qz)
        R_ned_to_enu = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        R_frd_to_flu = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        R_frd_ned = CoordinateTransform._quat_to_rot(frd_qw, frd_qx, frd_qy, frd_qz)
        R_flu_enu = R_ned_to_enu @ R_frd_ned @ R_frd_to_flu
        return CoordinateTransform._rot_to_quat(R_flu_enu)

    @staticmethod
    def flu_enu_to_frd_ned_quaternion(flu_qw: float, flu_qx: float, flu_qy: float, flu_qz: float) -> list:
        """
        姿态四元数: FLU/ENU -> FRD/NED

        ROS quaternion: world=ENU, body=FLU
        PX4 quaternion: world=NED, body=FRD

        需要变换: R_frd_ned = R_enu_to_ned * R_flu_enu * R_flu_to_frd

        其中:
        - R_enu_to_ned = [[0,1,0], [1,0,0], [0,0,-1]] (ENU->NED)
        - R_flu_to_frd = [[1,0,0], [0,-1,0], [0,0,-1]] (FLU->FRD, 轴反射)
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().flu_enu_to_frd_ned_quaternion(flu_qw, flu_qx, flu_qy, flu_qz)
        R_enu_to_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        R_flu_to_frd = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        R_flu_enu = CoordinateTransform._quat_to_rot(flu_qw, flu_qx, flu_qy, flu_qz)
        R_frd_ned = R_enu_to_ned @ R_flu_enu @ R_flu_to_frd
        return CoordinateTransform._rot_to_quat(R_frd_ned)

    @staticmethod
    def flu_to_ned_yawspeed(flu_wz: float) -> float:
        """FLU yawspeed -> NED yawspeed"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().flu_to_ned_yawspeed(flu_wz)
        return -flu_wz

    @staticmethod
    def ned_to_flu_yawspeed(ned_wz: float) -> float:
        """NED yawspeed -> FLU yawspeed"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().ned_to_flu_yawspeed(ned_wz)
        return -ned_wz

    @staticmethod
    def flu_to_enu_position(flu_x: float, flu_y: float, flu_z: float, heading: float) -> list:
        """FLU位置 -> ENU位置"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().flu_to_enu_position(flu_x, flu_y, flu_z, heading)
        ned_pos = CoordinateTransform.flu_to_ned_position(flu_x, flu_y, flu_z, heading)
        return CoordinateTransform.ned_to_enu_position(ned_pos[0], ned_pos[1], ned_pos[2])

    @staticmethod
    def enu_to_flu_position(enu_x: float, enu_y: float, enu_z: float, heading: float) -> list:
        """ENU位置 -> FLU位置"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().enu_to_flu_position(enu_x, enu_y, enu_z, heading)
        ned_pos = CoordinateTransform.enu_to_ned_position(enu_x, enu_y, enu_z)
        return CoordinateTransform.ned_to_flu_position(ned_pos[0], ned_pos[1], ned_pos[2], heading)

    @staticmethod
    def flu_to_enu_velocity(flu_vx: float, flu_vy: float, flu_vz: float, heading: float) -> list:
        """FLU速度 -> ENU速度"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().flu_to_enu_velocity(flu_vx, flu_vy, flu_vz, heading)
        ned_vel = CoordinateTransform.flu_to_ned_velocity(flu_vx, flu_vy, flu_vz, heading)
        return CoordinateTransform.ned_to_enu_velocity(ned_vel[0], ned_vel[1], ned_vel[2])

    @staticmethod
    def enu_to_flu_velocity(enu_vx: float, enu_vy: float, enu_vz: float, heading: float) -> list:
        """ENU速度 -> FLU速度"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().enu_to_flu_velocity(enu_vx, enu_vy, enu_vz, heading)
        ned_vel = CoordinateTransform.enu_to_ned_velocity(enu_vx, enu_vy, enu_vz)
        return CoordinateTransform.ned_to_flu_velocity(ned_vel[0], ned_vel[1], ned_vel[2], heading)

    @staticmethod
    def flu_to_enu_acceleration(flu_ax: float, flu_ay: float, flu_az: float, heading: float) -> list:
        """FLU加速度 -> ENU加速度"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().flu_to_enu_acceleration(flu_ax, flu_ay, flu_az, heading)
        ned_accel = CoordinateTransform.flu_to_ned_acceleration(flu_ax, flu_ay, flu_az, heading)
        return CoordinateTransform.ned_to_enu_acceleration(ned_accel[0], ned_accel[1], ned_accel[2])

    @staticmethod
    def enu_to_flu_acceleration(enu_ax: float, enu_ay: float, enu_az: float, heading: float) -> list:
        """ENU加速度 -> FLU加速度"""
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().enu_to_flu_acceleration(enu_ax, enu_ay, enu_az, heading)
        ned_accel = CoordinateTransform.enu_to_ned_acceleration(enu_ax, enu_ay, enu_az)
        return CoordinateTransform.ned_to_flu_acceleration(ned_accel[0], ned_accel[1], ned_accel[2], heading)

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
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().flu_to_enu_vector_by_rotation_matrix(flu_vec, rotation_matrix)
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
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().enu_to_flu_vector_by_rotation_matrix(enu_vec, rotation_matrix)
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
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().flu_to_ned_vector_by_rotation_matrix(flu_vec, rotation_matrix)
        enu_vec = rotation_matrix @ flu_vec
        return np.array(CoordinateTransform.enu_to_ned_velocity(
            enu_vec[0], enu_vec[1], enu_vec[2]))

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
        """旋转矩阵转四元数 (Shepperd方法，带浮点安全保护)"""
        tr = np.trace(R)
        if tr > 0:
            S = np.sqrt(max(0.0, tr + 1.0)) * 2
            w = 0.25 * S
            x = (R[2,1] - R[1,2]) / S
            y = (R[0,2] - R[2,0]) / S
            z = (R[1,0] - R[0,1]) / S
        elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = np.sqrt(max(0.0, 1.0 + R[0,0] - R[1,1] - R[2,2])) * 2
            w = (R[2,1] - R[1,2]) / S
            x = 0.25 * S
            y = (R[0,1] + R[1,0]) / S
            z = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = np.sqrt(max(0.0, 1.0 + R[1,1] - R[0,0] - R[2,2])) * 2
            w = (R[0,2] - R[2,0]) / S
            x = (R[0,1] + R[1,0]) / S
            y = 0.25 * S
            z = (R[1,2] + R[2,1]) / S
        else:
            S = np.sqrt(max(0.0, 1.0 + R[2,2] - R[0,0] - R[1,1])) * 2
            w = (R[1,0] - R[0,1]) / S
            x = (R[0,2] + R[2,0]) / S
            y = (R[1,2] + R[2,1]) / S
            z = 0.25 * S
        
        norm = np.sqrt(w*w + x*x + y*y + z*z)
        if norm > 1e-10:
            w, x, y, z = w/norm, x/norm, y/norm, z/norm
        return [float(w), float(x), float(y), float(z)]

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
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().flu_to_ned_position_px4_odom(
                flu_x, flu_y, flu_z, init_n, init_e, init_d, init_heading)
        ned_offset = CoordinateTransform.flu_to_ned_position(flu_x, flu_y, flu_z, init_heading)
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
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().flu_to_ned_quaternion_px4_odom(
                flu_qw, flu_qx, flu_qy, flu_qz, init_heading)
        return CoordinateTransform.flu_enu_to_frd_ned_quaternion(flu_qw, flu_qx, flu_qy, flu_qz)

    @staticmethod
    def qmult(q1_w: float, q1_x: float, q1_y: float, q1_z: float,
              q2_w: float, q2_x: float, q2_y: float, q2_z: float) -> list:
        """
        四元数乘法: q1 * q2
        
        Args:
            q1_w, q1_x, q1_y, q1_z: 第一个四元数
            q2_w, q2_x, q2_y, q2_z: 第二个四元数
        Returns:
            [w, x, y, z]: 结果四元数
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().qmult(q1_w, q1_x, q1_y, q1_z, q2_w, q2_x, q2_y, q2_z)
        w = q1_w * q2_w - q1_x * q2_x - q1_y * q2_y - q1_z * q2_z
        x = q1_w * q2_x + q1_x * q2_w + q1_y * q2_z - q1_z * q2_y
        y = q1_w * q2_y - q1_x * q2_z + q1_y * q2_w + q1_z * q2_x
        z = q1_w * q2_z + q1_x * q2_y - q1_y * q2_x + q1_z * q2_w
        return [w, x, y, z]

    @staticmethod
    def quat2mat(w: float, x: float, y: float, z: float) -> np.ndarray:
        """
        四元数转旋转矩阵
        
        Args:
            w, x, y, z: 四元数
        Returns:
            3x3 旋转矩阵
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().quat2mat(w, x, y, z)
        return CoordinateTransform._quat_to_rot(w, x, y, z)

    @staticmethod
    def inverse_transform(pos_x: float, pos_y: float, pos_z: float,
                          quat_w: float, quat_x: float, quat_y: float, quat_z: float) -> tuple:
        """
        计算逆变换 (位置和四元数)
        
        Args:
            pos_x, pos_y, pos_z: 位置
            quat_w, quat_x, quat_y, quat_z: 四元数
        Returns:
            (inv_pos, inv_quat): 逆位置和逆四元数
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().inverse_transform(
                pos_x, pos_y, pos_z, quat_w, quat_x, quat_y, quat_z)
        R = CoordinateTransform._quat_to_rot(quat_w, quat_x, quat_y, quat_z)
        pos = np.array([pos_x, pos_y, pos_z])
        inv_pos = -R.T @ pos
        inv_quat = [quat_w, -quat_x, -quat_y, -quat_z]
        norm = np.sqrt(sum(q*q for q in inv_quat))
        if norm > 1e-10:
            inv_quat = [q/norm for q in inv_quat]
        return (list(inv_pos), inv_quat)

    @staticmethod
    def heading_from_quaternion(w: float, x: float, y: float, z: float) -> float:
        """
        从四元数提取航向角 (yaw)
        
        Args:
            w, x, y, z: 四元数
        Returns:
            heading: 航向角 (弧度)
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().heading_from_quaternion(w, x, y, z)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def create_rotation_matrix_from_quaternion(w: float, x: float, y: float, z: float) -> np.ndarray:
        """
        从四元数创建旋转矩阵
        
        Args:
            w, x, y, z: 四元数
        Returns:
            3x3 旋转矩阵
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().create_rotation_matrix_from_quaternion(w, x, y, z)
        return CoordinateTransform._quat_to_rot(w, x, y, z)

    @staticmethod
    def multiply_transforms(t1_x: float, t1_y: float, t1_z: float,
                            t1_qw: float, t1_qx: float, t1_qy: float, t1_qz: float,
                            t2_x: float, t2_y: float, t2_z: float,
                            t2_qw: float, t2_qx: float, t2_qy: float, t2_qz: float) -> tuple:
        """
        组合两个变换: result = t1 * t2
        
        Args:
            t1_x, t1_y, t1_z: 第一个变换的位置
            t1_qw, t1_qx, t1_qy, t1_qz: 第一个变换的四元数
            t2_x, t2_y, t2_z: 第二个变换的位置
            t2_qw, t2_qx, t2_qy, t2_qz: 第二个变换的四元数
        Returns:
            (result_pos, result_quat): 组合后的位置和四元数
        """
        if USE_CPP_IMPL:
            return CoordinateTransform._get_cpp_impl().multiply_transforms(
                t1_x, t1_y, t1_z, t1_qw, t1_qx, t1_qy, t1_qz,
                t2_x, t2_y, t2_z, t2_qw, t2_qx, t2_qy, t2_qz)
        result_quat = CoordinateTransform.qmult(t1_qw, t1_qx, t1_qy, t1_qz, t2_qw, t2_qx, t2_qy, t2_qz)
        R1 = CoordinateTransform._quat_to_rot(t1_qw, t1_qx, t1_qy, t1_qz)
        t1_trans = np.array([t1_x, t1_y, t1_z])
        t2_trans = np.array([t2_x, t2_y, t2_z])
        result_trans = t1_trans + R1 @ t2_trans
        return (list(result_trans), result_quat)
