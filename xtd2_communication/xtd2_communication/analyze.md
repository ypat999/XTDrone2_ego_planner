# livox_frame → base_link 坐标变换分析

## 1. /lio/odom 发布的是什么位姿？

Super-LIO 的 ESKF 状态 `state.R` 和 `state.p` 代表 **livox_frame（雷达）在世界系下的位姿**，不是竖直的 base_link 位姿。

### 证据：初始化逻辑

`super_lio.cpp` 初始化时：

```cpp
// gravity = -mean_acce * g / |mean_acce|   → 体坐标系下的重力方向（倾斜的，因为雷达倾斜30°）
// ref_gravity = [0, 0, -9.8]               → 世界坐标系下的重力方向（竖直向下）
M3 init_rot = Quat::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();
M3 rot = g_lidar_robo_yaw * R_yaw_inv * init_rot;
state.R = SO3(rot);
state.p = g_odom_robo.t_;  // = [0,0,0]（因为 extrinsic_odom_robo 全为0）
```

`FromTwoVectors(gravity, ref_gravity)` 把体坐标系下的重力方向旋转到世界系下的重力方向。因为雷达倾斜30°安装，体坐标系下的重力方向也是倾斜的，所以 **`init_rot` 已经包含了30°倾斜修正**。

`R_yaw_inv` 只去除 yaw 不确定性，不影响 pitch/roll。

### 证据：点云变换

```cpp
// super_lio.cpp:822-823
V3& point_body = points_body_v3_[idx];       // livox_frame 下的点
V3 point_world = pose * point_body;           // 变换到世界系
```

`pose` 就是 `state.R, state.p`，它把 livox_frame 下的点变换到世界系，说明 `state.R` 是 `R_world_livox`。

### 证据：TF 发布

```cpp
// ROSWrapper.cpp:854-861
tf_msg.header.frame_id = g_world_frame;
tf_msg.child_frame_id = g_imu_frame;
tf_msg.transform.translation = state.p;   // livox 在世界系下的位置
tf_msg.transform.rotation = state.R;      // livox 在世界系下的旋转
```

### 结论

**`/lio/odom` 发布的 `state.R` 是 livox_frame 在世界系下的旋转（含30°倾斜角），`state.p` 是 livox_frame 原点在世界系下的位置。不是 base_link 的位姿。**

---

## 2. 当前 publish_px4_visual_odometry 的转换是否正确？

### 位置转换

```python
flu_x = msg.pose.pose.position.x   # 实际是 LIO 世界系坐标，不是 FLU
flu_y = msg.pose.pose.position.y
flu_z = msg.pose.pose.position.z
px4_msg.position = CoordinateTransform.flu_to_ned_position(flu_x, flu_y, flu_z, self.init_heading)
```

`flu_to_ned_position` 配合 `init_heading` 是正确的。因为 Super-LIO 的世界系 X 轴方向 = IMU 初始朝向（经过 `R_yaw_inv` 对齐），而 `init_heading` 补偿了这个世界系 X 轴与 NED 北向之间的夹角。

**但位置是 livox_frame 的，不是 base_link 的，这就是 5cm 偏移的根源。**

### 姿态转换

```python
if self.cur_vehicle_odometry is not None:
    px4_msg.q = self.cur_vehicle_odometry.q   # 直接用 PX4 自己的姿态
```

当前代码直接用了 PX4 自己的姿态，避开了倾斜问题。这是务实的做法，因为 PX4 的 IMU 能很好地估计水平姿态。

---

## 3. 注释掉的代码为什么会导致飞行混乱？

```python
# 第一次变换：livox_frame → base_link
transform = self.tf_buffer.lookup_transform('livox_frame', 'base_link', ...)
odom_transformed = tf2_geometry_msgs.do_transform_pose(msg.pose.pose, transform)

# 第二次变换：world → base_link（覆盖了第一次的结果！）
transform = self.tf_buffer.lookup_transform('world', 'base_link', ...)
odom_transformed = tf2_geometry_msgs.do_transform_pose(msg.pose.pose, transform)
```

### 三个致命错误

1. **两次变换，第二次覆盖第一次**：第一次计算的结果被第二次完全覆盖，第一次毫无意义。

2. **`do_transform_pose` 语义理解错误**：`lookup_transform('world', 'base_link')` 得到的是 world→base_link 的变换。`do_transform_pose(world_坐标系下的位姿, world→base_link变换)` 会把世界坐标系下的位姿**变换到 base_link 坐标系下**。这给出的是"世界原点在 base_link 下的位置"，而不是"base_link 在世界下的位置"。

3. **frame_id 设置错误**：变换后设 `msg.header.frame_id = 'base_link'`，但 `publish_px4_visual_odometry` 期望的是世界坐标系下的位置。

**结果**：PX4 收到的是一个在 base_link 坐标系下的位姿，被当作 NED 世界坐标系下的位姿使用，导致位置和姿态完全错乱 → 飞行混乱。

---

## 4. 正确的转换方式

需要计算的是 **base_link 在世界坐标系下的位姿**。

### 变换链

```
T_world_base = T_world_livox × T_livox_base
```

具体来说：
- **位置**：`p_base_world = R_world_livox × t_livox_base + p_livox_world`
- **姿态**：`R_world_base = R_world_livox × R_livox_base`

其中 `T_livox_base` 来自 launch 文件中的静态 TF：

```python
arguments=['-0.05', '0', '-0.15', str(deg_to_rad(180)), str(deg_to_rad(210)), '0', 'livox_frame', 'base_link']
```

这表示 `p_livox = R_lb × p_base + t_lb`，其中 `t_lb = (-0.05, 0, -0.15)` 是 base_link 原点在 livox_frame 下的坐标。

---

## 5. 修改方案（方案 B）

在 `ros2_odom_callback` 中手动计算 base_link 在世界系下的位置，再传给 `publish_px4_visual_odometry`。

### 核心公式

```
p_base_world = R_world_livox × t_livox_base + p_livox_world
```

- **R_world_livox**：从 `/lio/odom` 的四元数提取的旋转矩阵（livox 在世界系下的姿态，含 30° 倾斜）
- **t_livox_base**：base_link 原点在 livox_frame 下的坐标，来自静态 TF `[-0.05, 0, -0.15]`
- **p_livox_world**：livox 在世界系下的位置（`/lio/odom` 原始位置）

### 与之前注释代码的区别

| | 旧代码（注释掉的） | 新代码 |
|---|---|---|
| **变换方式** | `do_transform_pose` 两次，第二次覆盖第一次 | 一次矩阵乘法 `R_wl @ t_lb + p_livox` |
| **语义** | 把世界系位姿变换到 base_link 坐标系下（错误） | 计算 base_link 在世界系下的位置（正确） |
| **TF 查找** | 查了两次，第二次查 `world→base_link`（动态 TF 不存在） | 只查一次 `livox_frame→base_link`（静态 TF，缓存） |
| **frame_id** | 错误地设为 `base_link` | 不修改 frame_id，位置本身已是世界系坐标 |

### 关键设计决策

1. **只改位置，不改姿态**：PX4 姿态仍用自身 IMU（`cur_vehicle_odometry.q`），这是最稳定的做法
2. **缓存静态 TF**：`_livox_to_base_t` 只查找一次，避免每帧重复查找
3. **保留原始 LIO 高度**：`cur_lio_vehicle_odometry` 保存原始 livox 位姿用于着陆高度判断，不受 base_link 偏移影响
4. **仿真环境跳过**：`hostname` 为仿真环境时不做变换

### 预期效果

当无人机水平悬停时，5cm 的前向偏移会被修正：
- 修正前：PX4 把 livox 放在目标位置 → base_link 偏前 5cm
- 修正后：PX4 把 base_link 放在目标位置 → livox 偏后 5cm（但 PX4 不关心 livox 位置）

---

## 6. 数值验证

### 水平悬停时的偏移计算

无人机水平悬停时，`R_world_livox` 包含30°倾斜。假设 yaw=0，livox_frame 的 X 轴在世界系中朝前偏下30°。

静态 TF 参数：`t_livox_base = [-0.05, 0, -0.15]`

水平悬停时 R_world_livox 的简化形式（yaw=0，绕 Y 轴旋转 -30° + 180°翻转）：

```
p_base = R_wl @ [-0.05, 0, -0.15] + p_livox
```

这会在世界系 X（前向）和 Z（上下）方向产生偏移，正好补偿 livox 和 base_link 之间的物理位移差异。

### 为什么之前直接用 livox 位置会有 5cm 前向偏移

因为 base_link 在 livox_frame 下 x=-0.05（即 base_link 在 livox 后方 5cm），当无人机水平时，livox 的位置比 base_link 靠前 5cm。PX4 把 livox 位置当作 base_link 位置，所以 base_link 实际偏前 5cm。
