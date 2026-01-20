# XTDrone2

## 介绍

XTDrone2是基于PX4、ROS2与Gazebo Ignition的无人机通用仿真平台。

在[XTDrone](https://gitee.com/robin_shaun/XTDrone)的基础上，XTDrone2更新采用了更模块化和轻量化的仿真器Gazebo Ignition；同时由于ROS1版本不再更新维护，XTDrone2将全部基于ROS2进行开发；同时PX4的版本也采用了更新更稳定的1.15版本。

限于开源项目团队人力有限并且为爱发电，过程中难免会存在许多问题，很感激您能够向我们反馈您遇到的BUG或解决方案；过程中遇到的BUG请通过仓库的issue向我们提出，您遇到的问题也可以尝试在issue中先寻求答案。

## 使用手册和教程文档

详细内容参见[XTDrone2安装教程](https://www.yuque.com/xtdrone/xtdrone2/tutorial)

## 项目团队

## 贡献者

## 加入我们

## 建立合作

## PX4构建问题修复记录

### 问题描述
在构建PX4 v1.15.3时遇到构建失败，错误信息显示事件系统JSON文件版本不匹配：
```
Traceback (most recent call last):
  File "/home/ywj/git/PX4-Autopilot/src/lib/events/libevents/scripts/combine.py", line 93, in <module>
    main()
  File "/home/ywj/git/PX4-Autopilot/src/lib/events/libevents/scripts/combine.py", line 35, in main
    assert new_events["version"] == 1
AssertionError
```

### 根本原因分析
通过git submodule检查发现，多个子模块使用了比PX4 v1.15.3更新的开发版本，导致版本不兼容：

- `src/lib/events/libevents` - 使用了版本2的JSON格式
- `src/modules/mavlink/mavlink` - 使用了不兼容的开发版本
- `src/drivers/uavcan/libuavcan` - 使用了不兼容的开发版本
- `platforms/nuttx/NuttX/apps` - 使用了不兼容的开发版本
- `platforms/nuttx/NuttX/nuttx` - 使用了不兼容的开发版本

### 解决方案
正确的解决方案是**将子模块恢复到与PX4 v1.15.3兼容的版本**，而不是修改代码来适应不兼容的子模块版本。

#### 修复步骤

1. **检查子模块状态**
   ```bash
   git submodule status
   git submodule foreach 'echo "$name: $(git describe --tags --always 2>/dev/null || echo "No tags") - $(git log --oneline -1)"'
   ```

2. **恢复不兼容的子模块**
   ```bash
   # 修复events子模块
   git submodule deinit -f src/lib/events/libevents
   git submodule update --init src/lib/events/libevents
   
   # 修复mavlink子模块（需要递归初始化）
   git submodule deinit -f src/modules/mavlink/mavlink
   git submodule update --init --recursive src/modules/mavlink/mavlink
   
   # 修复其他不兼容的子模块
   git submodule deinit -f src/drivers/uavcan/libuavcan
   git submodule update --init src/drivers/uavcan/libuavcan
   
   git submodule deinit -f platforms/nuttx/NuttX/apps
   git submodule update --init platforms/nuttx/NuttX/apps
   
   git submodule deinit -f platforms/nuttx/NuttX/nuttx
   git submodule update --init platforms/nuttx/NuttX/nuttx
   ```

3. **验证修复结果**
   ```bash
   make clean
   make px4_sitl
   ```

### 关键经验
- **不要修改combine.py脚本**来支持版本2，这会破坏版本兼容性
- 正确的做法是**保持子模块与主项目版本的同步**
- 在大型项目中，子模块版本管理非常重要
- 遇到构建问题时，首先检查git submodule状态

### 修复结果
✅ 构建成功 - 所有1029个目标都成功编译完成  
✅ 版本兼容性 - 所有子模块现在都与PX4 v1.15.3版本兼容  
✅ 问题解决 - 原始的错误已经完全消除

## drone_detect模块编译问题修复记录

### 问题描述
在编译XTDrone2项目中的`drone_detect`模块时遇到编译错误：
```
In file included from .../drone_detector.h:26:10: fatal error: cv_bridge/cv_bridge.hpp: 没有那个文件或目录
  26 | #include <cv_bridge/cv_bridge.hpp>
      |          ^~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
```

### 根本原因分析
通过检查发现，问题在于ROS2 Humble版本中`cv_bridge`的头文件路径发生了变化：

- **错误路径**: `cv_bridge/cv_bridge.hpp`
- **正确路径**: `cv_bridge/cv_bridge.h`

在ROS2 Humble中，`cv_bridge`的头文件实际位于：
- `/opt/ros/humble/include/cv_bridge/cv_bridge/cv_bridge.h`
- `/opt/ros/humble/include/cv_bridge/cv_bridge/cv_bridge_export.h`

### 解决方案
修改头文件包含路径，将错误的`.hpp`扩展名改为正确的`.h`扩展名。

#### 修复步骤

1. **定位问题文件**
   ```bash
   # 检查cv_bridge实际头文件位置
   find /opt/ros/humble -name "*cv_bridge*" -type f | grep -E "\.hpp|\.h"
   ```

2. **修改头文件包含**
   在文件 `drone_detector.h` 中：
   ```cpp
   // 错误：
   #include <cv_bridge/cv_bridge.hpp>
   
   // 正确：
   #include <cv_bridge/cv_bridge.h>
   ```

3. **验证修复**
   ```bash
   # 重新编译项目
   colcon build --packages-select drone_detect
   ```

### 关键经验
- **ROS2版本差异**: 不同ROS2版本可能有不同的头文件路径和命名规范
- **头文件检查**: 遇到头文件缺失时，应先检查实际安装路径
- **版本兼容性**: 第三方库在不同ROS2版本中可能有细微差异

### 修复结果
✅ 头文件路径修正 - 正确引用ROS2 Humble版本的cv_bridge头文件  
✅ 编译错误消除 - `drone_detect`模块可以正常编译  
✅ 兼容性保证 - 确保与ROS2 Humble版本的兼容性


### 确保安装gazebo harmonic，并卸载ignition gazebo
<!-- sudo apt install ros-humble-ros-gz-bridge -->
sudo apt install ros-humble-tf-transformations 
<!-- sudo apt install libignition-gazebo6-plugins -->

<!-- sudo apt remove libignition-* -->
<!-- sudo apt install ros-humble-ros-gzharmonic -->
sudo apt install ros-humble-ros-gzgarden


xtd2_ws/XTDrone2_ego_planner/xtd2_launch/xtd2_launch/utils/px4_launch.py  16行配置px4目录


cd ~/git/xtd2_ws
colcon build --symlink-install --parallel-workers 8
source install/setup.bash


## 启动飞机单独模拟
PX4_SIM_MODEL=gz_x500_depth PX4_GZ_WORLD=tugbot_warehouse /home/ywj/git/PX4-Autopilot/build/px4_sitl_default/bin/px4


PX4_UXRCE_DDS_NS=x500_depth_0 PX4_GZ_WORLD=tugbot_warehouse PX4_SYS_AUTOSTART=4002 PX4_SIM_MODEL=x500_depth PX4_GZ_MODEL_POSE='0.0,0.0,0.0,0.0,0.0,0.0' PX4_GZ_MODELS=~/git/PX4-Autopilot/Tools/simulation/gz/models PX4_GZ_WORLDS=~/git/PX4-Autopilot/Tools/simulation/gz/worlds XTD2_GZ_MODELS=/home/ywj/git/xtd2_ws/install/xtd2_gz_sim/share/xtd2_gz_sim/models GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PX4_GZ_MODELS:$PX4_GZ_WORLDS:/home/ywj/git/xtd2_ws/install/xtd2_gz_sim/share/xtd2_gz_sim/models ~/git/PX4-Autopilot/build/px4_sitl_default/bin/px4 -d -s ~/git/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/rcS ~/git/PX4-Autopilot/ROMFS/px4fmu_common -i 0 -w ~/git/PX4-Autopilot/build/px4_sitl_default


## 启动XTDrones2模拟
ros2 launch xtd2_launch ros2_single_vehicle_demo_launch.py

## 地面站QGC
./QGroundControl-x86_64.AppImage

## 键盘控制
ros2 run xtd2_control multirotor_keyboard_control --model gz_x500_depth --id 0

## 可视化
rviz2


ros2 launch xtd2_launch ego_planner_launch.py 

## 清除后台残余
/home/ywj/git/xtd2_ws/XTDrone2_ego_planner/clear_background.sh

## 近期重要更新记录

### 2026-01-19 - 修复模拟环境问题，换用gz garden
**提交**: c9ace48ccdad2f6b5ca361047d99f6f51ec359f4

#### 主要更新内容
- **模拟环境升级**: 从Ignition Gazebo迁移到Gazebo Garden
- **新增ego_planner启动文件**: 创建完整的ego planner启动配置
- **TF关系优化**: 改进坐标变换发布逻辑
- **PX4启动配置**: 更新PX4启动脚本配置

#### 关键文件修改
- `xtd2_launch/launch/ego_planner_launch.py` - 新增完整的ego planner启动配置
- `xtd2_launch/launch/launch_config/ros_gz_bridge.yaml` - 更新ROS-Gazebo桥接配置
- `xtd2_launch/launch/tf_publisher.py` - 优化TF坐标变换发布
- `xtd2_gz_sim/models/OakD-Lite/model.sdf` - 修复相机模型配置

#### 依赖更新
```bash
# 确保安装Gazebo Garden相关包
sudo apt install ros-humble-ros-gzgarden
sudo apt install ros-humble-tf-transformations
```

### 2026-01-15 - 修正TF关系
**提交**: 10bebd1c39cd965f78f01bba929d3ffe187a9f27

#### 主要改进
- **URDF模型优化**: 修正x500_depth无人机的TF坐标关系
- **坐标变换精度**: 提高传感器和机体坐标变换的准确性
- **可视化一致性**: 确保RViz显示与实际仿真一致

#### 影响范围
- 所有使用x500_depth模型的仿真场景
- 传感器数据与机体坐标的转换精度
- 点云和图像数据的正确显示

### 2026-01-13 - 修正点云显示
**提交**: b50518d2b7f17a7ed3f76040d2b9392539217e84

#### 问题修复
- **点云显示异常**: 修复深度相机点云在RViz中的显示问题
- **模型配置优化**: 改进OakD-Lite相机模型的SDF配置
- **桥接配置更新**: 优化ROS-Gazebo点云数据桥接

#### 技术细节
- 修正了相机坐标系与点云数据的转换关系
- 优化了点云数据的发布频率和质量
- 提高了点云在仿真环境中的可视化效果

### 2026-01-12 - 同步更新
**提交**: e9de8e3

#### 主要内容
- **代码同步**: 保持与主分支的同步更新
- **依赖管理**: 更新项目依赖配置
- **构建优化**: 改进colcon构建配置

### 2026-01-09 - 基础功能完善
**提交**: 6749e0c, d45ecbf, 60508f5, cdf8daa

#### 主要功能
- **TF发布器**: 实现完整的坐标变换发布系统
- **室内环境**: 添加室内环境模型加载功能
- **启动方式**: 恢复标准的launch文件启动方式
- **仿真对接**: 完成与Gazebo仿真的完整对接

#### 重要说明
- 启动程序应为`gz sim`而非`ign gazebo`
- 需要去除Ignition相关主要组件，保留必要插件
- 确保与ROS2 Humble版本的兼容性

## 使用注意事项

### 环境要求
- **ROS2版本**: Humble Hawksbill
- **Gazebo版本**: Garden (推荐) 或 Harmonic
- **PX4版本**: 1.15.3
- **Python版本**: 3.10+

### 配置要点
1. **PX4目录配置**: 在`xtd2_launch/xtd2_launch/utils/px4_launch.py`第16行配置PX4目录
2. **环境变量**: 确保正确设置所有必要的环境变量
3. **模型路径**: 正确配置Gazebo模型搜索路径

### 构建命令
```bash
cd ~/git/xtd2_ws
colcon build --symlink-install --parallel-workers 8
source install/setup.bash
```

### 常见问题
1. **点云显示异常**: 检查TF关系和相机模型配置
2. **仿真启动失败**: 确认Gazebo版本和模型路径
3. **规划失败**: 检查ego planner参数配置和传感器数据
