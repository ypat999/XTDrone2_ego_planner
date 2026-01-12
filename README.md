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

sudo apt remove libignition-*
sudo apt install ros-humble-ros-gzharmonic


xtd2_ws/XTDrone2_ego_planner/xtd2_launch/xtd2_launch/utils/px4_launch.py  16行配置px4目录


cd ~/git/xtd2_ws
colcon build --symlink-install --parallel-workers 8
source install/setup.bash


## 启动飞机单独模拟
PX4_SIM_MODEL=gz_x500 /home/ywj/git/PX4-Autopilot/build/px4_sitl_default/bin/px4


## 启动XTDrones2模拟
ros2 launch xtd2_launch ros2_single_vehicle_demo_launch.py

## 地面站QGC
./QGroundControl-x86_64.AppImage

## 键盘控制
ros2 run xtd2_control multirotor_keyboard_control --model gz_x500 --id 0

## 可视化
rviz2

## 清除后台残余
/home/ywj/git/xtd2_ws/XTDrone2_ego_planner/clear_background.sh


