#!/bin/bash

# 定义目标点数组 (x y z)
# 格式: "x y z"
points=(
    "0.0 0.0 1.0"
    "0.0 0.0 1.0"
    "0.0 0.0 1.0"
    "0.0 0.0 0.2"
    "0.0 0.0 0.2"
    "0.0 0.0 0.2"
    "0.0 0.0 1.0"
    "0.0 0.0 1.0"
    "0.0 0.0 1.0"
    "-1.0 2.0 2.0"
    "-1.0 2.0 2.0"
    "-1.0 2.0 2.0"
    "-1.0 2.0 2.0"
)

echo "开始循环发布目标点，按 Ctrl+C 停止..."

while true; do
    for pt in "${points[@]}"; do
        # 将字符串拆分为坐标
        read -r x y z <<< "$pt"
        
        echo "------------------------------------------------"
        echo "正在发布目标点: [X: $x, Y: $y, Z: $z]"

        # 发布消息
        ros2 topic pub --once /goal_pose_3d geometry_msgs/msg/PoseStamped "{
            header: {
                stamp: now,
                frame_id: 'map'
            },
            pose: {
                position: {x: $x, y: $y, z: $z},
                orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
            }
        }"

        echo "等待 1 秒..."
        sleep 1
    done
done