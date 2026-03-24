#!/bin/bash

# 等待 px4_odom 话题发布的脚本

echo "等待话题发布: ${PX4_ODOM_TOPIC}"
echo "最大等待时间: 300秒"
echo "检查间隔: 1秒"
echo "发布后等待时间: 5秒"

start_time=$(date +%s)
timeout=300
check_interval=1
post_wait_delay=5

while [ $(($(date +%s) - start_time)) -lt $timeout ]; do
    if ros2 topic list | grep -q "${PX4_ODOM_TOPIC}"; then
        echo "话题 ${PX4_ODOM_TOPIC} 已发布！"
        echo "等待 $post_wait_delay 秒后退出..."
        sleep $post_wait_delay
        exit 0
    else
        elapsed=$(($(date +%s) - start_time))
        echo "等待中... 已等待 $elapsed 秒"
    fi
    sleep $check_interval
done

echo "错误：等待超时！话题 ${PX4_ODOM_TOPIC} 在 $timeout 秒内未发布"
exit 1
