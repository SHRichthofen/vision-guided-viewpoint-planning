#!/bin/bash
# 启动改进的可视化系统
# 在相机画面上显示圆柱位姿

set -e

cd "$(dirname "$0")"
source install/setup.bash

echo "=========================================="
echo "启动改进的可视化系统"
echo "=========================================="
echo ""
echo "该系统将在实时相机画面上显示："
echo "  1. 所有检测到的圆柱位置（蓝色圆点）"
echo "  2. 选中圆柱的详细位姿信息（绿色圆点，带坐标轴）"
echo "  3. 实时键盘交互界面"
echo ""
echo "控制方式："
echo "  W/S 或 UP/DOWN 箭头 - 在圆柱间切换"
echo "  ENTER - 确认选择"
echo "  A - 切换自动/手动模式"
echo "  Q - 退出"
echo ""
echo "=========================================="
echo ""

timeout 120 ros2 launch vision_arm_control vision_arm_integration.launch.py

