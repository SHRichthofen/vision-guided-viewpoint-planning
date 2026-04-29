# 改进的可视化系统 - 快速参考

## 启动方式

```bash
# 进入工作区
cd ~/grad_proj/other_hands/implementation/arm_ws

# 停用conda（如果有的话）
conda deactivate

# 启动完整系统
./run_visualization.sh

# 或者直接用ros2
source install/setup.bash
ros2 launch vision_arm_control vision_arm_integration.launch.py
```

## 屏幕显示内容

**相机实时画面 + 检测结果标注**

- 左上：标题 + 圆柱数量
- 画面中心：所有检测到的圆柱 (蓝色圆点 + ID标签)
- 选中圆柱：绿色圆点 + 详细信息面板
  - ID标记
  - Z轴箭头（蓝色）
  - 深度值
  - XY位置
- 右上：当前选中圆柱ID
- 底部：控制提示 + 模式状态

## 键盘控制

| 按键 | 功能 |
|------|------|
| **W** / **↑** | 选择上一个圆柱 |
| **S** / **↓** | 选择下一个圆柱 |
| **ENTER** | 确认选择 |
| **A** | 切换自动/手动模式 |
| **Q** | 退出程序 |

## 工作流程

```
1. 启动系统 → 相机实时显示
   ↓
2. 看到检测到的圆柱（蓝色圆点）
   ↓
3. 使用W/S键切换选择
   ↓
4. 选中的圆柱变绿色，显示位姿信息
   ↓
5. 按ENTER确认选择
   ↓
6. 系统发送目标ID到arm_pose_controller
   ↓
7. 机械臂执行任务
```

## 实现原理

```
相机图像 (RGB)
    ↓
检测结果 (PoseArray)
    ↓
3D位置 + 法线方向
    ↓
投影到2D图像 (使用相机内参)
    ↓
绘制圆点 + 位姿信息
```

## 相机参数（RealSense D435i）

```python
fx = 906.948   # 水平焦距
fy = 905.906   # 竖直焦距
cx = 640.0     # 图像中心X
cy = 360.0     # 图像中心Y
```

如需调整，修改 `target_selector.py` 中的：
```python
self.fx = 906.948
self.fy = 905.906
self.cx = 640.0
self.cy = 360.0
```

## 故障排除

### 问题：看不到圆柱
- ✓ 检查vision_detection节点是否运行（看log中"检测到"信息）
- ✓ 检查圆柱是否在相机视野内
- ✓ 检查/vision/cylinders话题是否有发布

### 问题：窗口不显示或坐标不对
- ✓ 检查相机内参是否正确
- ✓ 检查enable_ui参数是否为true
- ✓ 可以忽略Qt字体警告

### 问题：程序闪退
- ✓ 检查是否有Python依赖缺失
- ✓ 确保conda已停用
- ✓ 查看terminal中的错误信息

## 相关文档

- 详细说明: `VISUALIZATION_GUIDE.md`
- 改进总结: `VISUALIZATION_UPDATE.md`
- 源代码: `src/vision_arm_control/vision_arm_control/target_selector.py`
- 启动脚本: `run_visualization.sh`

