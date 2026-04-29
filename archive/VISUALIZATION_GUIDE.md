# 改进的可视化系统 - 使用说明

## 概述

现在的target_selector节点已经改进为**直接在相机实时画面上显示圆柱位姿**，而不是显示一个独立的列表窗口。

## 功能特性

### 1. **实时相机画面显示**
- 订阅RealSense相机的RGB图像 (`/camera/color/image_raw`)
- 在实时画面上叠加目标检测结果

### 2. **圆柱检测结果标注**
- ✅ **所有检测到的圆柱** - 显示为蓝色圆点 (Orange: `(255, 128, 0)`)
- ✅ **选中的圆柱** - 显示为绿色圆点 (Green: `(0, 255, 0)`)，带有详细信息

### 3. **位姿信息显示** (仅选中圆柱)
```
- 圆柱ID标签: "ID:0 [SEL]"
- 3D坐标轴: 蓝色箭头表示Z轴（法线方向）
- 深度信息: "Z: 0.123m"
- 位置信息: "Pos:(0.045, -0.032)"
```

### 4. **键盘交互控制**

| 按键 | 功能 |
|------|------|
| W / UP | 选择前一个圆柱 |
| S / DOWN | 选择后一个圆柱 |
| ENTER | 确认选择 |
| A | 切换自动/手动模式 |
| Q | 退出程序 |

### 5. **信息面板**
- **左上角**: 标题 + 检测到的圆柱数量
- **右上角**: 当前选中的圆柱ID
- **底部**: 控制提示 + 模式状态

## 代码修改总结

### 文件: `target_selector.py`

#### 新增导入
```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
```

#### 新增订阅
```python
# 订阅相机RGB图像
self.image_sub = self.create_subscription(
    Image,
    '/camera/color/image_raw',
    self.on_image_callback,
    10
)
```

#### 新增方法

1. **on_image_callback()** - 接收相机图像
```python
def on_image_callback(self, msg: Image):
    """处理相机图像"""
    try:
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except Exception as e:
        self.get_logger().warn(f"图像转换失败: {e}")
```

2. **project_point_to_image()** - 3D到2D投影
```python
def project_point_to_image(self, point_3d):
    """将3D点投影到图像平面"""
    z = point_3d.z
    if z <= 0:
        return None
    
    x_img = int(self.fx * point_3d.x / z + self.cx)
    y_img = int(self.fy * point_3d.y / z + self.cy)
    
    return (x_img, y_img)
```

3. **draw_cylinder_pose()** - 绘制圆柱信息
```python
def draw_cylinder_pose(self, frame, pose, cylinder_id, is_selected=False):
    """在图像上绘制圆柱和位姿"""
    # 投影中心点
    # 绘制圆点
    # 绘制ID标签
    # 绘制3D坐标轴 (仅选中)
```

4. **ui_loop()** - 完全重写为基于相机图像的可视化
```python
def ui_loop(self):
    """实时相机画面UI线程"""
    # 使用相机图像或黑色背景
    # 绘制所有检测圆柱
    # 绘制信息面板
    # 处理键盘输入
```

## 技术细节

### 坐标系投影
使用相机内参（RealSense D435i）进行3D→2D投影：
```
x_img = fx * X_3d / Z_3d + cx
y_img = fy * Y_3d / Z_3d + cy
```

默认参数:
- fx = 906.948 (水平焦距)
- fy = 905.906 (竖直焦距)  
- cx = 640.0 (图像中心X)
- cy = 360.0 (图像中心Y)

### 颜色编码
- **蓝色** `(255, 0, 0)` - Z轴/法线方向
- **绿色** `(0, 255, 0)` - 选中圆柱
- **Orange** `(255, 128, 0)` - 未选中圆柱
- **Yellow** `(0, 255, 255)` - 距离/位置信息

## 使用场景

1. **实时检测验证** - 看到相机实际检测到的结果
2. **位姿调试** - 观察圆柱的3D位置和方向是否正确
3. **交互选择** - 直观地在画面上选择目标圆柱
4. **系统集成** - 验证从检测到控制的完整数据流

## 运行命令

```bash
# 方式1: 使用启动脚本
./run_visualization.sh

# 方式2: 直接使用ros2 launch
ros2 launch vision_arm_control vision_arm_integration.launch.py
```

## 常见问题

### Q: 显示的圆点位置不对？
A: 检查相机内参是否正确。可能需要根据实际相机重新标定。

### Q: 看不到圆柱检测结果？
A: 
1. 确保vision_detection节点正常运行（检查log中是否有"检测到"信息）
2. 确保检测到的圆柱在相机视野内
3. 确认/vision/cylinders话题有正确发布

### Q: 窗口没有出现？
A: 
1. 确保在可以显示GUI的环境（有X11或类似）
2. 检查`enable_ui: true`参数是否设置
3. 查看log中是否有Qt字体警告（可忽略）

## 后续改进方向

- [ ] 支持鼠标点击选择圆柱
- [ ] 添加圆柱尺寸信息显示
- [ ] 显示置信度分值
- [ ] 支持保存截图/视频
- [ ] 添加更多的颜色/绘制选项配置

