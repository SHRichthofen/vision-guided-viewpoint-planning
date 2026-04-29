# 📚 手眼标定包 - 快速参考

## 🎯 包的目的

将**手眼标定**功能从 `control` 包中独立出来，形成一个自包含、易于维护和复用的模块。

## 📦 包内容一览

```
hand_eye_calibration/
│
├─ 📂 include/hand_eye_calibration/
│  └─ calibration_utils.hpp          [共享库接口]
│
├─ 📂 src/
│  ├─ calibration_utils.cpp          [核心算法实现]
│  ├─ hand_eye_calibration_node.cpp  [ROS2 标定节点]
│  └─ visual_servo_controller_node.cpp [ROS2 伺服节点]
│
├─ 📂 launch/
│  ├─ hand_eye_calibration.launch.py [启动标定]
│  └─ visual_servo_demo.launch.py    [启动伺服]
│
├─ 📂 config/
│  └─ hand_eye_calibration.yaml      [参数配置]
│
├─ CMakeLists.txt                    [构建配置]
├─ package.xml                       [包声明]
└─ README.md                         [详细文档]
```

## 🚀 三种快速使用方式

### 🔷 方式 1: 作为独立标定工具

```bash
# 编译
colcon build --packages-select hand_eye_calibration

# 启动标定
ros2 launch hand_eye_calibration hand_eye_calibration.launch.py
```

### 🔷 方式 2: 在应用中使用库

```cpp
#include "hand_eye_calibration/calibration_utils.hpp"
using namespace hand_eye_calibration;

// 直接调用库函数
CheckerboardDetector detector(fx, fy, cx, cy);
TsaiLenzSolver solver;
// ...
```

### 🔷 方式 3: 视觉伺服演示

```bash
# 启动视觉伺服
ros2 launch hand_eye_calibration visual_servo_demo.launch.py target_color:=red
```

## 📊 核心类和函数速查

| 类/函数 | 文件 | 功能 |
|--------|------|------|
| `CheckerboardDetector` | `calibration_utils.hpp` | 检测棋盘并估计位姿 |
| `TsaiLenzSolver` | `calibration_utils.hpp` | 求解手眼标定 |
| `CalibrationSample` | `calibration_utils.hpp` | 标定样本数据结构 |
| `CalibrationResult` | `calibration_utils.hpp` | 标定结果数据结构 |
| `io::saveCalibrationResultYAML()` | `calibration_utils.hpp` | 保存标定结果 |
| `io::loadCalibrationResultYAML()` | `calibration_utils.hpp` | 加载标定结果 |

## 🔄 工作流程概览

```
【数据采集】
  相机 → 棋盘检测（PnP）
  机械臂 → 末端位姿
         ↓
【同步采样】
  收集样本对：(T_base_tool, T_cam_board)
         ↓
【标定计算】
  Tsai-Lenz 求解
  → X：末端到相机的变换
         ↓
【验证】
  计算重投影误差
  保存结果到 YAML
         ↓
【应用】
  用于视觉伺服
  → 目标物体检测
  → 坐标变换
  → 发送到机械臂
```

## 🎮 快速命令参考

### 编译和安装
```bash
# 编译单个包
colcon build --packages-select hand_eye_calibration

# 编译所有（包括本包）
colcon build

# 清理后重新编译
colcon build --packages-select hand_eye_calibration --cmake-clean-first
```

### 运行节点
```bash
# 直接运行（使用默认参数）
ros2 run hand_eye_calibration hand_eye_calibration_node

# 运行并指定参数
ros2 run hand_eye_calibration hand_eye_calibration_node \
  --ros-args -p min_samples:=15

# 使用 launch 文件
ros2 launch hand_eye_calibration hand_eye_calibration.launch.py
```

### 查看输出
```bash
# 查看节点输出
ros2 node list | grep calibration

# 查看话题
ros2 topic list | grep calibration

# 监听话题
ros2 topic echo /calibration_result
```

## 💾 配置文件快速编辑

### `hand_eye_calibration.yaml`

```yaml
# 棋盘尺寸
checkerboard_rows: 6      # ← 改这里改棋盘行数
checkerboard_cols: 9      # ← 改这里改棋盘列数
square_size_mm: 30.0      # ← 改这里改格子大小

# 相机参数（D435i）
camera_fx: 906.948        # ← 对应不同相机改焦距
camera_fy: 905.906
camera_cx: 648.379        # ← 改主点位置
camera_cy: 383.873

# 标定参数
min_samples: 10           # ← 改这里改最少采样数
output_file: ...          # ← 改这里改输出路径
```

## 🧩 与其他包的关系

```
control (机械臂控制)
    ↓ 发布 /tool_pose
hand_eye_calibration (标定)
    ├─ 读取 /tool_pose
    ├─ 读取相机图像
    └─ 输出 /calibration_result
    ↓ 用于
应用程序 (视觉伺服等)
```

## 📈 性能指标

| 指标 | 值 | 备注 |
|-----|-----|------|
| 棋盘检测速度 | ~30ms | 1280×720 分辨率 |
| 标定计算时间 | ~100ms | 12 个样本 |
| 重投影误差（优秀）| < 1cm | 可接受范围 |
| 最少样本数 | 10 个 | 建议 15+ |
| 采样同步频率 | 10Hz | 可调整 |

## 🔒 质量指标检查清单

- [ ] 棋盘能被可靠检测（>95% 成功率）
- [ ] 采集 ≥10 个均匀分布的样本
- [ ] 重投影误差 < 5cm
- [ ] 标定结果已保存到文件
- [ ] 视觉伺服能正确识别目标
- [ ] 坐标变换计算正确

## 🐛 常见问题速查

| 问题 | 快速解决 |
|-----|----------|
| 编译错误 | `rosdep install --from-paths src -i -y` |
| 找不到话题 | 检查 `remappings` 和话题名 |
| 棋盘检测失败 | 改善光线、调整角度 |
| 误差太大 | 增加样本数、覆盖更多位置 |
| 节点无法启动 | 检查依赖是否安装 |

## 📖 推荐阅读顺序

1. **README.md** - 包总体说明
2. **calibration_utils.hpp** - 库接口和注释
3. **hand_eye_calibration_node.cpp** - 如何使用库
4. **INTEGRATION_GUIDE.md** - 与系统集成
5. **D435i_quick_reference.md** - 相机参数详解

## 🎓 学习要点

- ✅ 理解 Tsai-Lenz 标定算法
- ✅ 掌握 PnP 位姿估计
- ✅ 学会模块化 ROS2 包设计
- ✅ 理解坐标系变换
- ✅ 实际运用手眼标定进行视觉伺服

## 📞 文档导航

```
项目根目录
├── README.md                      ← 总体项目说明
├── INTEGRATION_GUIDE.md           ← 本文件指向
├── hand_eye_calibration/          ← 标定包
│   └── README.md                  ← 包级文档
└── D435i_*                        ← 相机参数文档
```

---

**提示**：每个源文件顶部都有详细的中文注释，说明函数功能和参数含义。❄️
