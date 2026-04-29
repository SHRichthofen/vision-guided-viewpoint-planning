# 2026-04-28 论文与项目检索记录

## 1. 检索目标

本轮检索围绕以下问题展开：

- 是否存在与当前项目类似的成熟方案：
  - `D435i / RGB 相机`
  - `2D 几何图像`
  - `圆口 / 圆柱`
  - `3D 位姿估计`
- 单圆、椭圆、侧边直线、环境约束之间，已有方法如何处理二义性和退化？
- 在圆口精确提取上，有哪些成熟的高质量 ellipse/circle detection 方法值得迁移？

---

## 2. 总结结论

### 2.1 没有找到与当前项目完全同构的成熟 ROS2 开源项目

没有找到一个“D435i + 单目圆柱口 + 纯 2D 几何 + ROS2”完全同构、可直接照搬的成熟仓库。

但论文和成熟项目在几个核心问题上的结论非常一致：

- 单圆位姿解天然存在双解或退化。
- 椭圆中心一般不等于真实圆心投影。
- 高质量几何提取应先做 arc/segment 筛选，再做拟合与验证。
- 已知圆尺寸后，可以把单圆问题变成有尺度候选解问题，但仍需要额外约束选真解。

### 2.2 当前项目最该吸收的经验

- 从“全边缘拟合”转向“弧段支持拟合”。
- 从“单次拟合直接给解”转向“候选解 + 评分 + 消歧义”。
- 正视退化应单独处理，不应沿用普通椭圆姿态公式。
- tube/rim depth 在当前目标材质条件下风险高，应退出主链路。

---

## 3. 与单圆位姿相关的论文

### 3.1 Single view based pose estimation from circle or parallel lines

链接：

- https://www.sciencedirect.com/science/article/pii/S0167865508000433

关键信息：

- 单圆或平行线都可以参与单视图位姿估计。
- 单圆位姿求解不是简单的“拟合出椭圆就完事”，而是一个本身有歧义的几何问题。
- 加入平行线/额外几何约束可明显提升可解性。

对当前项目的启发：

- 已知圆口内径以后，单圆可以提供有尺度 pose candidates。
- 若可见圆柱侧边母线，应优先利用它们做消歧义。

### 3.2 Vision Based Position Control for MAVs Using One Single Circular Landmark

链接：

- https://www.research-collection.ethz.ch/bitstreams/ac426ab6-530a-40aa-9dbc-6ded3e39279d/download

关键信息：

- 单个圆形 landmark 可以用于位姿估计和控制。
- 但单圆仍有退化和双解问题，工程系统必须结合附加约束。

对当前项目的启发：

- “只靠单椭圆硬解完整姿态”风险很高。
- 更适合输出 `center + axis`，并对退化场景显式降级。

### 3.3 Vision pose estimation from planar dual circles in a single image

链接：

- https://www.sciencedirect.com/science/article/pii/S0030402616001716

关键信息：

- 双圆/同心圆能明显改善单圆位姿解的歧义与稳定性。

对当前项目的启发：

- 当前环境没有稳定可用的第二个共面圆，因此不能直接照搬。
- 但这篇工作强化了一个判断：
  - 如果没有第二个圆，就必须寻找别的外部约束。

### 3.4 General fusion frame of circles and points in vision pose estimation

链接：

- https://www.sciencedirect.com/science/article/pii/S0030402617311932

关键信息：

- circles 可以与其他几何特征联合使用，而不是单独使用。

对当前项目的启发：

- 圆口之外的侧边直线、轮廓主轴、场景几何都可以成为弱约束。
- 但外壳尺寸未知，因此不能把外壳几何当硬尺寸模型。

---

## 4. 与“投影圆心不等于椭圆中心”相关的论文

### 4.1 The projected circle centres and polar line for camera self-calibration

链接：

- https://www.sciencedirect.com/science/article/abs/pii/S0030402615005082

关键信息：

- 透视投影下，真实圆心的图像投影一般不与拟合椭圆中心重合。
- pole-polar 关系可以用于推导投影圆心的位置。

对当前项目的启发：

- 当前 `fitEllipse` 中心不能继续直接当作圆口中心投影。
- 若能从侧边母线得到圆口平面的无穷远线，就应改用极线/极点方式修正圆心。

---

## 5. 与高质量圆/椭圆提取相关的项目和论文

### 5.1 Arc-support Line Segments Revisited: An Efficient and High-quality Ellipse Detection

论文：

- https://pubmed.ncbi.nlm.nih.gov/31425075/

仓库：

- https://github.com/AlanLuSun/High-quality-ellipse-detection

README：

- https://github.com/AlanLuSun/High-quality-ellipse-detection/blob/master/README.md

主入口：

- https://github.com/AlanLuSun/High-quality-ellipse-detection/blob/master/ellipseDetectionByArcSupportLSs.m

候选生成核心：

- https://github.com/AlanLuSun/High-quality-ellipse-detection/blob/master/generateEllipseCandidates.cpp

关键信息：

- 核心思想不是“对所有边缘点直接做椭圆拟合”，而是先找到 arc-support line segments。
- 利用 polarity、support inlier ratio、angular coverage 等几何验证，筛出高质量椭圆。
- 候选由局部显著弧组和全局配对弧组两条路生成，再聚类、去重、refit。

对当前项目的启发：

- 这是当前 rim 提取部分最值得借鉴的方法来源。
- 适合把你当前 `mask -> Canny -> convexHull -> fitEllipse` 改成候选生成 + 严格验证。

### 5.2 Circle detection by arc-support line segments

链接：

- https://alanlusun.github.io/files/ICIP%202017-Circle%20detection.pdf

关键信息：

- 同一作者更早提出了 circle detection 的 arc-support 路线。
- 对不完整、遮挡、模糊和过曝光场景更鲁棒。

对当前项目的启发：

- 正视场景下，管口更接近圆而非扁椭圆。
- 这时不应硬走普通 ellipse-only 流程，而应切到 circle mode。

### 5.3 ELSD / 其他高质量 ellipse detector

ELSD 仓库：

- https://github.com/viorik/ELSD

关键信息：

- 也是成熟的 ellipse detection 路线。

对当前项目的启发：

- 说明“高质量椭圆提取”本身就是独立问题。
- 当前项目在这一步确实不该继续停留在 `convexHull + fitEllipse`。

---

## 6. 与粗检测 + 精修相关的项目

### 6.1 MarkerPose

论文：

- https://openaccess.thecvf.com/content/CVPR2021W/LXCV/html/Meza_MarkerPose_Robust_Real-Time_Planar_Target_Tracking_for_Accurate_Stereo_Pose_CVPRW_2021_paper.html

仓库：

- https://github.com/jhacsonmeza/MarkerPose

关键信息：

- 先做粗检测，再做 patch-level 精修和几何求解。

对当前项目的启发：

- YOLO 负责 ROI 是合理的。
- 真正的几何精度应该来自 ROI 内的高质量局部提取，而不是检测框本身。

### 6.2 Deep ChArUco

论文：

- https://openaccess.thecvf.com/content_CVPR_2019/html/Hu_Deep_ChArUco_Dark_ChArUco_Marker_Pose_Estimation_CVPR_2019_paper.html

关键信息：

- 也是“粗检测 + 精确局部几何”的思路。

对当前项目的启发：

- 进一步支持把重点放在 rim ROI 内的高精度几何提取上。

---

## 7. 与 D435i / 深度使用方式相关的资料

### 7.1 D435i 官方规格

链接：

- https://www.intelrealsense.com/depth-camera-d435i

关键信息：

- D435i 是主动双目深度，相机结构和 RGB 通道特性决定了它并不天然适合所有小白色光滑目标。

对当前项目的启发：

- 深度不是不能用，但在当前 tube/rim 场景下，错误先验风险很高。

### 7.2 RealSense post-processing filters

链接：

- https://dev.realsenseai.com/docs/post-processing-filters/

关键信息：

- temporal filter 更适合静态或近静态场景，错误情况下可能带来 smear。

对当前项目的启发：

- 进一步支持“不要用 tube/rim depth 当主先验”。

### 7.3 realsense-ros

链接：

- https://github.com/realsenseai/realsense-ros

说明：

- 主要作为官方 ROS 接入参考，并不直接解决当前圆柱圆口几何问题。

### 7.4 ROS2 ArUco pose estimation with Realsense D435

链接：

- https://github.com/AIRLab-POLIMI/ros2-aruco-pose-estimation

说明：

- 可以作为“D435 系列 + ROS2 + 已知几何标记”的实现参考。
- 但它依赖的是 marker 角点几何，不适用于当前无 marker 圆口。

---

## 8. 对当前项目的最终转化结论

基于以上检索，当前项目最合理的方向已经比较明确：

1. 主链路改为纯 RGB。
2. 已知内径进入单圆 pose candidate 求解。
3. tube/rim depth 不进入主估计。
4. 圆口提取必须升级为 arc-support inspired candidate pipeline。
5. side lines 是最优先的额外几何约束来源。
6. 正视退化必须单独做 frontal / circle mode。
7. 环境平面只保留为未来弱辅助，不进入第一轮主链路。

---

## 9. 与当前代码最直接相关的结论

当前最需要重构的位置是：

- `src/vision_detection/vision_detection/pose_estimator.py`

当前最需要替换的旧思路是：

- `mask -> edges -> convexHull -> fitEllipse -> 直接解 pose`

当前最建议的新思路是：

- `rim boundary band -> arc-support segments -> candidate circle/ellipse -> score/refit -> pose candidates -> side-line disambiguation / frontal fallback`

这份检索记录可作为后续实现和继续检索的基础文档。
