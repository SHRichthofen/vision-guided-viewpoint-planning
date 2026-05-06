# View Goal Solver FOV 投影失败案例

日期：2026-05-05

## 1. 失败案例备份

当前 FOV 虚拟投影版本已备份：

```text
archive/5-5update/view_goal_solver.cpp.failure_case_fov_projection_2026-05-05
archive/5-5update/view_goal_solver.params.yaml.failure_case_fov_projection_2026-05-05
```

## 2. 观察到的问题

典型 debug：

```text
bottom_rim_fov_score = 1.0
bottom_rim_inside_count = 16 / 16
axis_error = 0.62 ~ 0.69 rad
lateral_error = 0.13 m
```

FOV 投影认为底部 rim 完整入画，但真实画面仍然是侧面观察，底面不可见或不完整。

## 3. 原因分析

FOV 虚拟投影回答的是：

```text
目标几何采样点投影后是否落在图像范围内
```

它不回答：

```text
底面是否被圆柱侧壁遮挡
相机是否沿圆柱轴线观察
管内底部是否真的可见
```

也就是说，FOV 把圆柱当成透明几何体处理。对于侧视姿态，底部圆环点可以投影到图像中央，但真实光路会被前侧管壁遮挡。

## 4. 结论

FOV 投影适合做：

```text
粗略入画检查
边界贴边诊断
debug 解释
```

不适合作为当前任务的核心观察质量目标。当前任务更应依赖几何主约束：

```text
axis_error 足够小
lateral_error 足够小
axial_standoff 位于合理区间
```

直观解释：

```text
如果相机轴向指向接近圆柱轴线，
相机/光轴离圆柱中轴足够近，
观察高度在合理范围内，
则位姿才具备看清管内底部的物理基础。
```

FOV 在这个几何前提下只保留为辅助项，防止目标明显出画。

## 5. 本次参数方向

恢复几何主约束：

```yaml
enforce_axis_max: true
axis_max_rad: 0.35
enforce_lateral_max: true
lateral_max_m: 0.035
```

降低 FOV 地位：

```yaml
min_fov_score: 0.50
min_bottom_rim_fov_score: 0.50
weight_fov: 1.0
weight_bottom_rim_fov: 1.0
```

增强几何 soft cost：

```yaml
weight_axis: 16.0
weight_lateral: 8.0
```

下一轮判断重点：

```text
constraint_failures 是否主要剩 axis_max / lateral_max
axis_error 是否能进入 0.35 rad 内
lateral_error 是否能进入 0.035 m 内
axial_standoff 是否保持在 0.055 ~ 0.17 m
```

