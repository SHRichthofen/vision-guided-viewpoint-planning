# 2026-04-29 Survey: Stable Reachable View Pose Generation

本文只记录当前关于“从管口中心与轴向生成稳定、可达观察位姿”的 survey 结论。reachability map 的具体采集流程不写入本文档，另行讨论。

## 1. 当前问题抽象

当前控制层把视觉输出的 `top_center_base + axis_base` 转成一个唯一观察位姿：

```text
camera_position = center - view_axis * camera_standoff_m
look_at_point = center
```

这会把任务硬化为：

- 相机必须位于管口轴线上
- 相机光轴必须严格看向管口中心
- standoff 是固定值
- roll 由单一 reference axis 决定
- MoveIt 只接收一个 `/target_pose_stamped`

实机现象表明，对侧装相机和近竖直管口，这类单一轴向位姿很容易落入机械臂不可达、腕部姿态极端、或规划空间很窄的区域。

同时，MoveIt 中的 `goal_position_tolerance` 与 `goal_orientation_tolerance` 只能放宽末端目标接受误差。它们不会理解“管口仍需在 FOV 内”“相机仍需看到管底”“光轴需与管轴近似平行”等视觉任务约束。因此，不能把 MoveIt tolerance 当作观察约束本身。

## 2. 成熟系统的共同范式

抓取与观察位姿生成的成熟系统很少只生成一个目标 pose。更常见范式是：

1. 从对象几何或任务语义生成一批候选位姿或一个连续任务区域。
2. 用任务质量指标评分，例如抓取质量、视角质量、碰撞风险、管口覆盖率。
3. 用 IK、碰撞检查、规划器过滤不可达候选。
4. 必要时用连续优化或闭环伺服把目标从粗略可达位置收敛到任务更优位置。

对当前任务，这意味着目标不应是“唯一轴向俯视 pose”，而应是“满足视觉任务约束的可达区域/可达候选集合”。

## 3. MoveIt Grasps / MTC

MoveIt Grasps 的结构是 grasp generator、grasp scorer、grasp filter、grasp planner。其 grasp filter 会用 IK 检查候选是否可达，并剔除不可行动作。它还支持柱体等几何对象的抓取生成。

参考：

- MoveIt Grasps tutorial: https://moveit.github.io/moveit_tutorials/doc/moveit_grasps/moveit_grasps_tutorial.html
- MoveIt Grasps GitHub: https://github.com/moveit/moveit_grasps

MoveIt Task Constructor 更进一步，把复杂任务拆成 generator、wrapper、connector 等 stage。`GeneratePose`/`GenerateGraspPose` 用于产生候选，`ComputeIK` wrapper 用于对候选做 IK 求解与过滤。

参考：

- MTC concepts: https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor/moveit_task_constructor.html
- MTC generating stages: https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor/generating_stages.html
- MTC ComputeIK wrapper: https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor/wrappers.html

对当前项目的启发：

- 可以保留视觉层只发布 `center + axis` 的语义接口。
- 在控制/规划层增加 view pose generator。
- 对每个观察候选执行 IK/规划过滤，而不是直接发布唯一 `/target_pose_stamped`。
- 但如果用户已经验证离散选点仍找不到可行点，则需要继续升级到连续约束规划、优化式 IK 或可达性先验，而不是简单增加采样密度。

## 4. MoveIt OMPL Constrained Planning

MoveIt 的 OMPL constrained planning 针对的是小体积或零体积的 Cartesian 约束区域。官方文档指出，这类 planner 对普通 rejection sampling 难以命中的狭窄约束区域更有价值，例如把末端约束在平面或直线上。

参考：

- MoveIt OMPL constrained planning: https://moveit.picknik.ai/main/doc/how_to_guides/using_ompl_constrained_planning/ompl_constrained_planning.html
- MoveIt constrained planning background: https://moveit.ai/moveit/2020/09/10/ompl-constrained-planning-gsoc.html

对当前项目的启发：

- 当前“相机需看进管口”的可行域可能很窄，离散候选容易错过。
- 比起把很多离散 pose 交给普通 MoveIt 规划，更合理的是把视觉要求表达成约束区域，让 planner 在约束流形附近搜索。
- 需要注意 MoveIt 当前 constrained planning 接口更偏 path constraint，并不自动表达 FOV、管底可见性等任务语义；这些仍需转换成位置/方向/锥体等约束或外部评分。

## 5. Task Space Regions

Task Space Regions (TSR) 是经典的 pose-constrained manipulation 表达方式。它把目标从单一位姿扩展为某些自由度有范围的任务空间区域，适合表达“某些方向必须满足，某些自由度可放松”的操作任务。

参考：

- Task Space Regions paper: https://publications.ri.cmu.edu/task-space-regions-a-framework-for-pose-constrained-manipulation-planning

对当前项目的启发：

- 管口观察可以表达为 TSR 风格区域：
  - 相机位置在管口外侧一段 standoff 范围内
  - 相机光轴与管轴夹角小于阈值
  - 绕光轴 roll 可自由或半自由
  - 允许少量横向偏移，但偏移必须保持管口/管底可见
- 这比固定 `center - axis * standoff` 更符合任务本质。

## 6. Tesseract / TrajOpt / Descartes

Tesseract 提供多个规划器，包括 OMPL、TrajOpt、TrajOptIfopt、Descartes。官方文档把 TrajOpt 描述为优化式规划器，适合 Cartesian 路径、平滑与约束问题；Descartes 则适合 dense Cartesian toolpath。

参考：

- Tesseract motion planning docs: https://tesseract-robotics.github.io/tesseract_nanobind/user-guide/planning/
- TrajOpt GitHub: https://github.com/tesseract-robotics/trajopt

对当前项目的启发：

- 如果离散候选失败，应考虑优化式 IK/轨迹优化，把观察质量、碰撞距离、关节限位余量、末端姿态偏差放进 cost/constraint。
- 对“可以略偏轴，但仍要看见管底”的问题，软约束成本函数比硬枚举候选更自然。
- 工业场景中的半约束工具路径与本任务相似：工具轴/视线方向重要，roll 或少量位移可以放松。

## 7. Drake Gaze Constraint

Drake 的 inverse kinematics 提供 `AddGazeTargetConstraint`，可以约束目标点落入从相机源点出发的视锥内。其文档明确把用途描述为相机 gaze at target。

参考：

- Drake C++ IK docs: https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_inverse_kinematics.html
- Drake Python IK docs: https://drake.mit.edu/pydrake/pydrake.multibody.inverse_kinematics.html

对当前项目的启发：

- 当前任务不一定要强制相机光轴精确穿过管口中心。
- 可以用 gaze cone 约束表达：
  - 管口中心在视锥内
  - 管底代理点在更窄视锥内
  - 管口 rim 若干采样点大部分在 FOV 内
- 这比 `look_at_point=center` 更贴近“相机仍能看到圆柱管口大部分位置，并观察到管底”的目标。

## 8. IK Solver: TRAC-IK / pick_ik

如果当前 MoveIt 使用默认 KDL IK，近关节限位、腕部姿态极端、窄可行域下可能更容易失败。TRAC-IK 提供比 KDL 更能处理 joint limit 的 IK 求解；pick_ik 是 MoveIt 2 兼容 IK solver，包含局部优化和全局优化，并支持额外 cost functions。

参考：

- TRAC-IK ROS docs: https://docs.ros.org/en/rolling/p/trac_ik/
- TRAC-IK MoveIt tutorial: https://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/trac_ik/trac_ik_tutorial.html
- pick_ik docs: https://docs.ros.org/en/rolling/p/pick_ik/
- pick_ik MoveIt guide: https://moveit.picknik.ai/main/doc/how_to_guides/pick_ik/pick_ik_tutorial.html

对当前项目的启发：

- 在改大规划架构前，值得确认当前 IK plugin。
- 若仍是 KDL，可优先评估 TRAC-IK 或 pick_ik。
- pick_ik 的 cost function 思路适合加入“远离关节限位”“接近当前姿态”“偏好某类腕部 roll”等目标。

## 9. MoveIt Servo / 闭环视觉伺服

MoveIt Servo 支持末端速度、关节速度或目标 pose 的实时控制，并具有奇异性处理与碰撞检查能力。官方文档也提到它可用于 visual servoing 或 closed-loop position control。

参考：

- MoveIt Servo rolling docs: https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html
- MoveIt Servo Noetic tutorial: https://moveit.github.io/moveit_tutorials/doc/realtime_servo/realtime_servo_tutorial.html

对当前项目的启发：

- 不必一次规划到最终最佳观察位姿。
- 可以先规划到一个保守、安全、粗略可见的相机位置。
- 再用图像中管口中心、rim 覆盖、管底代理点偏差做小步闭环修正。
- 这能降低一次性全局规划对精确目标 pose 的依赖。

## 10. 当前推荐技术方向

如果已经确认简单离散选点仍无法找到成功规划点，后续应避免只增加采样密度。更推荐的路线是：

1. 确认并增强 IK solver，例如 TRAC-IK 或 pick_ik。
2. 把观察目标建模为连续约束区域，而不是单一 pose。
3. 将管口中心、管底代理点、rim 覆盖转换成 gaze/FOV/方向约束或评分函数。
4. 用 constrained planning、优化式 IK/轨迹优化、或可达性先验引导搜索。
5. 规划只负责到达安全可见区，最终视角质量通过 TF + 图像闭环验证或微调。

一句话总结：

```text
当前任务更像 gaze-constrained manipulation，而不是普通 pose-goal planning。
```

