# ARX_ARM_WS — Vision-Guided Viewpoint Planning for the AgileX Arm

A ROS 2 Humble workspace for closed-loop, vision-guided inspection with an
[AgileX](https://github.com/agilexrobotics) manipulator and an Intel RealSense
D435i. A YOLO-based detector localises cylindrical targets, a solver searches
joint space for a configuration whose camera pose observes the target's opening,
and a MoveIt executor plans and runs the motion.

The distinguishing component is `view_goal_solver`: instead of emitting a single
Cartesian goal pose and hoping IK and planning succeed, it searches directly in
joint space, separates hard geometric feasibility from observation quality, and
hands MoveIt a *pool* of feasible candidates so that `plan()` itself becomes the
final acceptance gate.

---

## Pipeline

```
RealSense D435i
      │  (colour + depth)
      ▼
cylinder_detection ──────────────► /target_pose, /target_pose_debug
 (vision_detection)                        │
                                           ▼
                            vision_to_arm_transform
                             (vision_arm_control)
                                           │  camera frame → base_link,
                                           │  using the hand-eye calibration
                                           ▼
                                /cylinder_semantics_base
                                /target_id
                                           │
                                           ▼
                                 view_goal_solver  ──► /view_goal_solver_debug
                                    (control)
                                           │  /view_goal_joint_candidates
                                           │  (ranked pool of feasible q)
                                           ▼
                             vision_moveit_executor  ──► /vision_exec_status
                                    (control)
                                           │  MoveIt plan() + execute()
                                           ▼
                                   agx_arm_ctrl  ──► CAN ──► arm
```

The solver uses forward kinematics for scoring only. It does not call
`/check_state_validity` and does not plan a path; collision and trajectory
feasibility are delegated to the executor's MoveIt `plan()` call.

A legacy Cartesian path (`arm_pose_controller` → `/target_pose_stamped`) still
exists but is disabled in the recommended bringup.

---

## Repository layout

| Path | Contents |
| --- | --- |
| `src/control/` | C++ core: `view_goal_solver`, `vision_moveit_executor`, `tf_compat_broadcaster`, plus the top-level bringup launch files |
| `src/vision_detection/` | Python. YOLO cylinder detection on the RealSense stream; two-stage body + rim/hole detection and 3D pose estimation |
| `src/vision_arm_control/` | Python. Camera→`base_link` transformation, target selection, and the legacy pose controller |
| `src/hand_eye_calibration/` | C++/Python. OpenCV hand-eye calibration (chessboard and AprilTag variants) for the tool↔camera transform |
| `dependencies.repos` | Pinned upstream vendor packages, fetched into `src/` — not committed here |
| `scripts/setup_deps.sh` | One-shot fetch of those vendor packages, including the nested URDF submodule |
| `archive/` | Dated design notes, surveys, and code snapshots documenting how the solver evolved. Reference material, not part of the build |

`src/agx_arm_ros/` (providing `agx_arm_ctrl`, `agx_arm_description`,
`agx_arm_moveit`, `agx_arm_msgs`) is **not** stored in this repository. It is
fetched at a pinned revision — see [Setup](#setup).

---

## Requirements

- Ubuntu 22.04 with ROS 2 Humble
- MoveIt 2 (`ros-humble-moveit`)
- `python3-vcstool`, `python3-colcon-common-extensions`, `python3-rosdep`
- Intel RealSense SDK 2 and `pyrealsense2`
- `ultralytics`, `opencv-python`, `numpy`, `scipy`
- `ros-humble-apriltag-ros` (only for the AprilTag calibration flow)
- A CAN interface to the arm (`can0` by default)

## Setup

```bash
git clone https://github.com/starexplorer-robotics/ARX_ARM_WS.git
cd ARX_ARM_WS

# Fetch the upstream AgileX packages at their pinned revisions
./scripts/setup_deps.sh

# Resolve the remaining system dependencies
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

`scripts/setup_deps.sh` wraps `vcs import src < dependencies.repos` and then
pins the nested `agx_arm_urdf` submodule. That extra step matters: the revision
this workspace was validated against is newer than the one `agx_arm_ros`
records, and it changes joint limits that the view-goal solver treats as hard
constraints.

## Running

### Single-terminal bringup

Starts `agx_arm_ctrl`, MoveIt, RViz, `view_goal_solver`,
`vision_moveit_executor`, `vision_detection`, and `vision_to_arm_transform`:

```bash
source install/setup.bash

ros2 launch control vision_bottom_scan_bringup.launch.py \
  arm_type:=piper \
  follow:=true \
  enable_view_goal_solver:=true \
  enable_legacy_arm_pose_controller:=false \
  effector_type:=agx_gripper
```

Useful arguments: `can_port` (`can0`), `namespace`, `effector_type`,
`tcp_offset`, `calib_file`, `base_frame` (`base_link`), `effector_frame`
(`tcp_link`), `camera_optical_frame` (`camera_color_optical_frame`).

### Split terminals

Separating the nodes makes each one's log readable — useful when tuning the
solver.

```bash
# 1 — arm + MoveIt only
ros2 launch control run_agx_moveit_refactor.launch.py \
  arm_type:=piper follow:=true \
  enable_executor:=false enable_view_goal_solver:=false

# 2 — executor
ros2 run control vision_moveit_executor --ros-args \
  --params-file src/control/config/vision_moveit_executor.params.yaml \
  -r joint_states:=/feedback/joint_states

# 3 — view-goal solver
ros2 run control view_goal_solver --ros-args \
  --params-file src/control/config/view_goal_solver.params.yaml \
  -r joint_states:=/feedback/joint_states

# 4 — perception + transform
ros2 launch vision_arm_control vision_arm_integration_refactor.launch.py \
  enable_arm_pose_controller:=false
```

> **The `joint_states` remap is mandatory when running nodes standalone.**
> `view_goal_solver` and `vision_moveit_executor` each construct their own
> `MoveGroupInterface`, which subscribes to `joint_states` by default. Without
> `-r joint_states:=/feedback/joint_states` they never receive robot state and
> fail with `latest received state has time 0.000000`. The launch files apply
> this remap for you.

## Tuning the view-goal solver

`src/control/config/view_goal_solver.params.yaml` is the main tuning surface.
The parameters divide into three layers:

**Hard geometric constraints** — a candidate is feasible only if it satisfies
all of these, together with joint bounds:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `standoff_min_m` / `standoff_max_m` | `0.06` / `0.15` | Camera distance along the ideal viewing direction |
| `lateral_max_m` | `0.005` | Allowed offset from the target's axis line |
| `axis_max_rad` | `0.25` | Optical-axis vs. cylinder-axis misalignment |
| `gaze_max_rad` | `0.35` | Gaze error; disabled by default (`enforce_gaze_max: false`) |

**Feasibility-first search** (`enable_feasibility_first: true`) — while a
candidate is infeasible the search optimises only constraint violation plus a
small comfort term (`infeasible_comfort_weight`), so it cannot be pulled toward
a boundary it is simultaneously being pushed away from.

**Quality weights** — applied once a candidate is feasible:
`quality_weight_lateral_error` (`1e5`), `quality_weight_axis_error` (`10`),
`quality_weight_joint_limit` (`3`), `quality_weight_motion` (`2`),
`quality_weight_standoff_target` (`1`), `quality_weight_wrist_motion` (`0.1`).

The top `feasible_candidate_pool_size` (16) configurations are published on
`/view_goal_joint_candidates` for the executor to try in rank order.

The full derivation is in
[`archive/5-7update/view_goal_solver_algorithm_notes.md`](archive/5-7update/view_goal_solver_algorithm_notes.md),
with the design surveys that preceded it in `archive/5-6update/`.

## Hand-eye calibration

The camera→tool transform must be calibrated before the pipeline is meaningful.
`src/hand_eye_calibration/` supports both a chessboard and an AprilTag target:

```bash
ros2 launch hand_eye_calibration easy_handeye_agx_calibrate.launch.py   # collect + solve
ros2 launch hand_eye_calibration easy_handeye_agx_publish.launch.py     # publish the result
```

The result is written to `src/hand_eye_calibration/calib.yaml`, which the
bringup launch files read via the `calib_file` argument. See that package's
`README.md` and `AUTO_COLLECTION_GUIDE.md` for the collection procedure.

## License

Apache-2.0 — see [`LICENSE`](LICENSE). Upstream AgileX packages are fetched
separately and carry their own licenses.
