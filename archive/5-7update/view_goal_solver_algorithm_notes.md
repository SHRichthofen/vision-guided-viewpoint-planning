# View Goal Solver 5-7 Update Notes

Date: 2026-05-07

This note records the current relatively stable version of the view-goal solver for later paper writing. The matching parameter snapshot is `archive/5-7update/view_goal_solver.params.yaml`, which is identical to the active `src/control/config/view_goal_solver.params.yaml` at the time of this note.

## 1. Motivation

The task is to generate executable camera viewpoints for observing a detected cylindrical target. The perception module provides a target center and cylinder axis in the robot base frame. The solver must choose an arm joint configuration whose forward kinematics places the camera at a useful observation pose while remaining compatible with MoveIt planning.

Several earlier versions mixed hard constraints, observation quality terms, and feasibility heuristics inside one objective. That made tuning unstable: a candidate could be simultaneously treated as violating a boundary and being softly attracted to that same boundary. The current version separates the problem into three roles:

1. Hard geometry and joint-bounds feasibility.
2. Joint-space search that reaches feasibility before optimizing quality.
3. Candidate-pool publication so the executor can use MoveIt `plan()` as the final planning gate.

The solver itself does not call `/check_state_validity` and does not plan a path. It uses forward kinematics for scoring and delegates final path feasibility to the executor.

## 2. Inputs and Coordinate Definitions

For each selected target, the solver receives:

- `center_base`: the cylinder top-center position in the robot base frame.
- `axis_base`: the raw cylinder axis in the robot base frame.
- Current arm state `q_start` from MoveIt.
- A fixed transform from the end-effector link to the optical camera frame.

The raw cylinder axis is oriented into a working `view_axis_base`. If the current camera position is available, the axis sign is chosen so the view direction is consistent with the camera side. Otherwise `axis_into_tube_sign` is used as a fallback.

For a trial joint vector `q`, the solver computes:

- `p_cam`: camera optical-frame origin in base frame.
- `z_cam`: camera optical-frame +Z axis in base frame.
- `d = -view_axis_base`: ideal camera-side direction from the target center toward the camera.
- `center_to_camera = p_cam - center_base`.

The main observation metrics are:

```text
axial_standoff = dot(center_to_camera, d)
lateral_error  = norm(center_to_camera - axial_standoff * d)
axis_error     = angle(z_cam, view_axis_base)
gaze_error     = angle(z_cam, normalize(center_base + depth_proxy_m * view_axis_base - p_cam))
```

`axis_error` measures optical-axis alignment with the detected tube axis. `lateral_error` measures distance from the target axis line. `axial_standoff` measures how far the camera is from the top center along the ideal camera-side direction.

## 3. Hard Feasibility Layer

The hard feasibility layer represents publish-level geometry and joint-bounds acceptability. The main constraints are:

```text
standoff_min_m <= axial_standoff <= standoff_max_m
lateral_error <= lateral_max_m
axis_error <= axis_max_rad
joint_bounds_valid == true
```

`gaze_error <= gaze_max_rad` remains available, but the active configuration keeps `enforce_gaze_max: false` because axis alignment is the primary orientation metric.

For search diagnostics, every hard constraint is converted into a normalized hinge violation:

```text
axis_violation_norm =
  max(0, axis_error - axis_max_rad) / max(0.05, axis_max_rad)

lateral_violation_norm =
  max(0, lateral_error - lateral_max_m) / max(0.005, lateral_max_m)

standoff_min_violation_norm =
  max(0, standoff_min_m - axial_standoff) / max(0.03, standoff_max_m - standoff_min_m)

standoff_max_violation_norm =
  max(0, axial_standoff - standoff_max_m) / max(0.03, standoff_max_m - standoff_min_m)
```

The scalar hard violation is:

```text
hard_violation_total =
  axis_violation_norm^2
  + lateral_violation_norm^2
  + standoff_min_violation_norm^2
  + standoff_max_violation_norm^2
  + gaze_violation_norm^2
  + joint_bounds_violation_norm^2
```

The active feasible gate uses small engineering tolerances:

```text
constraint_tolerance_m:   0.0005
constraint_tolerance_rad: 0.005
```

A separate raw feasibility check is also retained. Raw feasible candidates satisfy the same hard constraints without tolerance. Candidate sorting gives raw feasible candidates priority over tolerance-only candidates. This keeps tolerance as a numerical robustness mechanism rather than making it the preferred operating boundary.

## 4. Quality and Ranking Layer

Once a candidate is feasible, the solver ranks it with a single quality score:

```text
rank_score = execution_proxy_score + observation_score
quality_score = rank_score
```

The execution proxy is a joint-space surrogate for MoveIt-friendliness:

```text
execution_proxy_score =
  quality_weight_motion       * motion_cost
  + quality_weight_wrist_motion * wrist_motion_cost
  + quality_weight_joint_limit  * joint_limit_cost
```

The active configuration uses:

```text
quality_weight_motion:       2.0
quality_weight_wrist_motion: 0.1
quality_weight_joint_limit:  3.0
joint_limit_margin_threshold_rad: 0.2
```

The observation score uses zero-residual objectives for axis and lateral placement, rather than preference thresholds:

```text
axis_cost            = axis_error^2
lateral_cost         = lateral_error^2
standoff_target_cost = (axial_standoff - standoff_desired_m)^2

observation_score =
  quality_weight_axis_error      * axis_cost
  + quality_weight_lateral_error * lateral_cost
  + quality_weight_standoff_target * standoff_target_cost
```

The active configuration uses:

```text
quality_weight_axis_error:      10.0
quality_weight_lateral_error:   100000.0
quality_weight_standoff_target: 1.0
```

The lateral weight is intentionally large because `lateral_error^2` is measured in square meters. For example, a 5 mm lateral error contributes only `2.5e-5` before weighting.

## 5. Feasibility-First Coordinate Descent

The solver uses deterministic multi-start coordinate descent in joint space:

1. Seed 0 is the current joint state.
2. Seed 1 is the center of bounded joint ranges.
3. Remaining seeds are deterministic sinusoidal perturbations around the current state.

The active configuration uses:

```text
multi_start_count:        24
max_iterations_per_seed:  300
initial_step_rad:         0.14
min_step_rad:             0.0005
step_shrink:              0.5
seed_perturbation_rad:    0.35
```

For each seed, the solver tries plus/minus coordinate moves on each joint, clamps trial joints to bounds, evaluates the FK-based score, and accepts the better candidate according to a feasibility-first rule:

- Before reaching feasibility, a candidate is accepted if it reduces `hard_violation_total`.
- If hard-violation progress is numerically tied, `feasibility_merit` uses the execution proxy as a small tie-breaker:

```text
feasibility_merit =
  hard_violation_total + infeasible_comfort_weight * execution_proxy_score
```

- After reaching feasibility, candidates are compared by:
  1. raw feasibility before tolerance-only feasibility;
  2. lower `rank_score` within the same feasibility class.

This behavior is intended to avoid an observation-first failure mode in which the solver finds a visually good but difficult-to-plan or joint-limit-heavy goal before satisfying the basic geometric conditions.

## 6. Candidate Pool and MoveIt Execution

The solver publishes a sorted pool of feasible joint candidates on:

```text
/view_goal_joint_candidates
```

Message type:

```text
trajectory_msgs/msg/JointTrajectory
```

The message contains:

- `joint_names`: planning-group joint names.
- one `JointTrajectoryPoint` per candidate.
- each point's `positions`: candidate joint vector.

The active pool size is:

```text
feasible_candidate_pool_size: 16
```

The legacy single-joint-target publisher is disabled:

```text
publish_joint_candidate_pool: true
publish_legacy_joint_target: false
```

The executor subscribes to the candidate pool and tries each candidate in order:

1. Clear pose targets and path constraints.
2. Set start state to current state.
3. Set the joint target with `setJointValueTarget()`.
4. Call MoveIt `plan()`.
5. Execute the first candidate whose plan succeeds.
6. If all candidates fail, report `all_candidates_plan_failed` on `/vision_exec_status`.

This design keeps the solver lightweight and lets MoveIt remain the final arbiter of path feasibility.

## 7. Debug Fields and Interpretation

The solver publishes a JSON debug message on:

```text
/view_goal_solver_debug
```

Important fields:

- `has_feasible_candidate`: whether any candidate satisfied the tolerance-aware gate.
- `candidate_pool_size`: number of retained feasible candidates.
- `hard_violation_total`: normalized raw hard-boundary violation.
- `raw_constraints_satisfied`: whether the selected candidate satisfies hard constraints without tolerance.
- `constraint_failures`: raw hard-boundary failures, even if the tolerance-aware gate accepts the candidate.
- `execution_proxy_score`: joint-space planning-friendliness proxy.
- `observation_score`: axis/lateral/standoff quality score.
- `rank_score`: final feasible-candidate ordering score.
- `axis_error`: optical axis error in radians.
- `lateral_error`: camera offset from the cylinder axis in meters.
- `axial_standoff`: camera distance from top center along the ideal camera-side direction.
- `joint_limit_margin` and `joint_limit_cost`: distance to joint limits and its soft quality penalty.

Typical interpretation:

- `constraints_satisfied=true` with `raw_constraints_satisfied=false` means the solution is in the tolerance band.
- Large `observation_score` means the pose is geometrically poor even if it can be planned.
- Large `execution_proxy_score` usually indicates high joint motion, wrist motion, or joint-limit proximity.
- `joint_limit_margin=0` indicates a candidate is exactly at a joint boundary. In the current version this is a soft ranking penalty, not a hard rejection.

## 8. Observed Behavior of the Current Version

The current 5-7 version performs best when the hard geometric bounds are interpreted as safety/acceptance limits and the quality score is tuned to prefer centered, axis-aligned views within those bounds.

Observed improvements:

- Candidate-pool execution improves robustness because the executor can fall back when the first joint target is not plannable.
- Removing static state-validity checks avoids false confidence from a configuration-only validity test that does not match MoveIt goal sampling behavior.
- Zero-residual axis and lateral costs make the quality objective easier to explain: axis and lateral errors are always encouraged toward zero.
- Raw feasible priority prevents tolerance-only candidates from displacing true hard-boundary-feasible candidates.

Observed limitations:

- If no raw feasible candidate is found, tolerance-only candidates can still be published.
- Joint-limit proximity is still a quality penalty rather than a hard publish blocker.
- The coordinate-descent search can remain local and may miss better basins if the reachable feasible set is narrow.
- Since final path feasibility is checked only in the executor, solver quality does not guarantee executable motion; it only orders candidates for planning attempts.

## 9. Paper-Writing Summary

The method can be described as a joint-space, FK-based viewpoint optimizer with a feasibility-first search policy and executor-side planning fallback. Instead of directly solving a pose IK problem, the solver searches in the robot joint space, evaluates camera geometry through FK, enforces a small set of task-specific hard constraints, and publishes multiple sorted joint candidates. This decouples viewpoint generation from path planning while keeping MoveIt as the final motion feasibility gate.

A compact description:

```text
Given a detected cylinder center and axis, the method searches robot joint
configurations whose FK camera pose satisfies axial standoff, lateral-axis
offset, view-axis alignment, and joint-bound constraints. The search first
minimizes normalized hard-constraint violation, then ranks feasible candidates
by a weighted sum of execution-proxy and observation-quality terms. A sorted
candidate pool is passed to MoveIt, which attempts path planning candidate by
candidate and executes the first successful plan.
```

This is useful for the paper because it separates:

- perception-derived viewpoint geometry;
- joint-space feasibility and comfort;
- final path feasibility through MoveIt planning.

The main design claim is not that the FK score predicts full path feasibility, but that it produces a prioritized candidate set that improves the chance of finding a feasible motion without embedding full path planning inside the solver.
