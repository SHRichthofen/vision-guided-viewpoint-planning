# Hand-eye calibration update record (2026-03-30)

## 1) Implemented optimizations summary

### Node-side (sampling and solve robustness)
- Sampling pose source changed to **real `/tool_pose`** only; `/target_pose` kept for logging only.
- Added stale-data guards:
  - `max_pose_age_sec`
  - `max_image_age_sec`
- Added checkerboard border guard:
  - `board_margin_px`
- Added manual compute service:
  - `/calibration/compute`
- Added optional robust re-solve:
  - `enable_outlier_rejection`
  - `outlier_reject_ratio`
  - `min_inlier_samples`
  - `orientation_residual_weight`
  - pipeline: full-set solve -> residual ranking (position + orientation) -> outlier drop -> re-solve if improved

### Orientation-aware residual (new)
- Previous outlier ranking only used board position consistency in base frame.
- Now residual is:
  - position term: distance to mean board position
  - orientation term: geodesic rotation distance to orientation medoid
  - combined score: `pos_err + orientation_residual_weight * rot_err`
- Purpose: prevent solutions that keep similar position error while drifting significantly in orientation.

### Solver-side
- Replaced custom hand-eye solver core with OpenCV `calibrateHandEye` (PARK method).
- Added rotation orthonormalization (SVD projection to SO(3)).
- Quaternion normalized before YAML saving.

### Script-side (`calib_pose_auto.sh`)
- Candidate pool sampling with success-target stop condition.
- Added per-pose capture retries:
  - `CAPTURE_RETRY`
  - `RETRY_WAIT_SEC`
- Randomized candidate order (if `shuf` available) to reduce order bias.
- Always triggers `/calibration/compute` at end if success target reached.

---

## 2) Why repeated runs can still be unstable

Observed issue: multiple runs produce similar error values but noticeably different extrinsics.

Likely causes:
1. Remaining sample distribution bias across runs (despite randomization).
2. Pose-image pairing is asynchronous; freshness checks reduce but do not eliminate micro-mismatch.
3. `/tool_pose` is computed by FK from servo angles + model; not direct Cartesian sensor measurement.
4. Checkerboard quality constraints are still permissive under some edge conditions (lighting, partial blur).

---

## 3) Verification of arm state source and `/tool_pose` semantics

### Is calibration using command pose or measured pose?
- Calibration sample uses `latest_arm_pose_` from `/tool_pose` in capture callback.
- `/target_pose` callback no longer writes sampling pose, only logs target command.

### Is `/tool_pose` real end-effector pose?
- In control node, servo DDS topic `current_servo_angle` is read into `current_joint_positions_`.
- `/tool_pose` is then published from FK of these positions via robot model.
- Therefore `/tool_pose` is **feedback-based FK pose** (derived from measured joint angles), not raw command pose.

Implication:
- Better than command pose sampling, but still model-dependent.
- FK/model error, joint offsets, and timing can still affect final hand-eye consistency.

---

## 4) Next recommended actions

1. Keep anti-flicker camera settings fixed and constant lighting.
2. Increase `REQUIRED_SUCCESS` and keep broad orientation diversity.
3. Run 3 independent calibrations and compare pairwise translation/rotation consistency.
4. If still unstable, tighten inlier policy (higher reject ratio / stricter quality gates).

---

## 5) New runtime optimization (2026-03-30, later)

### Stability-gated capture (replacing fixed waiting)
- Added `/calibration/is_stable` service.
- Stability is evaluated over a recent tool-pose window:
  - position max change <= `stable_pos_thresh_m`
  - orientation max change <= `stable_rot_thresh_deg`
- Added motion-start gate after each new target:
  - must observe movement (`motion_start_pos_thresh_m` or `motion_start_rot_thresh_deg`) before stable=true
  - avoids false immediate stable right after command publish
- Script now waits for stability service instead of pure fixed sleep.
- Benefit: faster when arm stabilizes quickly; safer when arm settles slowly.

### Online sample acceptance (per-capture filtering)
- Added optional per-sample online filter before inserting sample into pool.
- After warmup, each candidate is validated by temporary solve + residual checks:
  - candidate residual threshold
  - residual median threshold
  - temporary solver error threshold
- Benefit: reduce late-stage contamination by low-quality samples.
