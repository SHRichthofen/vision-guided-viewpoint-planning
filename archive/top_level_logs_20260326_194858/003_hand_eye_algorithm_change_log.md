# Hand-eye calibration algorithm change log

Date: 2026-03-26

## Background symptom
- Even with 25-46 samples, calibration output was unstable:
  - translation magnitude was unrealistic for wrist-mounted camera
  - quaternion norm tended to be non-unit-like before normalization
  - reported error stayed high (~1.2m+)

## Old solver behavior (custom implementation)
File: `src/hand_eye_calibration/src/calibration_utils.cpp`

- Rotation estimation used an ad-hoc matrix accumulation (`R_A - R_B^T`) and SVD.
- Translation estimation solved a linear system with coefficient `(I - R_cam_board)` and RHS `t_base_tool - R_X * t_cam_board`.
- Validation compared transforms in a way that mixed terms not directly equivalent to the hand-eye objective consistency check.

### Risk in old approach
- Formulation was not equivalent to a standard, proven AX=XB solver pipeline.
- Numeric sensitivity could produce physically implausible solutions even with many samples.

## New solver behavior (OpenCV implementation)
File: `src/hand_eye_calibration/src/calibration_utils.cpp`

- Replaced solving core with `cv::calibrateHandEye(...)` using Park method.
- Inputs are assembled as:
  - gripper->base from `/tool_pose` samples
  - target(board)->camera from checkerboard PnP per frame
- Output is directly camera->gripper (`^gT_c`), matching tool->camera semantic in project.
- Added orthonormalization of solved rotation (SVD projection to SO(3)).
- YAML saving now normalizes quaternion before writing.

## Why this change
1. **Correctness**: standard library solver follows established hand-eye formulations.
2. **Stability**: robust numerical behavior across practical sample noise.
3. **Maintainability**: easier to audit and compare with known references.

## Additional quality guards already added
- Sample pose source forced to `/tool_pose` (real measured pose), not command `/target_pose`.
- Rejected stale image/pose samples by age threshold.
- Rejected board detections too close to image border.

## Expected outcome
- Result should become physically plausible (translation on realistic mount scale).
- Error metric should drop significantly when sample diversity and board visibility are good.
- Remaining high error would then likely point to data quality (intrinsics mismatch, board config mismatch, motion blur, or timestamp sync) rather than solver core.
