#!/usr/bin/env python3
"""Hand-eye repeatability and static-board drift checker.

Usage:
  python3 repeatability_check.py compare --files hand_eye_calibration_result_1.yaml hand_eye_calibration_result_2.yaml hand_eye_calibration_result_3.yaml
  python3 repeatability_check.py board --samples 30 --interval 0.5
"""

import argparse
import itertools
import math
import re
import subprocess
import time
from typing import List, Tuple


def _load_tq(path: str) -> Tuple[List[float], List[float]]:
    txt = open(path, "r", encoding="utf-8").read()
    block = "tool_to_camera"
    if "tool_to_camera:" not in txt:
        raise RuntimeError(f"Missing tool_to_camera block in {path}")

    def _get_val(field: str) -> float:
        m = re.search(rf"{field}:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", txt)
        if not m:
            raise RuntimeError(f"Missing field {field} in {path}")
        return float(m.group(1))

    # 限制在对应段内更稳健
    m_block = re.search(rf"{block}:\n((?:\s{{2,}}.*\n)+)", txt)
    if not m_block:
        raise RuntimeError(f"No transform block in {path}")
    btxt = m_block.group(1)

    def _get_block_val(name: str) -> float:
        m = re.search(rf"{name}:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", btxt)
        if not m:
            return _get_val(name)
        return float(m.group(1))

    t = [_get_block_val("x"), _get_block_val("y"), _get_block_val("z")]

    # rotation x/y/z/w：取 rotation 小节
    rot = re.search(r"rotation:\n(\s+x:.*\n\s+y:.*\n\s+z:.*\n\s+w:.*)", btxt)
    if rot:
        rtxt = rot.group(1)
        qx = float(re.search(r"x:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", rtxt).group(1))
        qy = float(re.search(r"y:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", rtxt).group(1))
        qz = float(re.search(r"z:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", rtxt).group(1))
        qw = float(re.search(r"w:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", rtxt).group(1))
    else:
        qx = _get_val("x")
        qy = _get_val("y")
        qz = _get_val("z")
        qw = _get_val("w")
    return t, [qx, qy, qz, qw]


def _norm(v: List[float]) -> float:
    return math.sqrt(sum(x * x for x in v))


def _quat_angle_deg(q1: List[float], q2: List[float]) -> float:
    n1 = _norm(q1)
    n2 = _norm(q2)
    q1n = [x / n1 for x in q1]
    q2n = [x / n2 for x in q2]
    dot = abs(sum(a * b for a, b in zip(q1n, q2n)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def _vec_sub(a: List[float], b: List[float]) -> List[float]:
    return [x - y for x, y in zip(a, b)]


def _vec_norm(a: List[float]) -> float:
    return math.sqrt(sum(x * x for x in a))


def compare(files: List[str], trans_thr: float, rot_thr: float) -> int:
    vals = []
    for f in files:
        t, q = _load_tq(f)
        vals.append((f, t, q))

    print("== Calibration pairwise repeatability ==")
    ok = True
    for (fa, ta, qa), (fb, tb, qb) in itertools.combinations(vals, 2):
        dt = _vec_norm(_vec_sub(ta, tb))
        dr = _quat_angle_deg(qa, qb)
        print(f"{fa} <-> {fb}: dT={dt:.4f} m, dR={dr:.2f} deg")
        if dt > trans_thr or dr > rot_thr:
            ok = False

    print(f"Thresholds: dT < {trans_thr:.3f} m, dR < {rot_thr:.1f} deg")
    print("RESULT: PASS" if ok else "RESULT: FAIL")
    return 0 if ok else 2


def _call_test_board_pose() -> str:
    cmd = [
        "ros2",
        "service",
        "call",
        "/calibration/test_board_pose",
        "std_srvs/srv/Trigger",
        "{}",
    ]
    out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True)
    return out


def _parse_vec(output: str, key: str):
    m = re.search(rf"{key}=\[([^\]]+)\]", output)
    if not m:
        return None
    vals = [float(v) for v in m.group(1).split(",")]
    return vals


def board(samples: int, interval: float) -> int:
    base_t_list = []
    base_q_list = []

    print("== Static board drift: base<-board ==")
    for i in range(samples):
        raw = _call_test_board_pose()
        base_t = _parse_vec(raw, "base_t")
        base_q = _parse_vec(raw, "base_q")
        if base_t is None or base_q is None:
            print(f"[{i+1:02d}] parse failed")
        else:
            base_t_list.append(base_t)
            base_q_list.append(base_q)
            print(f"[{i+1:02d}] t=[{base_t[0]:.4f}, {base_t[1]:.4f}, {base_t[2]:.4f}]")
        time.sleep(interval)

    if len(base_t_list) < 3:
        print("Not enough valid samples.")
        return 3

    n = len(base_t_list)
    mean_t = [sum(v[i] for v in base_t_list) / n for i in range(3)]
    std_t = []
    for i in range(3):
        var = sum((v[i] - mean_t[i]) ** 2 for v in base_t_list) / n
        std_t.append(math.sqrt(var))
    radial = [_vec_norm(_vec_sub(v, mean_t)) for v in base_t_list]

    ref_q = base_q_list[0]
    rot_deg = [_quat_angle_deg(ref_q, q) for q in base_q_list]

    print("\n-- Stats --")
    print(f"mean xyz = [{mean_t[0]:.4f}, {mean_t[1]:.4f}, {mean_t[2]:.4f}] m")
    print(f"std  xyz = [{std_t[0]:.4f}, {std_t[1]:.4f}, {std_t[2]:.4f}] m")
    radial_mean = sum(radial) / len(radial)
    radial_var = sum((r - radial_mean) ** 2 for r in radial) / len(radial)
    radial_std = math.sqrt(radial_var)
    rot_mean = sum(rot_deg) / len(rot_deg)
    rot_var = sum((r - rot_mean) ** 2 for r in rot_deg) / len(rot_deg)
    rot_std = math.sqrt(rot_var)
    print(f"radial std/max = {radial_std:.4f} / {max(radial):.4f} m")
    print(f"rot mean/std/max = {rot_mean:.3f} / {rot_std:.3f} / {max(rot_deg):.3f} deg")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_compare = sub.add_parser("compare")
    p_compare.add_argument("--files", nargs="+", required=True)
    p_compare.add_argument("--trans-thr", type=float, default=0.03)
    p_compare.add_argument("--rot-thr", type=float, default=8.0)

    p_board = sub.add_parser("board")
    p_board.add_argument("--samples", type=int, default=30)
    p_board.add_argument("--interval", type=float, default=0.5)

    args = parser.parse_args()
    if args.cmd == "compare":
        return compare(args.files, args.trans_thr, args.rot_thr)
    return board(args.samples, args.interval)


if __name__ == "__main__":
    raise SystemExit(main())
