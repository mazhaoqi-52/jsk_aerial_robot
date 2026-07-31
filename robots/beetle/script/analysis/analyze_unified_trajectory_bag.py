#!/usr/bin/env python3

"""Summarize Beetle unified-mode trajectory tracking from a ROS1 bag."""

import argparse
import json
import math
import re

import numpy as np
import rosbag
from scipy import signal
from tf.transformations import euler_from_quaternion


AXES = ("x", "y", "z", "yaw")
HEARTBEAT_NUMBER = re.compile(r"([a-z_]+)=(-?[0-9]+(?:\.[0-9]+)?)")


def message_time(msg, bag_time):
    if hasattr(msg, "header") and not msg.header.stamp.is_zero():
        return msg.header.stamp.to_sec()
    return bag_time.to_sec()


def scalar(value):
    if isinstance(value, (list, tuple)):
        return float(value[0]) if value else math.nan
    return float(value)


def statistics(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not values.size:
        return {}
    return {
        "mean": float(np.mean(values)),
        "std": float(np.std(values)),
        "rms": float(np.sqrt(np.mean(np.square(values)))),
        "max_abs": float(np.max(np.abs(values))),
        "p95_abs": float(np.percentile(np.abs(values), 95)),
        "peak_to_peak": float(np.ptp(values)),
    }


def sort_unique_series(times, values):
    """Sort samples and retain the last value for duplicate timestamps."""
    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=float)
    if not times.size:
        return times, values
    finite = np.isfinite(times)
    times, values = times[finite], values[finite]
    order = np.argsort(times, kind="stable")
    times, values = times[order], values[order]
    keep = np.ones(times.size, dtype=bool)
    keep[:-1] = times[:-1] != times[1:]
    return times[keep], values[keep]


def zero_order_hold(times, values, query_times):
    """Sample a discrete command using its controller-side hold semantics."""
    indices = np.searchsorted(times, query_times, side="right") - 1
    return values[indices]


def estimate_tracking_lag(times, target, actual, max_lag_s=2.0):
    times = np.asarray(times, dtype=float)
    target = np.asarray(target, dtype=float)
    actual = np.asarray(actual, dtype=float)
    finite = np.isfinite(times) & np.isfinite(target) & np.isfinite(actual)
    times, target, actual = times[finite], target[finite], actual[finite]
    times, paired = sort_unique_series(
        times, np.column_stack((target, actual)))
    if times.size < 32:
        return {"lag_s": math.nan, "correlation": math.nan}
    positive_steps = np.diff(times)
    positive_steps = positive_steps[positive_steps > 0.0]
    if not positive_steps.size:
        return {"lag_s": math.nan, "correlation": math.nan}
    dt = float(np.median(positive_steps))
    uniform_times = np.arange(times[0], times[-1] + 0.5 * dt, dt)
    target = np.interp(uniform_times, times, paired[:, 0])
    actual = np.interp(uniform_times, times, paired[:, 1])
    if (uniform_times.size < 32 or np.std(target) < 1e-5 or
            np.std(actual) < 1e-5):
        return {"lag_s": math.nan, "correlation": math.nan}
    target = signal.detrend(target)
    actual = signal.detrend(actual)
    corr = signal.correlate(actual, target, mode="full", method="fft")
    lags = signal.correlation_lags(actual.size, target.size, mode="full")
    max_samples = max(1, int(round(max_lag_s / dt)))
    valid = np.abs(lags) <= max_samples
    valid_indices = np.flatnonzero(valid)
    peak_index = valid_indices[np.argmax(corr[valid])]
    denom = np.linalg.norm(actual) * np.linalg.norm(target)
    return {
        "lag_s": float(lags[peak_index] * dt),
        "correlation": float(corr[peak_index] / denom) if denom > 0 else math.nan,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--reference-module", type=int, default=2)
    parser.add_argument(
        "--trim-start-s", type=float, default=0.0,
        help="Ignore this many seconds after the trajectory_run marker.",
    )
    args = parser.parse_args()

    pid_times = []
    pid = {
        axis: {
            field: [] for field in
            ("target", "error", "velocity_error", "p", "i", "d", "total")
        }
        for axis in AXES
    }
    phases = []
    active_attitude_target = [0.0, 0.0]
    attitude_target_times, attitude_targets = [], []
    body_attitude_times, body_attitudes = [], []
    desire_attitude_times, desire_attitudes = [], []
    gimbal_times, gimbal_error = [], []
    margin_times, margins = [], []
    pwm_times = {module_id: [] for module_id in (1, 2, 3)}
    pwm = {module_id: [] for module_id in (1, 2, 3)}
    heartbeat_times = {module_id: [] for module_id in (1, 2, 3)}
    heartbeats = {module_id: [] for module_id in (1, 2, 3)}
    flight_times = {module_id: [] for module_id in (1, 2, 3)}
    flight_states = {module_id: [] for module_id in (1, 2, 3)}

    reference = args.reference_module
    topics = [
        "/beetle_pid_tuning/phase",
        "/assembly/uav/nav",
        "/assemble/debug/pose/pid",
        f"/beetle{reference}/uav/baselink/odom",
        f"/beetle{reference}/desire_coordinate",
        f"/beetle{reference}/unified_control/gimbal_tracking",
        f"/beetle{reference}/unified_control/qp_thrust_margin",
    ]
    for module_id in (1, 2, 3):
        topics.extend([
            f"/beetle{module_id}/motor_pwms",
            f"/beetle{module_id}/unified_control/heartbeat",
            f"/beetle{module_id}/flight_state",
        ])

    with rosbag.Bag(args.bag) as bag:
        for topic, msg, bag_time in bag.read_messages(topics=topics):
            stamp = message_time(msg, bag_time)
            if topic == "/beetle_pid_tuning/phase":
                phases.append((bag_time.to_sec(), msg.data))
            elif topic == "/assembly/uav/nav":
                if msg.roll_nav_mode == msg.POS_MODE:
                    active_attitude_target[0] = msg.target_roll
                if msg.pitch_nav_mode == msg.POS_MODE:
                    active_attitude_target[1] = msg.target_pitch
                attitude_target_times.append(bag_time.to_sec())
                attitude_targets.append(list(active_attitude_target))
            elif topic == f"/beetle{reference}/uav/baselink/odom":
                q = msg.pose.pose.orientation
                roll, pitch, _ = euler_from_quaternion(
                    (q.x, q.y, q.z, q.w))
                body_attitude_times.append(bag_time.to_sec())
                body_attitudes.append([roll, pitch])
            elif topic == f"/beetle{reference}/desire_coordinate":
                desire_attitude_times.append(bag_time.to_sec())
                desire_attitudes.append([msg.roll, msg.pitch])
            elif topic == "/assemble/debug/pose/pid":
                pid_times.append(stamp)
                for axis in AXES:
                    axis_msg = getattr(msg, axis)
                    pid[axis]["target"].append(float(axis_msg.target_p))
                    pid[axis]["error"].append(float(axis_msg.err_p))
                    pid[axis]["velocity_error"].append(float(axis_msg.err_d))
                    pid[axis]["p"].append(scalar(axis_msg.p_term))
                    pid[axis]["i"].append(scalar(axis_msg.i_term))
                    pid[axis]["d"].append(scalar(axis_msg.d_term))
                    pid[axis]["total"].append(scalar(axis_msg.total))
            elif topic.endswith("/unified_control/gimbal_tracking"):
                gimbal_times.append(stamp)
                gimbal_error.append(list(msg.effort))
            elif topic.endswith("/unified_control/qp_thrust_margin"):
                margin_times.append(stamp)
                margins.append(list(msg.data))
            else:
                for module_id in (1, 2, 3):
                    if not topic.startswith(f"/beetle{module_id}/"):
                        continue
                    if topic.endswith("/motor_pwms"):
                        pwm_times[module_id].append(stamp)
                        pwm[module_id].append(list(msg.motor_value[:4]))
                    elif topic.endswith("/unified_control/heartbeat"):
                        heartbeat_times[module_id].append(stamp)
                        parsed = {
                            key: float(value)
                            for key, value in HEARTBEAT_NUMBER.findall(msg.value)
                        }
                        parsed["key"] = msg.key
                        parsed["value"] = msg.value
                        heartbeats[module_id].append(parsed)
                    elif topic.endswith("/flight_state"):
                        flight_times[module_id].append(stamp)
                        flight_states[module_id].append(int(msg.data))

    phases.sort()
    run_start = next(
        (stamp for stamp, name in phases
         if name.startswith("trajectory_run:")),
        None,
    )
    run_end = next(
        (stamp for stamp, name in phases if name == "trajectory_post_settle"),
        None,
    )
    if run_start is None or run_end is None or run_end <= run_start:
        raise RuntimeError(f"trajectory phase markers missing: {phases}")
    analysis_start = run_start + args.trim_start_s
    if analysis_start >= run_end:
        raise RuntimeError(
            f"trim-start-s={args.trim_start_s} leaves no trajectory data")

    def mask(times):
        times = np.asarray(times, dtype=float)
        return (times >= analysis_start) & (times < run_end)

    pid_times_array = np.asarray(pid_times, dtype=float)
    pid_mask = mask(pid_times_array)
    selected_times = pid_times_array[pid_mask]
    result = {
        "bag": args.bag,
        "trajectory_phase_s": [run_start, run_end],
        "analysis_phase_s": [analysis_start, run_end],
        "duration_s": float(run_end - analysis_start),
        "trim_start_s": args.trim_start_s,
        "phases": phases,
        "tracking": {},
        "actuation": {},
        "runtime": {},
        "attitude_semantics": {
            "command_source": "/assembly/uav/nav",
            "physical_body_source": (
                f"/beetle{reference}/uav/baselink/odom"),
            "rate_limited_target_source": (
                f"/beetle{reference}/desire_coordinate"),
            "excluded_virtual_frame": "/assemble/cog/odom",
        },
    }

    def attitude_comparison(reference_times, references, sample_times, samples,
                            reference_source, measured_source):
        reference_times, references = sort_unique_series(
            reference_times, references)
        sample_times, samples = sort_unique_series(sample_times, samples)
        if reference_times.size:
            reference_keep = reference_times < run_end
            reference_times = reference_times[reference_keep]
            references = references[reference_keep]
        if sample_times.size:
            sample_keep = mask(sample_times)
            sample_times = sample_times[sample_keep]
            samples = samples[sample_keep]
        comparison = {
            "reference_source": reference_source,
            "measured_source": measured_source,
            "reference_count": int(reference_times.size),
            "sample_count": int(sample_times.size),
        }
        if (reference_times.size == 0 or sample_times.size == 0 or
                references.ndim != 2 or samples.ndim != 2):
            comparison["available"] = False
            return comparison

        common_start = max(analysis_start, reference_times[0], sample_times[0])
        common_samples = sample_times >= common_start
        sample_times = sample_times[common_samples]
        samples = samples[common_samples]
        comparison["sample_count"] = int(sample_times.size)
        if not sample_times.size:
            comparison["available"] = False
            return comparison

        comparison["available"] = True
        for index, axis in enumerate(("roll", "pitch")):
            reference_values = zero_order_hold(
                reference_times, references[:, index], sample_times)
            measured = samples[:, index]
            error = reference_values - measured
            comparison[axis] = {
                "reference": statistics(reference_values),
                "measured": statistics(measured),
                "error": statistics(error),
                "tracking_lag": estimate_tracking_lag(
                    sample_times, reference_values, measured),
            }
        return comparison

    # /assemble/cog/odom is a virtual CoG control frame for Beetle: it remains
    # near level while desire_coordinate tilts the physical assembled body.
    result["tracking"]["physical_body_attitude"] = attitude_comparison(
        attitude_target_times,
        attitude_targets,
        body_attitude_times,
        body_attitudes,
        "/assembly/uav/nav",
        f"/beetle{reference}/uav/baselink/odom",
    )
    result["actuation"]["rate_limited_body_attitude_target"] = (
        attitude_comparison(
            attitude_target_times,
            attitude_targets,
            desire_attitude_times,
            desire_attitudes,
            "/assembly/uav/nav",
            f"/beetle{reference}/desire_coordinate",
        )
    )
    result["tracking"]["body_response_to_rate_limited_target"] = (
        attitude_comparison(
            desire_attitude_times,
            desire_attitudes,
            body_attitude_times,
            body_attitudes,
            f"/beetle{reference}/desire_coordinate",
            f"/beetle{reference}/uav/baselink/odom",
        )
    )

    error_matrix = []
    target_matrix = []
    actual_matrix = []
    for axis in AXES:
        selected = {
            field: np.asarray(values, dtype=float)[pid_mask]
            for field, values in pid[axis].items()
        }
        actual = selected["target"] - selected["error"]
        target_matrix.append(selected["target"])
        actual_matrix.append(actual)
        error_matrix.append(selected["error"])
        result["tracking"][axis] = {
            "target": statistics(selected["target"]),
            "error": statistics(selected["error"]),
            "velocity_error": statistics(selected["velocity_error"]),
            "p_term": statistics(selected["p"]),
            "i_term": statistics(selected["i"]),
            "d_term": statistics(selected["d"]),
            "total": statistics(selected["total"]),
            "tracking_lag": estimate_tracking_lag(
                selected_times, selected["target"], actual),
        }

    error_matrix = np.asarray(error_matrix).T
    target_matrix = np.asarray(target_matrix).T
    actual_matrix = np.asarray(actual_matrix).T
    xyz_error_norm = np.linalg.norm(error_matrix[:, :3], axis=1)
    target_xyz_step = np.linalg.norm(np.diff(target_matrix[:, :3], axis=0), axis=1)
    actual_xyz_step = np.linalg.norm(np.diff(actual_matrix[:, :3], axis=0), axis=1)
    result["tracking"]["combined"] = {
        "xyz_error_norm": statistics(xyz_error_norm),
        "target_path_length_m": float(np.sum(target_xyz_step)),
        "actual_path_length_m": float(np.sum(actual_xyz_step)),
    }

    gimbal = np.asarray(gimbal_error, dtype=float)
    gimbal_selected = gimbal[mask(gimbal_times)] if gimbal.size else gimbal
    result["actuation"]["gimbal_tracking_error_rad"] = statistics(
        gimbal_selected.ravel())

    margin = np.asarray(margins, dtype=float)
    margin_selected = margin[mask(margin_times)] if margin.size else margin
    result["actuation"]["qp_thrust_margin_n"] = statistics(
        margin_selected.ravel())
    if margin_selected.size:
        result["actuation"]["qp_min_margin_per_cycle_n"] = statistics(
            np.min(margin_selected, axis=1))

    result["actuation"]["motor_pwm"] = {}
    for module_id in (1, 2, 3):
        values = np.asarray(pwm[module_id], dtype=float)
        selected = values[mask(pwm_times[module_id])] if values.size else values
        result["actuation"]["motor_pwm"][str(module_id)] = {
            "all_motors": statistics(selected.ravel()),
            "per_motor_std": (
                [float(value) for value in np.std(selected, axis=0)]
                if selected.size else []
            ),
        }

    for module_id in (1, 2, 3):
        heartbeat_selected = [
            item for stamp, item in zip(
                heartbeat_times[module_id], heartbeats[module_id])
            if analysis_start <= stamp < run_end
        ]
        flight_selected = [
            state for stamp, state in zip(
                flight_times[module_id], flight_states[module_id])
            if analysis_start <= stamp < run_end
        ]
        gaps = [
            item.get("since_pub", math.nan) for item in heartbeat_selected
            if item.get("since_pub", -1.0) >= 0.0
        ]
        result["runtime"][str(module_id)] = {
            "flight_states": sorted(set(flight_selected)),
            "command_gap_s": statistics(gaps),
            "allocation_failed_count": sum(
                item["key"] == "allocation_failed" or
                "allocation_failed" in item["value"]
                for item in heartbeat_selected
            ),
        }

    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=True))


if __name__ == "__main__":
    main()
