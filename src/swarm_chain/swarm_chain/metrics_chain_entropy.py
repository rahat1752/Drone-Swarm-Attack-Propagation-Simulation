#!/usr/bin/env python3

import argparse
import glob
import json
import math
import os
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

import numpy as np
import pandas as pd



# Eight non-negative formation-error bins with upper bounds
# {0.5, 1, 2, 4, 8, 16, 32, infinity} m.  np.histogram expects
# bin *edges*, so the leading 0.0 creates the eight intended bins.
FORMATION_ERROR_BIN_EDGES = np.array([
    0.0, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, np.inf
])
FORMATION_ERROR_BIN_COUNT = 8
FORMATION_ENTROPY_MAX_BITS = math.log2(FORMATION_ERROR_BIN_COUNT)  # = 3 bits


# -----------------------------
# File helpers
# -----------------------------

def newest(pattern: str) -> str:
    files = glob.glob(pattern)
    if not files:
        raise FileNotFoundError(f"No files matched: {pattern}")
    return max(files, key=os.path.getmtime)


def read_latest_csv(data_dir: Path, pattern: str) -> pd.DataFrame:
    return pd.read_csv(newest(str(data_dir / pattern)))


# -----------------------------
# Shannon entropy helpers
# -----------------------------

def probabilities_from_counts(counts) -> np.ndarray:
    counts = np.asarray(counts, dtype=float)
    total = counts.sum()
    if total <= 0:
        return np.array([])
    return counts[counts > 0] / total


def shannon_from_counts(counts, normalization_bins: Optional[int] = None) -> Dict[str, float]:
    """Shannon entropy in bits plus a normalized value.

    When ``normalization_bins`` is supplied, normalization uses the theoretical
    maximum log2(K) for K bins.  Formation-error entropy passes K=8 to match
    the paper: H_hat_e = H_e / log2(8) = H_e / 3.

    For diagnostic position/velocity/speed entropies, where the number of bins
    is data-dependent, the legacy occupied-bin normalization is retained.
    """
    counts = np.asarray(counts, dtype=float)
    p = probabilities_from_counts(counts)
    occupied_bins = int(np.count_nonzero(counts))

    if len(p) == 0:
        return {
            "shannon": 0.0,
            "shannon_norm": 0.0,
            "occupied_bins": 0,
        }

    h = float(-(p * np.log2(p)).sum())

    if normalization_bins is not None:
        if normalization_bins <= 1:
            h_norm = 0.0
        else:
            h_norm = float(h / math.log2(normalization_bins))
    else:
        h_norm = float(h / math.log2(occupied_bins)) if occupied_bins > 1 else 0.0

    return {
        "shannon": h,
        "shannon_norm": h_norm,
        "occupied_bins": occupied_bins,
    }


def entropy_prefix(prefix: str, counts, normalization_bins: Optional[int] = None) -> Dict[str, float]:
    e = shannon_from_counts(counts, normalization_bins=normalization_bins)
    return {
        f"{prefix}_shannon": e["shannon"],
        f"{prefix}_shannon_norm": e["shannon_norm"],
        f"{prefix}_occupied_bins": e["occupied_bins"],
    }


def histogram_2d_keys(df: pd.DataFrame, col_a: str, col_b: str, bin_size: float):
    if len(df) == 0:
        return np.array([])
    a = np.floor(df[col_a].to_numpy(dtype=float) / bin_size).astype(int)
    b = np.floor(df[col_b].to_numpy(dtype=float) / bin_size).astype(int)
    return pd.Series(list(zip(a, b))).value_counts().to_numpy()


def histogram_1d_values(values, bin_size: float):
    if len(values) == 0:
        return np.array([])
    bins = np.floor(np.asarray(values, dtype=float) / bin_size).astype(int)
    return pd.Series(bins).value_counts().to_numpy()


def histogram_fixed_bins(values, bins):
    if len(values) == 0:
        return np.array([])
    counts, _ = np.histogram(values, bins=bins)
    return counts


def calculate_window_entropies(
    veh: pd.DataFrame,
    err: pd.DataFrame,
    window_ms: int,
    position_bin_m: float,
    velocity_bin_mps: float,
) -> pd.DataFrame:
    """
    Window-level Shannon entropy. This is mostly diagnostic; paper tables can use
    the full-run summary values.
    """
    window_ns = window_ms * 1_000_000
    veh = veh.copy()
    err = err.copy()

    if len(veh):
        veh["window_id"] = (veh["mission_time_ns"] // window_ns).astype(int)
    else:
        veh["window_id"] = pd.Series(dtype=int)

    if len(err):
        err["window_id"] = (err["mission_time_ns"] // window_ns).astype(int)
    else:
        err["window_id"] = pd.Series(dtype=int)

    all_windows = sorted(set(veh["window_id"].unique()).union(set(err["window_id"].unique())))
    rows = []

    for window_id in all_windows:
        vwin = veh[veh["window_id"] == window_id]
        ewin = err[err["window_id"] == window_id]

        row = {
            "window_id": int(window_id),
            "start_time_s": float((window_id * window_ns) / 1e9),
            "sample_count_vehicle": int(len(vwin)),
            "sample_count_error": int(len(ewin)),
        }

        row.update(entropy_prefix("position_entropy", histogram_2d_keys(vwin, "x", "y", position_bin_m)))
        row.update(entropy_prefix("velocity_entropy", histogram_2d_keys(vwin, "vx", "vy", velocity_bin_mps)))
        row.update(entropy_prefix("speed_entropy", histogram_1d_values(vwin["speed_xy"].to_numpy(dtype=float), velocity_bin_mps)))
        row.update(entropy_prefix(
            "formation_entropy",
            histogram_fixed_bins(ewin["error"].to_numpy(dtype=float), FORMATION_ERROR_BIN_EDGES),
            normalization_bins=FORMATION_ERROR_BIN_COUNT,
        ))

        row["mean_formation_error"] = float(ewin["error"].mean()) if len(ewin) else 0.0
        row["p95_formation_error"] = float(ewin["error"].quantile(0.95)) if len(ewin) else 0.0
        row["max_formation_error"] = float(ewin["error"].max()) if len(ewin) else 0.0

        rows.append(row)

    return pd.DataFrame(rows)


# -----------------------------
# Mission duration and trimming
# -----------------------------

def mission_end_ns_from_events(data_dir: Path, veh: pd.DataFrame) -> Tuple[int, str]:
    """
    Uses the first mission_phase_landing event as mission end.
    If event file or landing event is missing, falls back to max vehicle mission_time_ns.
    """
    fallback = int(veh["mission_time_ns"].max()) if len(veh) else 0

    try:
        events = read_latest_csv(data_dir, "mission_events_*.csv")
    except Exception:
        return fallback, "vehicle_log_max_no_event_file"

    if "event" not in events.columns or "mission_time_ns" not in events.columns:
        return fallback, "vehicle_log_max_bad_event_file"

    landing = events[events["event"] == "mission_phase_landing"]
    if len(landing):
        return int(landing["mission_time_ns"].iloc[0]), "first_mission_phase_landing"

    for event_name in ["mission_finished", "mission_complete", "mission_phase_finished"]:
        rows = events[events["event"] == event_name]
        if len(rows):
            return int(rows["mission_time_ns"].iloc[0]), event_name

    return fallback, "vehicle_log_max_no_landing_event"


def trim_to_mission_end(
    veh: pd.DataFrame,
    err: pd.DataFrame,
    dist: pd.DataFrame,
    mission_end_ns: int,
) -> Tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    if len(veh):
        veh = veh[veh["mission_time_ns"] <= mission_end_ns].copy()
    if len(err):
        err = err[err["mission_time_ns"] <= mission_end_ns].copy()
    if len(dist):
        dist = dist[dist["mission_time_ns"] <= mission_end_ns].copy()
    return veh, err, dist


# -----------------------------
# Propagation metrics
# -----------------------------

def _dependency_graph_from_error_log(err: pd.DataFrame):
    """Build parent->children adjacency from the formation-error log."""
    graph = {}
    if len(err) == 0 or not {"drone_id", "parent_id"}.issubset(err.columns):
        return graph

    edges = err[["parent_id", "drone_id"]].dropna().drop_duplicates()
    for _, row in edges.iterrows():
        parent = int(row["parent_id"])
        child = int(row["drone_id"])
        graph.setdefault(parent, set()).add(child)
        graph.setdefault(child, set())
    return graph


def _reachable_with_distance(graph, source: int):
    """Return {reachable_node: hop_distance_from_source}."""
    distance = {}
    queue = [(int(source), 0)]
    seen = {int(source)}

    while queue:
        node, depth = queue.pop(0)
        for child in graph.get(node, set()):
            if child in seen:
                continue
            seen.add(child)
            distance[child] = depth + 1
            queue.append((child, depth + 1))
    return distance


def calculate_propagation_metrics(
    err: pd.DataFrame,
    threshold: float,
    target_drone: int,
    attack_start_s: float,
) -> Dict[str, float]:
    """Paper-aligned propagation metrics.

    PF  = |A_e(a)| / |R(a)|, with PF=0 when R(a) is empty.
    PD  = maximum hop distance from the attacked node to an affected node.
    PS  = |A_e(a)| / max_i(t_aff_i - t_s), in drones/s.
    PSV = mean_i(max_t e_i(t) / theta_e) over affected reachable drones.

    A downstream drone is classified as affected when its maximum logged
    formation error reaches/exceeds ``threshold``.  ``t_aff`` is the first
    threshold crossing in this implementation.
    """
    empty = {
        "propagation_factor": 0.0,
        "propagation_severity_psv": 0.0,
        "propagation_speed_drones_per_s": 0.0,
        "propagation_depth": 0,
        "reachable_downstream_count": 0,
        "affected_link_fraction": 0.0,
        "affected_link_count": 0,
    }
    if len(err) == 0:
        return empty

    graph = _dependency_graph_from_error_log(err)
    reachable_dist = _reachable_with_distance(graph, int(target_drone))
    reachable = set(reachable_dist.keys())
    reachable_count = len(reachable)

    grouped = err.groupby("drone_id")["error"].max()
    affected_ids = [
        int(drone_id)
        for drone_id, max_error in grouped.items()
        if int(drone_id) in reachable and float(max_error) >= threshold
    ]
    affected_count = len(affected_ids)

    # Paper PF: fraction of structurally reachable downstream drones affected.
    pf = float(affected_count / reachable_count) if reachable_count > 0 else 0.0

    # Paper PSV: average normalized peak formation error over affected drones.
    if affected_count > 0 and threshold > 0:
        psv = float(np.mean([float(grouped.loc[d]) / threshold for d in affected_ids]))
    else:
        psv = 0.0

    # PD is relative to the attacked node, not the leader's global hop index.
    pd = max((reachable_dist[d] for d in affected_ids), default=0)

    # First threshold crossing for each affected drone.
    affected_times = []
    for drone_id in affected_ids:
        rows = err[(err["drone_id"] == drone_id) & (err["error"] >= threshold)]
        if len(rows):
            affected_times.append(float(rows["mission_time_ns"].min() / 1e9))

    if affected_times:
        elapsed = max(affected_times) - float(attack_start_s)
        ps = float(affected_count / elapsed) if elapsed > 1e-9 else float("inf")
    else:
        ps = 0.0

    # Diagnostic fraction over all logged parent-child formation links.
    total_logged_links = int(grouped.index.nunique())
    affected_fraction_all_links = (
        float(affected_count / total_logged_links) if total_logged_links else 0.0
    )

    return {
        "propagation_factor": pf,
        "propagation_severity_psv": psv,
        "propagation_speed_drones_per_s": ps,
        "propagation_depth": int(pd),
        "reachable_downstream_count": int(reachable_count),
        "affected_link_fraction": affected_fraction_all_links,
        "affected_link_count": int(affected_count),
    }


# -----------------------------
# Near-collision metrics
# -----------------------------

def count_near_collision_episodes(
    dist: pd.DataFrame,
    threshold: float,
    max_gap_ns: int = 300_000_000,
) -> int:
    """
    Counts threshold-crossing near-collision episodes per drone pair.

    A new episode begins when a pair enters the unsafe distance region.
    If samples have a large time gap while unsafe, a new episode is also counted.
    """
    if len(dist) == 0:
        return 0

    required = {"drone_i", "drone_j", "mission_time_ns", "distance"}
    if not required.issubset(set(dist.columns)):
        missing = required - set(dist.columns)
        raise ValueError(f"pairwise_distance file missing columns: {missing}")

    episode_count = 0

    for _, group in dist.groupby(["drone_i", "drone_j"]):
        group = group.sort_values("mission_time_ns")
        unsafe = group["distance"].to_numpy(dtype=float) < threshold
        times = group["mission_time_ns"].to_numpy(dtype=np.int64)

        in_episode = False
        last_time = None

        for is_unsafe, t in zip(unsafe, times):
            large_gap = last_time is not None and (int(t) - int(last_time)) > max_gap_ns

            if is_unsafe:
                if (not in_episode) or large_gap:
                    episode_count += 1
                    in_episode = True
            else:
                in_episode = False

            last_time = int(t)

    return int(episode_count)


def estimate_near_collision_duration_s(
    dist: pd.DataFrame,
    threshold: float,
    default_sample_period_s: float,
) -> float:
    """
    Approximate unsafe duration from unsafe samples and median per-pair sample spacing.
    """
    if len(dist) == 0:
        return 0.0

    total_duration = 0.0

    for _, group in dist.groupby(["drone_i", "drone_j"]):
        group = group.sort_values("mission_time_ns")
        unsafe_count = int((group["distance"] < threshold).sum())

        if len(group) >= 2:
            diffs_s = np.diff(group["mission_time_ns"].to_numpy(dtype=np.int64)) / 1e9
            diffs_s = diffs_s[diffs_s > 0]
            dt = float(np.median(diffs_s)) if len(diffs_s) else default_sample_period_s
        else:
            dt = default_sample_period_s

        total_duration += unsafe_count * dt

    return float(total_duration)


# -----------------------------
# Mission Impact Score
# -----------------------------

def safe_mean(series: pd.Series) -> float:
    return float(series.dropna().mean()) if len(series.dropna()) else 0.0


def safe_max(series: pd.Series) -> float:
    return float(series.dropna().max()) if len(series.dropna()) else 0.0


def bounded_relative_larger_worse(value: float, baseline: float, floor: float) -> float:
    denom = max(abs(float(baseline)), float(floor))
    return float(min(1.0, max(0.0, (float(value) - float(baseline)) / denom)))


def bounded_relative_smaller_worse(value: float, baseline: float, floor: float) -> float:
    denom = max(abs(float(baseline)), float(floor))
    return float(min(1.0, max(0.0, (float(baseline) - float(value)) / denom)))


def calculate_absolute_mis(summary: Dict[str, float]) -> float:
    """
    Absolute threshold score for quick inspection only.
    Paper results should use baseline-relative MIS.
    """
    unsafe_flag = 1.0 if summary["min_interdrone_distance"] < summary["collision_threshold_m"] else 0.0

    score = 100.0 * (
        0.20 * min(summary["mean_formation_error"] / 10.0, 1.0)
        + 0.15 * min(summary["p95_formation_error"] / 20.0, 1.0)
        + 0.10 * min(summary["near_collision_episode_count"] / 5.0, 1.0)
        + 0.10 * min(summary["near_collision_duration_s"] / 5.0, 1.0)
        + 0.10 * min(summary["affected_link_fraction"], 1.0)
        + 0.10 * unsafe_flag
        + 0.10 * min(summary["formation_entropy_shannon"] / 3.0, 1.0)
        + 0.10 * min(summary["mean_latency_ms"] / 100.0, 1.0)
        + 0.05 * min(summary["mission_duration_s"] / 300.0, 1.0)
    )

    return float(min(100.0, max(0.0, score)))


def calculate_baseline_relative_mis(
    summary: Dict[str, float],
    baseline_row: Dict[str, float],
) -> Tuple[float, Dict[str, float]]:
    """
    Bounded [0,100] MIS relative to mean baseline values.

    Weights emphasize the metrics that are meaningful for this paper:
    formation degradation, safety loss, propagation, timing/freshness, and
    formation-error entropy. Position/velocity/speed entropy are still reported
    but are not used in MIS because they can be dominated by the planned mission path.
    """
    terms = {
        # metric: (weight, direction, denominator_floor)
        "mean_formation_error": (0.18, "larger", 0.5),
        "p95_formation_error": (0.18, "larger", 1.0),
        "max_formation_error": (0.08, "larger", 1.0),
        "min_interdrone_distance": (0.14, "smaller", 0.5),
        "near_collision_episode_count": (0.10, "larger", 1.0),
        "near_collision_duration_s": (0.08, "larger", 0.5),
        "propagation_severity_psv": (0.10, "larger", 0.5),
        "mean_latency_ms": (0.06, "larger", 5.0),
        "formation_entropy_shannon": (0.06, "larger", 0.25),
        "mission_duration_s": (0.02, "larger", 1.0),
    }

    weighted_sum = 0.0
    components = {}

    for metric, (weight, direction, floor) in terms.items():
        if metric not in summary or metric not in baseline_row:
            continue

        value = float(summary[metric])
        baseline = float(baseline_row[metric])

        if direction == "larger":
            delta = bounded_relative_larger_worse(value, baseline, floor)
        elif direction == "smaller":
            delta = bounded_relative_smaller_worse(value, baseline, floor)
        else:
            raise ValueError(f"Unknown MIS direction: {direction}")

        components[f"mis_delta_{metric}"] = delta
        weighted_sum += weight * delta

    score = float(min(100.0, max(0.0, 100.0 * weighted_sum)))
    return score, components


# -----------------------------
# Propagation threshold resolution
# -----------------------------

def recommended_threshold_from_baseline(
    baseline_row: Dict[str, float],
    fallback: float,
) -> float:
    """Return the paper's fixed empirical propagation threshold theta_e.

    The released artifact uses theta_e = 6 m by default.  ``fallback`` is kept
    as the configurable CLI value so users may override it explicitly.
    """
    return float(fallback)


def resolve_propagation_threshold(
    explicit_threshold: Optional[float],
    baseline_summary: Optional[Path],
    fallback: float,
) -> Tuple[float, str]:
    """Resolve theta_e. The paper artifact uses 6 m unless explicitly overridden."""
    if explicit_threshold is not None:
        return float(explicit_threshold), "explicit_cli"
    return float(fallback), "paper_default_theta_e"


# -----------------------------
# Per-run summary
# -----------------------------

def compute_run_summary(
    data_dir: Path,
    window_ms: int,
    collision_threshold: float,
    propagation_threshold: float,
    propagation_threshold_source: str,
    target_drone: int,
    attack_start_s: float,
    position_bin_m: float,
    velocity_bin_mps: float,
    episode_gap_s: float,
    default_sample_period_s: float,
    baseline_summary: Optional[Path] = None,
    write_outputs: bool = True,
) -> Dict[str, float]:
    err = read_latest_csv(data_dir, "formation_error_*.csv")
    veh = read_latest_csv(data_dir, "vehicle_log_*.csv")
    dist = read_latest_csv(data_dir, "pairwise_distance_*.csv")

    for col in ["parent_swarm_age_ms", "parent_update_age_ms"]:
        if col in err.columns:
            err[col] = pd.to_numeric(err[col], errors="coerce")

    mission_end_ns, mission_end_source = mission_end_ns_from_events(data_dir, veh)
    veh, err, dist = trim_to_mission_end(veh, err, dist, mission_end_ns)

    entropy_windows = calculate_window_entropies(
        veh,
        err,
        window_ms,
        position_bin_m,
        velocity_bin_mps,
    )

    position_entropy = entropy_prefix(
        "position_entropy",
        histogram_2d_keys(veh, "x", "y", position_bin_m),
    )
    velocity_entropy = entropy_prefix(
        "velocity_entropy",
        histogram_2d_keys(veh, "vx", "vy", velocity_bin_mps),
    )
    speed_entropy = entropy_prefix(
        "speed_entropy",
        histogram_1d_values(veh["speed_xy"].to_numpy(dtype=float), velocity_bin_mps),
    )
    formation_entropy = entropy_prefix(
        "formation_entropy",
        histogram_fixed_bins(err["error"].to_numpy(dtype=float), FORMATION_ERROR_BIN_EDGES),
        normalization_bins=FORMATION_ERROR_BIN_COUNT,
    )

    prop = calculate_propagation_metrics(err, propagation_threshold, target_drone, attack_start_s)

    latency_series = pd.Series([], dtype=float)
    if "parent_swarm_age_ms" in err.columns:
        latency_series = err["parent_swarm_age_ms"].dropna()
    if len(latency_series) == 0 and "parent_update_age_ms" in err.columns:
        latency_series = err["parent_update_age_ms"].dropna()

    near_collision_sample_count = int((dist["distance"] < collision_threshold).sum()) if len(dist) else 0
    near_collision_episode_count = count_near_collision_episodes(
        dist,
        collision_threshold,
        max_gap_ns=int(episode_gap_s * 1e9),
    )
    near_collision_duration_s = estimate_near_collision_duration_s(
        dist,
        collision_threshold,
        default_sample_period_s=default_sample_period_s,
    )

    summary = {
        "data_dir": data_dir.name,
        "mission_end_source": mission_end_source,
        "mission_duration_s": float(mission_end_ns / 1e9),

        "collision_threshold_m": float(collision_threshold),
        "propagation_threshold_m": float(propagation_threshold),
        "propagation_threshold_source": propagation_threshold_source,
        "target_drone": int(target_drone),
        "attack_start_s": float(attack_start_s),

        "mean_formation_error": float(err["error"].mean()) if len(err) else 0.0,
        "min_formation_error": float(err["error"].min()) if len(err) else 0.0,
        "max_formation_error": float(err["error"].max()) if len(err) else 0.0,
        "p95_formation_error": float(err["error"].quantile(0.95)) if len(err) else 0.0,

        "propagation_factor": prop["propagation_factor"],
        "propagation_severity_psv": prop["propagation_severity_psv"],
        "propagation_speed_drones_per_s": prop["propagation_speed_drones_per_s"],
        "propagation_depth": prop["propagation_depth"],
        "reachable_downstream_count": prop["reachable_downstream_count"],
        "affected_link_fraction": prop["affected_link_fraction"],
        "affected_link_count": prop["affected_link_count"],

        "mean_interdrone_distance": float(dist["distance"].mean()) if len(dist) else 0.0,
        "min_interdrone_distance": float(dist["distance"].min()) if len(dist) else 0.0,
        "near_collision_sample_count": near_collision_sample_count,
        "near_collision_episode_count": near_collision_episode_count,
        "near_collision_duration_s": near_collision_duration_s,

        "mean_latency_ms": safe_mean(latency_series),
        "max_latency_ms": safe_max(latency_series),

        # Shannon-only entropy outputs.
        "position_entropy_shannon": position_entropy["position_entropy_shannon"],
        "position_entropy_shannon_norm": position_entropy["position_entropy_shannon_norm"],
        "velocity_entropy_shannon": velocity_entropy["velocity_entropy_shannon"],
        "velocity_entropy_shannon_norm": velocity_entropy["velocity_entropy_shannon_norm"],
        "speed_entropy_shannon": speed_entropy["speed_entropy_shannon"],
        "speed_entropy_shannon_norm": speed_entropy["speed_entropy_shannon_norm"],
        "formation_entropy_shannon": formation_entropy["formation_entropy_shannon"],
        "formation_entropy_shannon_norm": formation_entropy["formation_entropy_shannon_norm"],

        # Occupied bins are diagnostic and useful for checking whether entropy is saturated.
        "position_entropy_occupied_bins": position_entropy["position_entropy_occupied_bins"],
        "velocity_entropy_occupied_bins": velocity_entropy["velocity_entropy_occupied_bins"],
        "speed_entropy_occupied_bins": speed_entropy["speed_entropy_occupied_bins"],
        "formation_entropy_occupied_bins": formation_entropy["formation_entropy_occupied_bins"],
    }

    summary["mission_impact_score_absolute"] = calculate_absolute_mis(summary)

    if baseline_summary is not None:
        baseline_df = pd.read_csv(baseline_summary)
        if len(baseline_df) == 0:
            raise ValueError(f"Empty baseline summary: {baseline_summary}")

        baseline_row = baseline_df.iloc[0].to_dict()
        baseline_mis, components = calculate_baseline_relative_mis(summary, baseline_row)

        summary["mission_impact_score_baseline_relative"] = baseline_mis
        summary["mission_impact_score"] = baseline_mis
        summary["mission_impact_score_mode"] = "baseline_relative"
        summary.update(components)
    else:
        summary["mission_impact_score_baseline_relative"] = np.nan
        summary["mission_impact_score"] = summary["mission_impact_score_absolute"]
        summary["mission_impact_score_mode"] = "absolute_threshold_no_baseline"

    if write_outputs:
        entropy_windows.to_csv(data_dir / "entropy_windows.csv", index=False)
        pd.DataFrame([summary]).to_csv(data_dir / "metrics_summary.csv", index=False)

        if len(err):
            err.sort_values("mission_time_ns").groupby("drone_id").tail(1).to_csv(
                data_dir / "final_errors.csv",
                index=False,
            )

            err.groupby("hop").agg(
                mean_error=("error", "mean"),
                min_error=("error", "min"),
                max_error=("error", "max"),
                p95_error=("error", lambda x: float(x.quantile(0.95))),
            ).reset_index().to_csv(data_dir / "per_hop_metrics.csv", index=False)

    return summary


# -----------------------------
# Baseline reference generation
# -----------------------------

def make_baseline_reference(
    baseline_dirs: Iterable[Path],
    baseline_out: Path,
    args,
) -> pd.DataFrame:
    baseline_dirs = [Path(d).expanduser().resolve() for d in baseline_dirs]

    # Baseline generation uses the same paper threshold as attack evaluation.
    threshold = args.propagation_threshold if args.propagation_threshold is not None else args.fallback_propagation_threshold
    threshold_source = "explicit_cli" if args.propagation_threshold is not None else "fallback_for_baseline_generation"

    run_summaries = []

    for d in baseline_dirs:
        summary = compute_run_summary(
            data_dir=d,
            window_ms=args.window_ms,
            collision_threshold=args.collision_threshold,
            propagation_threshold=threshold,
            propagation_threshold_source=threshold_source,
            target_drone=args.target_drone,
            attack_start_s=args.attack_start_s,
            position_bin_m=args.position_bin_m,
            velocity_bin_mps=args.velocity_bin_mps,
            episode_gap_s=args.episode_gap_s,
            default_sample_period_s=args.sample_period_s,
            baseline_summary=None,
            write_outputs=True,
        )
        run_summaries.append(summary)

    runs_df = pd.DataFrame(run_summaries)

    numeric_cols = [
        c for c in runs_df.columns
        if pd.api.types.is_numeric_dtype(runs_df[c])
        and not c.startswith("mis_delta_")
    ]

    baseline_mean = runs_df[numeric_cols].mean(numeric_only=True).to_dict()
    baseline_std = {
        f"{c}_std": float(runs_df[c].std(ddof=1)) if len(runs_df) > 1 else 0.0
        for c in numeric_cols
    }

    # Store the same theta_e used during baseline processing.
    recommended_threshold = float(threshold)

    reference = {
        "baseline_run_count": int(len(runs_df)),
        "baseline_dirs": ";".join(d.name for d in baseline_dirs),
        "recommended_propagation_threshold_m": recommended_threshold,
        **baseline_mean,
        **baseline_std,
    }

    baseline_out = Path(baseline_out).expanduser().resolve()
    baseline_out.parent.mkdir(parents=True, exist_ok=True)

    ref_df = pd.DataFrame([reference])
    ref_df.to_csv(baseline_out, index=False)

    runs_out = baseline_out.with_name(baseline_out.stem + "_runs.csv")
    runs_df.to_csv(runs_out, index=False)

    print(f"\nWrote baseline reference: {baseline_out}")
    print(f"Wrote per-baseline-run metrics: {runs_out}")
    print(f"Recommended propagation threshold: {recommended_threshold:.3f} m")

    return ref_df


# -----------------------------
# CLI
# -----------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Paper-oriented metrics for chain/inverted-V UAV swarm runs: "
            "mission-event trimming, near-collision episodes, Shannon-only entropy, "
            "baseline-relative MIS, fixed 8-bin formation entropy, PF/PSV, and paper-aligned thresholds."
        )
    )

    parser.add_argument("--data-dir", help="Single run directory to evaluate.")
    parser.add_argument("--baseline-summary", help="Baseline reference CSV generated with --make-baseline.")

    parser.add_argument("--make-baseline", action="store_true", help="Create a baseline reference from multiple baseline run directories.")
    parser.add_argument("--baseline-dirs", nargs="+", help="Baseline run directories for --make-baseline.")
    parser.add_argument("--baseline-out", default="baseline_reference.csv", help="Output CSV path for --make-baseline.")

    parser.add_argument("--window-ms", type=int, default=500)
    parser.add_argument("--collision-threshold", type=float, default=0.5)

    # Paper default: theta_e = 6 m. An explicit CLI value may override it.
    parser.add_argument("--propagation-threshold", type=float, default=None)
    parser.add_argument("--fallback-propagation-threshold", type=float, default=6.0)

    parser.add_argument("--position-bin-m", type=float, default=2.0)
    parser.add_argument("--velocity-bin-mps", type=float, default=0.5)
    parser.add_argument("--episode-gap-s", type=float, default=0.3)
    parser.add_argument("--sample-period-s", type=float, default=0.1)

    parser.add_argument(
        "--target-drone",
        type=int,
        default=1,
        help="Directly attacked drone used to compute R(a), PF, PD, and PS. Default: 1.",
    )
    parser.add_argument(
        "--attack-start-s",
        type=float,
        default=20.0,
        help="Attack start time in mission seconds used for propagation speed. Default: 20.0.",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    if args.make_baseline:
        if not args.baseline_dirs:
            raise SystemExit("--make-baseline requires --baseline-dirs DIR1 DIR2 ...")

        make_baseline_reference(
            baseline_dirs=[Path(d) for d in args.baseline_dirs],
            baseline_out=Path(args.baseline_out),
            args=args,
        )
        return

    if not args.data_dir:
        raise SystemExit("Either provide --data-dir for a run or use --make-baseline with --baseline-dirs.")

    baseline_summary = Path(args.baseline_summary).expanduser().resolve() if args.baseline_summary else None

    propagation_threshold, threshold_source = resolve_propagation_threshold(
        explicit_threshold=args.propagation_threshold,
        baseline_summary=baseline_summary,
        fallback=args.fallback_propagation_threshold,
    )

    summary = compute_run_summary(
        data_dir=Path(args.data_dir).expanduser().resolve(),
        window_ms=args.window_ms,
        collision_threshold=args.collision_threshold,
        propagation_threshold=propagation_threshold,
        propagation_threshold_source=threshold_source,
        target_drone=args.target_drone,
        attack_start_s=args.attack_start_s,
        position_bin_m=args.position_bin_m,
        velocity_bin_mps=args.velocity_bin_mps,
        episode_gap_s=args.episode_gap_s,
        default_sample_period_s=args.sample_period_s,
        baseline_summary=baseline_summary,
        write_outputs=True,
    )

    print("\n=== Chain / Inverted-V Swarm Metrics ===")
    for k, v in summary.items():
        print(f"{k}: {v}")
    print(f"\nWrote files in: {Path(args.data_dir).expanduser().resolve()}")


if __name__ == "__main__":
    main()
