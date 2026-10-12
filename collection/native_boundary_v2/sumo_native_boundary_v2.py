"""Pure, caller-timed Euler boundary allocation; no TraCI, features or I/O.

The adapter owns SUMO clock calibration, departure/arrival, path identity and
lane geometry. RESOLVED means this *estimated_euler_geometry* contract closes,
not that continuous physical motion or an unobserved entry feature is known.
Distance/speed is never rescaled to conceal a non-closing crossing interval.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Integral, Real
from typing import Optional, Tuple


SCHEMA = "sumo-native-boundary-euler-v2"
QUALITY = "estimated_euler_geometry"
THRESHOLDS = (0.1, 0.5, 1.0, 1.5, 2.0, 3.0)
LABELS = tuple("travel_time_label_gt_" + format(v, "g").replace(".", "p") for v in THRESHOLDS)
ZERO_LABELS = (0.0,) * len(THRESHOLDS)


class UnresolvedReason(str, Enum):
    INVALID_TIME = "invalid_time"
    INVALID_CLIP = "invalid_clip"
    INVALID_SPEED = "invalid_speed"
    INVALID_TOLERANCE = "invalid_tolerance"
    UNSUPPORTED_MODE = "unsupported_update_mode"
    AMBIGUOUS_PATH = "ambiguous_path"
    EMPTY_SEGMENTS = "empty_segments"
    INVALID_SEGMENT = "invalid_segment"
    INVALID_OCCURRENCE_ORDER = "invalid_occurrence_order"
    CROSSING_WITHOUT_POSITIVE_SPEED = "crossing_without_positive_speed"
    ZERO_CROSSING_DISTANCE = "zero_crossing_distance"
    GEOMETRY_MISMATCH = "geometry_not_closed"
    TIME_MISMATCH = "distance_speed_time_not_closed"
    NUMERIC_OVERFLOW = "numeric_overflow"
    TIME_RESOLUTION = "insufficient_timestamp_resolution"
    CONSERVATION_FAILED = "conservation_failed"


@dataclass(frozen=True)
class Segment:
    occurrence: Optional[int]
    road_id: str
    kind: str
    distance: Optional[float]


@dataclass(frozen=True)
class Tolerance:
    """Small numerical tolerances, not a model-error acceptance threshold.

    Caps prevent an accidentally huge tolerance accepting missing seconds.
    A distance check AND an independent time check are required at crossings.
    """
    distance_abs: float = 1e-7
    distance_rel: float = 1e-9
    time_abs: float = 1e-9
    time_rel: float = 1e-9


@dataclass(frozen=True)
class Part:
    occurrence: Optional[int]
    road_id: str
    kind: str
    t_enter: float
    t_exit: float
    time: float
    labels: Tuple[float, ...]
    distance: Optional[float]
    source_segment_index: int


@dataclass(frozen=True)
class Budget:
    full_elapsed: Optional[float]
    elapsed: Optional[float]
    labels: Optional[Tuple[float, ...]]


@dataclass(frozen=True)
class Audit:
    allocated_elapsed: float
    allocated_labels: Tuple[float, ...]
    unresolved_elapsed: Optional[float]
    unresolved_labels: Optional[Tuple[float, ...]]
    elapsed_residual: Optional[float]
    label_residuals: Optional[Tuple[float, ...]]
    geometry_distance: Optional[float]
    expected_distance: Optional[float]
    distance_residual: Optional[float]
    raw_time_residual: Optional[float]


@dataclass(frozen=True)
class AllocationResult:
    status: str
    reason: Optional[UnresolvedReason]
    detail: str
    parts: Tuple[Part, ...]
    budget: Budget
    audit: Audit
    method: str
    quality: str = QUALITY
    schema: str = SCHEMA


def _number(value):
    if not isinstance(value, Real) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except (OverflowError, TypeError, ValueError):
        return False


def _labels(seconds, speed):
    return tuple(seconds if speed > threshold else 0.0 for threshold in THRESHOLDS)


def _tolerance_valid(value):
    if not isinstance(value, Tolerance):
        return False
    limits = ((value.distance_abs, 1e-3), (value.distance_rel, 1e-6),
              (value.time_abs, 1e-4), (value.time_rel, 1e-6))
    return all(_number(number) and 0 <= number <= cap for number, cap in limits)


def allocate_step(*, t0, t1, speed, segments, mode="euler", path_unique=True,
                  clip_start=None, clip_end=None, tolerance=Tolerance()):
    """Allocate one already-calibrated full step, optionally clipped within it.

    `segments` is the ordered path through the FULL [t0,t1] step. Each distance
    is actual longitudinal displacement on that supplied segment, not full road
    length. For one explicitly known owner, elapsed ownership needs no geometry
    inference, including a stopped vehicle; distance may be None if not known.
    For multiple segments, positive Euler speed and distance/speed closure are
    mandatory. Internal segments are retained separately and never donated to
    external roads. Occurrences are supplied, not inferred from edge names.

    Clips must lie inside [t0,t1]. Geometry is checked BEFORE clipping; callers
    must not pass clipped distances with full-step times. Invalid/ambiguous
    inputs return UNRESOLVED with no parts. Known budgets remain unresolved,
    never become zero-valued pseudo-observations. Equal clip bounds return an
    empty, zero-budget resolved allocation after full-step validation.
    """
    budget = Budget(None, None, None)
    geometry_distance = expected_distance = distance_residual = raw_time_residual = None
    method = "unallocated"

    def unresolved(reason, detail):
        return AllocationResult(
            "UNRESOLVED", reason, detail, (), budget,
            Audit(0.0, ZERO_LABELS, budget.elapsed, budget.labels,
                  0.0 if budget.elapsed is not None else None,
                  ZERO_LABELS if budget.labels is not None else None,
                  geometry_distance, expected_distance, distance_residual, raw_time_residual), method)

    if not _number(t0) or not _number(t1) or t1 <= t0:
        return unresolved(UnresolvedReason.INVALID_TIME, "finite t0 < t1 required")
    t0, t1 = float(t0), float(t1)
    dt = t1 - t0
    if not math.isfinite(dt):
        return unresolved(UnresolvedReason.INVALID_TIME, "full interval overflow")
    budget = Budget(dt, None, None)
    begin = t0 if clip_start is None else clip_start
    end = t1 if clip_end is None else clip_end
    if not _number(begin) or not _number(end) or not t0 <= begin <= end <= t1:
        return unresolved(UnresolvedReason.INVALID_CLIP, "clip must satisfy t0 <= start <= end <= t1")
    begin, end = float(begin), float(end)
    budget = Budget(dt, end - begin, None)
    if not _number(speed) or speed < 0:
        return unresolved(UnresolvedReason.INVALID_SPEED, "finite nonnegative Euler speed required")
    speed = float(speed)
    if mode != "euler":
        return unresolved(UnresolvedReason.UNSUPPORTED_MODE, "only caller-confirmed Euler mode is supported")
    budget = Budget(dt, end - begin, _labels(end - begin, speed))
    if not _tolerance_valid(tolerance):
        return unresolved(UnresolvedReason.INVALID_TOLERANCE, "tolerance must be finite, nonnegative and numerically small")
    if path_unique is not True:
        return unresolved(UnresolvedReason.AMBIGUOUS_PATH, "adapter must explicitly confirm a unique ordered path")
    try:
        segments = tuple(segments)
    except TypeError:
        return unresolved(UnresolvedReason.INVALID_SEGMENT, "segments must be an iterable of Segment objects")
    if not segments:
        return unresolved(UnresolvedReason.EMPTY_SEGMENTS, "at least one known owner is required")
    previous_index, previous_road = None, None
    for segment in segments:
        if (not isinstance(segment, Segment) or not isinstance(segment.kind, str)
                or segment.kind not in {"external", "internal"}
                or not isinstance(segment.road_id, str) or not segment.road_id
                or (segment.distance is None and len(segments) != 1)
                or (segment.distance is not None and
                    (not _number(segment.distance) or segment.distance < 0))):
            return unresolved(UnresolvedReason.INVALID_SEGMENT, "invalid segment identity/kind/distance")
        index = segment.occurrence
        if (index is not None and (not isinstance(index, Integral) or isinstance(index, bool) or index < 0)
                or segment.kind == "external" and index is None):
            return unresolved(UnresolvedReason.INVALID_SEGMENT, "external occurrence must be a nonnegative integer")
        if segment.kind == "external":
            if previous_index is not None and (index < previous_index or
                                               index == previous_index and segment.road_id != previous_road):
                return unresolved(UnresolvedReason.INVALID_OCCURRENCE_ORDER, "external occurrence order/identity conflict")
            previous_index, previous_road = index, segment.road_id
    try:
        expected_distance = speed * dt
        if len(segments) != 1 or segments[0].distance is not None:
            geometry_distance = math.fsum(float(s.distance) for s in segments)
            distance_residual = geometry_distance - expected_distance
    except OverflowError:
        return unresolved(UnresolvedReason.NUMERIC_OVERFLOW, "distance sum overflow")
    if not all(v is None or math.isfinite(v) for v in (geometry_distance, expected_distance, distance_residual)):
        return unresolved(UnresolvedReason.NUMERIC_OVERFLOW, "nonfinite distance budget")

    if len(segments) == 1:
        method = "single_owner"
        spans = [(t0, t1, dt, None if segments[0].distance is None else float(segments[0].distance))]
        raw_time_residual = 0.0
    else:
        method = "distance_over_euler_speed"
        if speed <= 0:
            return unresolved(UnresolvedReason.CROSSING_WITHOUT_POSITIVE_SPEED, "crossing needs strictly positive speed")
        if geometry_distance <= 0:
            return unresolved(UnresolvedReason.ZERO_CROSSING_DISTANCE, "crossing needs positive total displacement")
        if not math.isclose(geometry_distance, expected_distance, rel_tol=tolerance.distance_rel,
                            abs_tol=tolerance.distance_abs):
            return unresolved(UnresolvedReason.GEOMETRY_MISMATCH, "sum(distance) differs from speed * full dt")
        durations = tuple(float(s.distance) / speed for s in segments)
        if not all(math.isfinite(v) for v in durations):
            return unresolved(UnresolvedReason.NUMERIC_OVERFLOW, "distance/speed overflow")
        try:
            raw_time_residual = math.fsum(durations) - dt
        except OverflowError:
            return unresolved(UnresolvedReason.NUMERIC_OVERFLOW, "duration sum overflow")
        if not math.isclose(math.fsum(durations), dt, rel_tol=tolerance.time_rel,
                            abs_tol=tolerance.time_abs):
            return unresolved(UnresolvedReason.TIME_MISMATCH, "distance/speed durations differ from full dt; no rescaling")
        spans, elapsed = [], 0.0
        for segment, duration in zip(segments, durations):
            next_elapsed = math.fsum((elapsed, duration))
            start, finish = t0 + elapsed, t0 + next_elapsed
            if not all(math.isfinite(v) for v in (start, finish)) or not math.isclose(
                    finish - start, duration, rel_tol=tolerance.time_rel, abs_tol=tolerance.time_abs):
                return unresolved(UnresolvedReason.TIME_RESOLUTION, "absolute timestamps cannot represent segment duration")
            spans.append((start, finish, duration, float(segment.distance)))
            elapsed = next_elapsed

    parts = []
    if end > begin:
        for index, (segment, (start, finish, duration, distance)) in enumerate(zip(segments, spans)):
            if duration == 0:
                # Preserve a zero-distance occurrence at a numerically rounded
                # clip boundary; no positive time or distance is created.
                if begin <= start <= end:
                    point = start
                elif math.isclose(start, begin, rel_tol=0., abs_tol=tolerance.time_abs):
                    point = begin
                elif math.isclose(start, end, rel_tol=0., abs_tol=tolerance.time_abs):
                    point = end
                else:
                    continue
                parts.append(Part(segment.occurrence, segment.road_id, segment.kind,
                                  point, point, 0., ZERO_LABELS, 0., index))
                continue
            left, right = max(start, begin), min(finish, end)
            if right < left or right == left and duration > 0:
                continue
            seconds = right - left
            # This is explicit interval clipping, never path-wide normalization.
            clipped_distance = None if distance is None else distance * (seconds / duration)
            parts.append(Part(segment.occurrence, segment.road_id, segment.kind,
                              left, right, seconds, _labels(seconds, speed), clipped_distance, index))
    allocated = math.fsum(p.time for p in parts)
    allocated_labels = tuple(math.fsum(p.labels[i] for p in parts) for i in range(len(THRESHOLDS)))
    elapsed_residual = allocated - budget.elapsed
    label_residuals = tuple(value - target for value, target in zip(allocated_labels, budget.labels))
    if not math.isclose(allocated, budget.elapsed, rel_tol=tolerance.time_rel, abs_tol=tolerance.time_abs) or any(
            not math.isclose(value, target, rel_tol=tolerance.time_rel, abs_tol=tolerance.time_abs)
            for value, target in zip(allocated_labels, budget.labels)):
        return unresolved(UnresolvedReason.CONSERVATION_FAILED, "allocated elapsed/threshold totals do not close")
    return AllocationResult(
        "RESOLVED", None, "", tuple(parts), budget,
        Audit(allocated, allocated_labels, 0.0, ZERO_LABELS, elapsed_residual, label_residuals,
              geometry_distance, expected_distance, distance_residual, raw_time_residual), method)


def contract():
    return {
        "schema": SCHEMA, "quality": QUALITY, "thresholds": list(THRESHOLDS),
        "clock": "caller-calibrated t0/t1; no inferred SUMO event offset",
        "speed": "caller-confirmed Euler speed for this full step; strict speed > threshold",
        "crossing": "unique ordered actual segment displacements; distance/speed; no normalization",
        "single_owner": "elapsed has an explicit owner; no geometry-based split, including stopped intervals; unknown distance remains None",
        "clipping": "explicit subinterval within full step; validate full-step geometry first",
        "internal": "separate parts, never absorbed into external roads",
        "unresolved": "typed reason, no parts; preserve available unallocated budgets",
        "features": "none; labels cannot establish an unobserved entry feature",
        "truth_scope": "estimated Euler geometry, not independently proven continuous SUMO truth",
    }
