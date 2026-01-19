from __future__ import annotations

from typing import List, Sequence, Tuple

Point = Tuple[float, float]


def _path_length(points: Sequence[Point]) -> float:
    import math

    if len(points) < 2:
        return 0.0
    L = 0.0
    n = len(points)
    for i in range(n):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % n]
        L += math.hypot(x1 - x0, y1 - y0)
    return L


def resample_closed_polyline(points: Sequence[Point], num_points: int) -> List[Point]:
    """Return num_points evenly spaced points along a closed polyline path."""
    import math

    if not points or num_points <= 1:
        return list(points)

    n = len(points)
    seg_lens: List[float] = []
    cum: List[float] = [0.0]
    total = 0.0
    for i in range(n):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % n]
        d = math.hypot(x1 - x0, y1 - y0)
        seg_lens.append(d)
        total += d
        cum.append(total)

    if total == 0.0:
        return [points[0]] * num_points

    step = total / num_points
    out: List[Point] = []
    si = 0
    for k in range(num_points):
        target_d = k * step
        while si < n and cum[si + 1] < target_d:
            si += 1
        x0, y0 = points[si % n]
        x1, y1 = points[(si + 1) % n]
        seg_start = cum[si]
        seg_len = seg_lens[si % n] if seg_lens[si % n] > 0 else 1e-12
        t = (target_d - seg_start) / seg_len
        t = max(0.0, min(1.0, t))
        x = x0 + (x1 - x0) * t
        y = y0 + (y1 - y0) * t
        out.append((x, y))
    return out


def compute_diameter(points: Sequence[Point]) -> float:
    """Maximum pairwise distance between points, robust diameter for any closed shape."""
    import math

    n = len(points)
    if n < 2:
        return 0.0
    max_d = 0.0
    for i in range(n):
        x0, y0 = points[i]
        for j in range(i + 1, n):
            x1, y1 = points[j]
            d = math.hypot(x1 - x0, y1 - y0)
            if d > max_d:
                max_d = d
    return max_d


def radial_offset_layer(
    prev_layer: Sequence[Point],
    shrink_amount: float,
    min_diameter: float,
    num_points: int | None = None,
) -> Tuple[List[Point], bool]:
    """
    Centroid based radial offset of a closed contour.

    shrink_amount is a diameter change in the same units as points.
    Returns (new_layer, done_flag) where done_flag is True if further
    shrinking is geometrically meaningless given min_diameter.
    """
    import math

    if not prev_layer:
        return [], True

    base = list(prev_layer)
    if num_points is not None and num_points > 1:
        base = resample_closed_polyline(base, num_points)

    d = compute_diameter(base)
    if d <= min_diameter or d <= 1e-9:
        return base, True

    target_d = max(d - shrink_amount, min_diameter)
    if target_d >= d:
        return base, True

    scale = target_d / d
    cx = sum(x for x, _ in base) / len(base)
    cy = sum(y for _, y in base) / len(base)

    out: List[Point] = []
    for x, y in base:
        dx = x - cx
        dy = y - cy
        out.append((cx + dx * scale, cy + dy * scale))

    if num_points is not None and num_points > 1:
        out = resample_closed_polyline(out, num_points)

    done = target_d <= min_diameter + 1e-9
    return out, done

