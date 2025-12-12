#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ILC Real-Time Tracker (Python)
- Real-time visualization with Pygame
- TCP command server (port 8765)
- Shapes: circle, ellipse, square, star
- ILC update: u_{k+1}(t) = u_k(t) + alpha * L * e_k(t)

Controls (via client, ilc_client.py):
  start | stop | reset
  error <0..1>
  lr <0.1..0.8>
  smooth <0.1..1.0>
  noise on|off
  shape circle [r] | ellipse [a b] | square [side] | star [outer inner]
  dome start|stop|reset|status
  plot3d [dz]
  stl [filename]
  status | help
"""

import math
import multiprocessing as mp
import queue as pyqueue
import random
import socket
import threading
from dataclasses import dataclass
from typing import List, Tuple

import pygame  # pip install pygame

from geometry_engine import compute_diameter  # expects (x,y) tuples

# --------------------------------------------------------------------------
# CONFIG & CONSTANTS
# --------------------------------------------------------------------------
WINDOW_SIZE = 800
MARGIN = 60
WORLD_MIN = -1.8
WORLD_MAX = 1.8
NUM_POINTS = 150
SERVER_PORT = 8765

FPS = 60
DT_INDEX = 0.8
MAX_CORRECTION_MAG = 0.1
MAX_DELTA_PER_ITER = 0.1

# Printer / dome geometry units
MM_PER_WORLD = 50.0  # 1 world unit = 50 mm
MIN_DIAMETER_MM = 0.4  # minimum printable diameter in mm (circle dia 0.4 mm)

# Convert min physical diameter -> min physical area -> min world^2 area
MIN_AREA_MM2 = math.pi * (MIN_DIAMETER_MM * 0.5) ** 2
MIN_AREA_WORLD = MIN_AREA_MM2 / (MM_PER_WORLD**2)

# Global "visual" layer spacing in world units
LAYER_DZ_WORLD = 0.05

# History caps to avoid slowdown
MAX_HISTORY = 150
MAX_LAYER_HISTORY = 150


# --------------------------------------------------------------------------
# STL EXPORT HELPERS
# --------------------------------------------------------------------------
def _write_ascii_stl(
    triangles: list[
        tuple[
            tuple[float, float, float],
            tuple[float, float, float],
            tuple[float, float, float],
        ]
    ],
    out_path: str,
    solid_name: str = "ilc_trajectories",
) -> None:
    def _normal(a, b, c):
        ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
        vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
        nx = uy * vz - uz * vy
        ny = uz * vx - ux * vz
        nz = ux * vy - uy * vx
        nlen = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        return (nx / nlen, ny / nlen, nz / nlen)

    with open(out_path, "w", encoding="ascii") as f:
        f.write(f"solid {solid_name}\n")
        for a, b, c in triangles:
            nx, ny, nz = _normal(a, b, c)
            f.write(f"  facet normal {nx} {ny} {nz}\n")
            f.write("    outer loop\n")
            f.write(f"      vertex {a[0]} {a[1]} {a[2]}\n")
            f.write(f"      vertex {b[0]} {b[1]} {b[2]}\n")
            f.write(f"      vertex {c[0]} {c[1]} {c[2]}\n")
            f.write("    endloop\n")
            f.write("  endfacet\n")
        f.write(f"endsolid {solid_name}\n")


def _polyline_strip_triangles(
    layer_pts_xy: list[tuple[float, float]],
    z: float,
    line_w: float,
    thick: float,
) -> list[
    tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]
]:
    """Triangulate a thin rectangular strip around each segment, with top+bottom and side walls."""
    tris = []
    if len(layer_pts_xy) < 2:
        return tris

    z_top = z + 0.5 * thick
    z_bot = z - 0.5 * thick

    def add_quad(p0, p1, p2, p3):
        tris.append((p0, p1, p2))
        tris.append((p0, p2, p3))

    for i in range(len(layer_pts_xy) - 1):
        x0, y0 = layer_pts_xy[i]
        x1, y1 = layer_pts_xy[i + 1]
        dx, dy = x1 - x0, y1 - y0
        L = math.hypot(dx, dy) or 1e-9
        nx, ny = (-dy / L, dx / L)
        ox, oy = 0.5 * line_w * nx, 0.5 * line_w * ny

        t0 = (x0 - ox, y0 - oy, z_top)
        t1 = (x0 + ox, y0 + oy, z_top)
        t2 = (x1 + ox, y1 + oy, z_top)
        t3 = (x1 - ox, y1 - oy, z_top)

        b0 = (x0 - ox, y0 - oy, z_bot)
        b1 = (x0 + ox, y0 + oy, z_bot)
        b2 = (x1 + ox, y1 + oy, z_bot)
        b3 = (x1 - ox, y1 - oy, z_bot)

        add_quad(t0, t1, t2, t3)
        add_quad(b3, b2, b1, b0)

        add_quad(t0, t3, b3, b0)
        add_quad(t1, t0, b0, b1)
        add_quad(t2, t1, b1, b2)
        add_quad(t3, t2, b2, b3)

        if i == 0:
            add_quad(t1, t0, b0, b1)
        if i == len(layer_pts_xy) - 2:
            add_quad(t3, t2, b2, b3)

    return tris


# ------------------------------
# C2: HOLLOW SHELL, CONTINUOUS WALL IN Z
# Side walls only, real thickness via outer+inner walls, no caps
# ------------------------------
def _offset_closed_path(points_xy: list[tuple[float, float]], half_w: float):
    n = len(points_xy)
    if n < 3:
        return points_xy[:], points_xy[:]

    def norm2(x, y):
        L = math.hypot(x, y) or 1e-12
        return x / L, y / L

    outer = []
    inner = []

    for i in range(n):
        x0, y0 = points_xy[(i - 1) % n]
        x1, y1 = points_xy[i]
        x2, y2 = points_xy[(i + 1) % n]

        d1x, d1y = x1 - x0, y1 - y0
        d2x, d2y = x2 - x1, y2 - y1

        d1x, d1y = norm2(d1x, d1y)
        d2x, d2y = norm2(d2x, d2y)

        n1x, n1y = -d1y, d1x
        n2x, n2y = -d2y, d2x

        nx, ny = n1x + n2x, n1y + n2y
        nx, ny = norm2(nx, ny)

        outer.append((x1 + half_w * nx, y1 + half_w * ny))
        inner.append((x1 - half_w * nx, y1 - half_w * ny))

    return outer, inner


def _connect_rings_quads(
    ring0_xy: list[tuple[float, float]],
    z0: float,
    ring1_xy: list[tuple[float, float]],
    z1: float,
    flip: bool,
):
    tris = []
    n = min(len(ring0_xy), len(ring1_xy))
    if n < 3:
        return tris

    def add_quad(p0, p1, p2, p3):
        if not flip:
            tris.append((p0, p1, p2))
            tris.append((p0, p2, p3))
        else:
            tris.append((p0, p2, p1))
            tris.append((p0, p3, p2))

    for i in range(n):
        j = (i + 1) % n

        a0 = (ring0_xy[i][0], ring0_xy[i][1], z0)
        a1 = (ring0_xy[j][0], ring0_xy[j][1], z0)
        b1 = (ring1_xy[j][0], ring1_xy[j][1], z1)
        b0 = (ring1_xy[i][0], ring1_xy[i][1], z1)

        add_quad(a0, a1, b1, b0)

    return tris


def _shell_walls_triangles_between_layers(
    layers_xy: list[list[tuple[float, float]]],
    layers_z: list[float],
    line_w: float,
) -> list[
    tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]
]:
    tris = []
    if len(layers_xy) < 2:
        return tris

    half_w = 0.5 * line_w

    rings_outer = []
    rings_inner = []
    for pts in layers_xy:
        if len(pts) < 3:
            rings_outer.append(pts[:])
            rings_inner.append(pts[:])
            continue
        outer, inner = _offset_closed_path(pts, half_w)
        rings_outer.append(outer)
        rings_inner.append(inner)

    for k in range(len(layers_xy) - 1):
        z0 = layers_z[k]
        z1 = layers_z[k + 1]

        o0, o1 = rings_outer[k], rings_outer[k + 1]
        i0, i1 = rings_inner[k], rings_inner[k + 1]

        if len(o0) >= 3 and len(o1) >= 3:
            tris += _connect_rings_quads(o0, z0, o1, z1, flip=False)

        if len(i0) >= 3 and len(i1) >= 3:
            tris += _connect_rings_quads(i0, z0, i1, z1, flip=True)

    return tris


def export_trajectories_as_stl(
    ilc,
    out_path: str = "ilc_trajectories.stl",
    dz: float = LAYER_DZ_WORLD,
    include_current: bool = True,
    mm_per_unit: float = MM_PER_WORLD,
    line_w_mm: float = 1.2,
    thick_mm: float = 0.6,
):
    completed = ilc.get_completed_paths()
    current = ilc.get_current_traj() if include_current else []

    triangles: list[
        tuple[
            tuple[float, float, float],
            tuple[float, float, float],
            tuple[float, float, float],
        ]
    ] = []

    def w2mm(x):
        return x * mm_per_unit

    layer_z_world = ilc.get_completed_layers_z_world() or []
    dome_start_index = ilc.get_dome_start_index()

    # Build per-layer XY + Z in mm, then connect layers to form continuous shell walls (C2)
    layers_xy_mm: list[list[tuple[float, float]]] = []
    layers_z_mm: list[float] = []

    for k, path in enumerate(completed):
        if len(path) < 3:
            continue

        if layer_z_world and dome_start_index is not None and k >= dome_start_index:
            idx = k - dome_start_index
            if idx < len(layer_z_world):
                z_world = layer_z_world[idx]
            else:
                z_world = (k + 1) * dz
        else:
            z_world = (k + 1) * dz

        layers_z_mm.append(w2mm(z_world))
        layers_xy_mm.append([(w2mm(p.x), w2mm(p.y)) for p in path])

    if len(current) >= 3:
        if (
            layer_z_world
            and dome_start_index is not None
            and len(completed) >= dome_start_index
        ):
            idx = len(completed) - dome_start_index
            if 0 <= idx < len(layer_z_world):
                z_world = layer_z_world[idx]
            else:
                z_world = (len(completed) + 1) * dz
        else:
            z_world = (len(completed) + 1) * dz

        layers_z_mm.append(w2mm(z_world))
        layers_xy_mm.append([(w2mm(p.x), w2mm(p.y)) for p in current])

    # C2: side walls only, continuous in Z, real thickness via outer+inner walls
    triangles += _shell_walls_triangles_between_layers(
        layers_xy=layers_xy_mm,
        layers_z=layers_z_mm,
        line_w=line_w_mm,
    )

    _write_ascii_stl(triangles, out_path, solid_name="ilc_trajectories")
    print(f"[EXPORT] STL written: {out_path}  (triangles: {len(triangles)})")


# --------------------------------------------------------------------------
# DATA TYPES AND PATH UTILS
# --------------------------------------------------------------------------
@dataclass
class Point2D:
    x: float = 0.0
    y: float = 0.0


# 2D VISUALIZER FIX: separate x and y mapping (y inverted for screen coordinates)
def world_to_screen_x(val: float) -> int:
    return int(
        MARGIN
        + (val - WORLD_MIN) / (WORLD_MAX - WORLD_MIN) * (WINDOW_SIZE - 2 * MARGIN)
    )


def world_to_screen_y(val: float) -> int:
    return int(
        WINDOW_SIZE
        - MARGIN
        - (val - WORLD_MIN) / (WORLD_MAX - WORLD_MIN) * (WINDOW_SIZE - 2 * MARGIN)
    )


def world_to_screen(val: float) -> int:
    # kept for compatibility, treat as x-mapper
    return world_to_screen_x(val)


def _interpolate(p: Point2D, q: Point2D, t: float) -> Point2D:
    return Point2D(p.x + (q.x - p.x) * t, p.y + (q.y - p.y) * t)


def resample_closed_path_uniform(
    points: List[Point2D], num_points: int
) -> List[Point2D]:
    if not points or num_points <= 1:
        return points[:]
    n = len(points)
    seg_lens = []
    cum = [0.0]
    total = 0.0
    for i in range(n):
        a, b = points[i], points[(i + 1) % n]
        d = math.hypot(b.x - a.x, b.y - a.y)
        seg_lens.append(d)
        total += d
        cum.append(total)
    step = total / num_points
    out: List[Point2D] = []
    si = 0
    for k in range(num_points):
        target_d = k * step
        while si < n and cum[si + 1] < target_d:
            si += 1
        a = points[si % n]
        b = points[(si + 1) % n]
        seg_start = cum[si]
        seg_len = seg_lens[si % n] if seg_lens[si % n] > 0 else 1e-12
        t = (target_d - seg_start) / seg_len
        out.append(_interpolate(a, b, max(0.0, min(1.0, t))))
    return out


def rotate_path(points: List[Point2D], shift: int) -> List[Point2D]:
    n = len(points)
    shift %= n
    return points[shift:] + points[:shift]


def best_alignment_shift(A: List[Point2D], B: List[Point2D], stride: int = 1) -> int:
    n = len(A)
    best_s, best_cost = 0, float("inf")
    for s in range(0, n, stride):
        cost = 0.0
        for i in range(0, n, stride):
            a, b = A[i], B[(i + s) % n]
            dx, dy = a.x - b.x, a.y - b.y
            cost += dx * dx + dy * dy
        if cost < best_cost:
            best_cost, best_s = cost, s
    return best_s


def align_reference(
    current_ref: List[Point2D], new_ref: List[Point2D]
) -> List[Point2D]:
    n = len(current_ref)
    A = resample_closed_path_uniform(current_ref, n)
    B = resample_closed_path_uniform(new_ref, n)
    shift = best_alignment_shift(A, B, stride=max(1, n // 150))
    return rotate_path(B, shift)


def polygon_area_xy(points_xy: List[Tuple[float, float]]) -> float:
    """Shoelace area for arbitrary closed polygon."""
    n = len(points_xy)
    if n < 3:
        return 0.0
    area = 0.0
    for i in range(n):
        x0, y0 = points_xy[i]
        x1, y1 = points_xy[(i + 1) % n]
        area += x0 * y1 - x1 * y0
    return abs(area) * 0.5


# --------------------------------------------------------------------------
# SHAPE GENERATOR
# --------------------------------------------------------------------------
class ShapeGenerator:
    @staticmethod
    def generate_circle(num_points: int, radius: float = 1.0) -> List[Point2D]:
        pts = []
        for i in range(num_points):
            theta = 2.0 * math.pi * i / num_points
            pts.append(Point2D(radius * math.cos(theta), radius * math.sin(theta)))
        return pts

    @staticmethod
    def generate_ellipse(
        num_points: int, a: float = 1.2, b: float = 0.7
    ) -> List[Point2D]:
        pts = []
        for i in range(num_points):
            theta = 2.0 * math.pi * i / num_points
            pts.append(Point2D(a * math.cos(theta), b * math.sin(theta)))
        return pts

    @staticmethod
    def generate_square(num_points: int, side: float = 1.6) -> List[Point2D]:
        pts: List[Point2D] = []
        pps = num_points // 4
        half = side / 2.0
        for i in range(pps):
            t = i / pps
            pts.append(Point2D(-half + side * t, -half))
        for i in range(pps):
            t = i / pps
            pts.append(Point2D(half, -half + side * t))
        for i in range(pps):
            t = i / pps
            pts.append(Point2D(half - side * t, half))
        for i in range(pps):
            t = i / pps
            pts.append(Point2D(-half, half - side * t))
        while len(pts) < num_points:
            pts.append(pts[-1])
        return pts[:num_points]

    @staticmethod
    def generate_star(
        num_points: int, outer: float = 1.0, inner: float = 0.4
    ) -> List[Point2D]:
        pts: List[Point2D] = []
        peaks = 5
        ppp = max(1, num_points // (peaks * 2))
        for i in range(peaks * 2):
            angle = math.pi * i / peaks
            radius = outer if (i % 2 == 0) else inner
            next_angle = math.pi * (i + 1) / peaks
            next_radius = outer if ((i + 1) % 2 == 0) else inner
            for j in range(ppp):
                t = j / ppp
                r = radius + t * (next_radius - radius)
                a = angle + t * (next_angle - angle)
                pts.append(Point2D(r * math.cos(a), r * math.sin(a)))
        if len(pts) < num_points:
            while len(pts) < num_points:
                pts.append(pts[-1])
        return pts[:num_points]


# --------------------------------------------------------------------------
# ILC CONTROLLER WITH DOME GEOMETRY
# --------------------------------------------------------------------------
class ILCController:
    def __init__(self, n_pts: int, learning_rate: float = 0.4):
        self.num_points = n_pts
        self.learning_rate = learning_rate
        self.system_error_level = 0.0
        self.iteration = 0
        self.enable_noise = False
        self.smoothing_alpha = 0.3
        self.corrections: List[Point2D] = [Point2D(0.0, 0.0) for _ in range(n_pts)]
        self.reference: List[Point2D] = ShapeGenerator.generate_circle(n_pts)
        self.current_traj: List[Point2D] = []
        self.prev_traj: list[Point2D] | None = None
        self.prev_command: list[Point2D] | None = None
        self.nominal_command: list[Point2D] = [
            Point2D(p.x, p.y) for p in self.reference
        ]
        self.correction_base_mode = "previous"
        self.last_errors: List[Point2D] = []
        self.completed_paths: List[List[Point2D]] = []
        self.last_rms_error: float = 0.0
        self.last_mean_abs_error: float = 0.0

        # dome geometry state (spherical dome)
        self.dome_active: bool = False
        self.dome_complete: bool = False
        self.last_observed_diameter: float = 0.0
        self.last_ref_area: float = 0.0

        # spherical dome parameters (in mm + world z)
        self.sphere_R_mm: float = 0.0  # outer sphere radius (mm)
        self.completed_layers_z_mm: List[float] = []  # per-dome-layer z (mm)
        self.completed_layers_z_world: List[float] = []  # same in world units

        # where in completed_paths dome layers start
        self.dome_start_index: int | None = None
        # base height offset so dome sits on top of whatever was printed
        self.dome_base_offset_world: float = 0.0

    @staticmethod
    def _clamp_vec(x: float, y: float, max_mag: float):
        mag = (x * x + y * y) ** 0.5
        if mag > max_mag and max_mag > 0:
            s = max_mag / mag
            return x * s, y * s
        return x, y

    # configuration
    def set_reference(self, ref: List[Point2D]):
        if ref:
            self.reference = (ref + [ref[-1]])[: self.num_points]

    def induce_error(self, level: float):
        self.system_error_level = max(0.0, min(1.0, float(level)))

    def set_learning_rate(self, lr: float):
        self.learning_rate = float(lr)

    def set_smoothing_factor(self, alpha: float):
        self.smoothing_alpha = max(0.1, min(1.0, float(alpha)))

    def set_noise(self, enabled: bool):
        self.enable_noise = bool(enabled)

    def set_correction_base_mode(self, mode: str):
        mode = mode.lower().strip()
        if mode in ("reference", "previous"):
            self.correction_base_mode = mode

    def reset(self):
        self.corrections = [Point2D(0.0, 0.0) for _ in range(self.num_points)]
        self.current_traj.clear()
        self.last_errors.clear()
        self.completed_paths.clear()
        self.iteration = 0
        self.system_error_level = 0.0
        self.last_rms_error = 0.0
        self.last_mean_abs_error = 0.0
        self.prev_traj = None
        self.prev_command = None
        self.nominal_command = [Point2D(p.x, p.y) for p in self.reference]
        self.dome_active = False
        self.dome_complete = False
        self.last_observed_diameter = 0.0
        self.last_ref_area = 0.0
        self.sphere_R_mm = 0.0
        self.completed_layers_z_mm = []
        self.completed_layers_z_world = []
        self.dome_start_index = None
        self.dome_base_offset_world = 0.0

    # getters
    def get_reference(self) -> List[Point2D]:
        return self.reference

    def get_current_traj(self) -> List[Point2D]:
        return self.current_traj

    def get_completed_paths(self) -> List[List[Point2D]]:
        return self.completed_paths

    def get_last_errors(self) -> List[Point2D]:
        return self.last_errors

    def get_iteration(self) -> int:
        return self.iteration

    def get_rms_error(self) -> float:
        return self.last_rms_error

    def get_mean_error(self) -> float:
        return self.last_mean_abs_error

    def get_error_level(self) -> float:
        return self.system_error_level

    def get_noise_enabled(self) -> bool:
        return self.enable_noise

    def get_avg_correction(self) -> float:
        s = 0.0
        for c in self.corrections:
            s += math.hypot(c.x, c.y)
        return s / self.num_points if self.num_points > 0 else 0.0

    def get_last_observed_diameter(self) -> float:
        return self.last_observed_diameter

    def is_dome_complete(self) -> bool:
        return self.dome_complete

    def get_completed_layers_z_world(self) -> List[float]:
        return self.completed_layers_z_world

    def get_dome_start_index(self) -> int | None:
        return self.dome_start_index

    # plant plus step
    def plant_model(self, command: Point2D, path_index: int) -> Point2D:
        out = Point2D(command.x, command.y)
        level = self.system_error_level
        if level > 0.0:
            theta = 2.0 * math.pi * path_index / self.num_points
            radial_error = level * (
                0.25 * math.sin(3 * theta)
                + 0.15 * math.sin(5 * theta)
                + 0.10 * math.cos(2 * theta)
            )
            phase_err = level * (0.18 + 0.02 * math.sin(0.5 * theta + self.iteration))
            contraction = level * (0.12 + 0.03 * math.cos(1.5 * theta))

            radius = math.hypot(out.x, out.y)
            angle = math.atan2(out.y, out.x)

            new_radius = radius * (1 - contraction) + radial_error
            new_angle = angle + phase_err

            out.x = new_radius * math.cos(new_angle)
            out.y = new_radius * math.sin(new_angle)

            if self.enable_noise:
                noise_amp = level * 0.04
                out.x += (random.random() - 0.5) * noise_amp
                out.y += (random.random() - 0.5) * noise_amp

        return out

    def get_current_position(self, path_index: int) -> Point2D:
        if path_index >= self.num_points:
            path_index = self.num_points - 1

        if self.prev_command is not None and len(self.prev_command) == self.num_points:
            base_cmd = self.prev_command[path_index]
        else:
            base_cmd = self.reference[path_index]

        corr = self.corrections[path_index]
        cmd = Point2D(base_cmd.x + corr.x, base_cmd.y + corr.y)

        if len(self.nominal_command) <= path_index:
            self.nominal_command.append(Point2D(cmd.x, cmd.y))
        else:
            self.nominal_command[path_index] = Point2D(cmd.x, cmd.y)

        actual = self.plant_model(cmd, path_index)

        if len(self.current_traj) <= path_index:
            self.current_traj.append(actual)
        else:
            self.current_traj[path_index] = actual
        return actual

    def _compute_ref_centroid_radius(self) -> tuple[float, float, float]:
        """Centroid and max radius of current reference in world units."""
        ref_xy = [(p.x, p.y) for p in self.reference]
        if not ref_xy:
            return 0.0, 0.0, 0.0
        cx = sum(x for x, _ in ref_xy) / len(ref_xy)
        cy = sum(y for _, y in ref_xy) / len(ref_xy)
        r_current = 0.0
        for x, y in ref_xy:
            r = math.hypot(x - cx, y - cy)
            if r > r_current:
                r_current = r
        return cx, cy, r_current

    def _init_sphere_if_needed(self):
        """Initialize spherical dome radius in mm from current reference diameter."""
        if self.sphere_R_mm > 0.0:
            return
        _, _, r_current = self._compute_ref_centroid_radius()
        if r_current <= 0.0:
            return
        # Sphere radius from starting circle radius (world -> mm)
        self.sphere_R_mm = r_current * MM_PER_WORLD

    def _adapt_learning_rate_for_shrink(self, mean_err: float):
        """
        Adapt LR only at shrink events:
        - first shrink and small error -> LR = 0.5
        - otherwise based on error bands.
        """
        if len(self.completed_layers_z_mm) == 1 and mean_err < 0.015:
            self.learning_rate = 0.5
            return

        if mean_err < 0.015:
            self.learning_rate = 0.3
        elif mean_err < 0.04:
            self.learning_rate = 0.2
        else:
            self.learning_rate = 0.1

    def _shrink_reference_if_converged(self):
        """
        Dome shrink logic (spherical):
        - Reference circle lies on a sphere of radius R_mm.
        - We map that sphere vertically, then ADD a base offset so the
          dome starts exactly above whatever has already been printed.
        """

        if not self.dome_active or self.dome_complete:
            return

        ref_xy = [(p.x, p.y) for p in self.reference]
        if len(ref_xy) < 3:
            return

        # Centroid and current radius (world units)
        cx, cy, r_current = self._compute_ref_centroid_radius()
        if r_current <= 0.0:
            self.dome_complete = True
            self.dome_active = False
            return

        # Initialize sphere radius if needed (mm)
        self._init_sphere_if_needed()
        if self.sphere_R_mm <= 0.0:
            # No valid sphere radius, just stop dome logic
            self.dome_complete = True
            self.dome_active = False
            return

        # Diagnostics using current reference
        self.last_observed_diameter = 2.0 * r_current
        area_current_circle = math.pi * r_current * r_current
        self.last_ref_area = area_current_circle

        # --- Compute spherical z for current radius ---
        r_mm_current = r_current * MM_PER_WORLD
        r_mm_current = min(r_mm_current, self.sphere_R_mm)

        z_mm_sphere = math.sqrt(max(self.sphere_R_mm**2 - r_mm_current**2, 0.0))
        z_world_sphere = z_mm_sphere / MM_PER_WORLD

        # --- Layer height logic ---
        if not self.completed_layers_z_world:
            # FIRST dome layer: must begin exactly on top of the printed cylinder
            z_world_current = self.dome_base_offset_world
        else:
            # SUBSEQUENT layers: follow incremental change in sphere Z
            prev_z_world = self.completed_layers_z_world[-1]

            # Sphere z for previous radius (use last observed diameter)
            prev_r_mm = (self.last_observed_diameter * 0.5) * MM_PER_WORLD
            prev_z_mm_sphere = math.sqrt(max(self.sphere_R_mm**2 - prev_r_mm**2, 0.0))
            prev_z_world_sphere = prev_z_mm_sphere / MM_PER_WORLD

            # Δz = spherical_z_now − spherical_z_prev
            dz_world = z_world_sphere - prev_z_world_sphere

            # Add incremental height to previous layer
            z_world_current = prev_z_world + dz_world

        # Convert world z → mm z for STL record
        z_mm_current = z_world_current * MM_PER_WORLD

        # Store layer heights
        self.completed_layers_z_mm.append(z_mm_current)
        self.completed_layers_z_world.append(z_world_current)

        # Cap dome layer history
        if len(self.completed_layers_z_world) > MAX_LAYER_HISTORY:
            self.completed_layers_z_world = self.completed_layers_z_world[
                -MAX_LAYER_HISTORY:
            ]
        if len(self.completed_layers_z_mm) > MAX_LAYER_HISTORY:
            self.completed_layers_z_mm = self.completed_layers_z_mm[-MAX_LAYER_HISTORY:]

        # If current circle area already below threshold, dome is done
        if area_current_circle <= MIN_AREA_WORLD:
            self.dome_complete = True
            self.dome_active = False
            return

        # Shrink only after convergence
        mean_err = self.last_mean_abs_error
        if mean_err >= 0.01:
            # Not converged enough; keep same reference radius, only accumulated a layer
            return

        # Adapt learning rate at shrink events
        self._adapt_learning_rate_for_shrink(mean_err)

        # Fixed radius decrement in mm; as r decreases, dz naturally shrinks.
        delta_r_mm = max(self.sphere_R_mm / 25.0, 0.5)
        r_mm_new = max(r_mm_current - delta_r_mm, 0.0)

        # New radius in world units
        r_new_world = r_mm_new / MM_PER_WORLD

        # If new radius too small, or area below threshold, stop dome
        area_new_circle = math.pi * r_new_world * r_new_world
        if area_new_circle <= MIN_AREA_WORLD or r_new_world <= 0.0:
            self.dome_complete = True
            self.dome_active = False
            return

        self.last_ref_area = area_new_circle
        self.last_observed_diameter = 2.0 * r_new_world

        # Build new circular reference around same centroid
        new_ref: List[Point2D] = []
        for i in range(self.num_points):
            theta = 2.0 * math.pi * i / self.num_points
            x = cx + r_new_world * math.cos(theta)
            y = cy + r_new_world * math.sin(theta)
            new_ref.append(Point2D(x, y))

        self.reference = new_ref

    def complete_iteration(self) -> float:
        if len(self.current_traj) < self.num_points:
            return self.last_rms_error

        # trim to exactly num_points and enforce closed loop
        if len(self.current_traj) > self.num_points:
            self.current_traj = self.current_traj[: self.num_points]
        if self.current_traj:
            first = self.current_traj[0]
            self.current_traj[-1] = Point2D(first.x, first.y)

        tracking_errors: List[Point2D] = []
        total_err_sq = 0.0
        total_abs = 0.0
        for i in range(self.num_points):
            ref = self.reference[i]
            act = self.current_traj[i]
            err = Point2D(ref.x - act.x, ref.y - act.y)
            tracking_errors.append(err)
            e_mag = math.hypot(err.x, err.y)
            total_abs += e_mag
            total_err_sq += err.x * err.x + err.y * err.y

        rms = math.sqrt(total_err_sq / self.num_points)
        self.last_rms_error = rms
        self.last_mean_abs_error = (
            total_abs / self.num_points if self.num_points > 0 else 0.0
        )

        for i in range(min(self.num_points, len(tracking_errors))):
            d_x = self.learning_rate * tracking_errors[i].x
            d_y = self.learning_rate * tracking_errors[i].y

            eff_dx = self.smoothing_alpha * d_x
            eff_dy = self.smoothing_alpha * d_y

            eff_dx, eff_dy = self._clamp_vec(eff_dx, eff_dy, MAX_DELTA_PER_ITER)

            if self.correction_base_mode == "previous":
                self.corrections[i].x = eff_dx
                self.corrections[i].y = eff_dy
            else:
                self.corrections[i].x += eff_dx
                self.corrections[i].y += eff_dy

            mag = math.hypot(self.corrections[i].x, self.corrections[i].y)
            if mag > MAX_CORRECTION_MAG:
                scale = MAX_CORRECTION_MAG / mag
                self.corrections[i].x *= scale
                self.corrections[i].y *= scale

        self.prev_command = [Point2D(p.x, p.y) for p in self.nominal_command]
        self.nominal_command = []

        self.iteration += 1
        self.last_errors = [Point2D(e.x, e.y) for e in tracking_errors]
        self.completed_paths.append(self.current_traj[:])
        self.prev_traj = self.current_traj[:]
        self.current_traj = []

        # Limit stored trajectory history
        if len(self.completed_paths) > MAX_HISTORY:
            self.completed_paths = self.completed_paths[-MAX_HISTORY:]

        # Dome logic: spherical shrink only when converged and dome is active
        if self.dome_active and not self.dome_complete:
            self._shrink_reference_if_converged()

        return rms


# --------------------------------------------------------------------------
# VISUALIZER (Pygame)
# --------------------------------------------------------------------------
class Visualizer:
    def __init__(self):
        pygame.init()
        self.surface = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
        pygame.display.set_caption("ILC Real-Time Tracker (Python)")
        self.clock = pygame.time.Clock()

        self.COLORS = {
            "bg": (255, 255, 255),
            "grid": (240, 240, 240),
            "ref": (34, 197, 94),
            "traj": (37, 99, 235),
            "err": (220, 38, 38),
            "text": (0, 0, 0),
        }
        self.font = pygame.font.SysFont("Arial", 16)

    def clear(self):
        self.surface.fill(self.COLORS["bg"])

    def draw_grid(self):
        color = self.COLORS["grid"]
        for val in [-1.5, -1.0, -0.5, 0, 0.5, 1.0, 1.5]:
            px = world_to_screen_x(val)
            py = world_to_screen_y(val)
            pygame.draw.line(
                self.surface, color, (px, MARGIN), (px, WINDOW_SIZE - MARGIN), 1
            )
            pygame.draw.line(
                self.surface, color, (MARGIN, py), (WINDOW_SIZE - MARGIN, py), 1
            )

        pygame.draw.rect(
            self.surface,
            (200, 200, 200),
            pygame.Rect(
                MARGIN, MARGIN, WINDOW_SIZE - 2 * MARGIN, WINDOW_SIZE - 2 * MARGIN
            ),
            1,
        )

    def draw_path(
        self, path: List[Point2D], color: Tuple[int, int, int], width: int, closed=True
    ):
        if len(path) < 2:
            return
        pts = [(world_to_screen_x(p.x), world_to_screen_y(p.y)) for p in path]
        pygame.draw.lines(self.surface, color, closed, pts, width)

    def draw_robot(self, pos: Point2D, color: Tuple[int, int, int], radius=10):
        x, y = world_to_screen_x(pos.x), world_to_screen_y(pos.y)
        pygame.draw.circle(self.surface, color, (x, y), radius)
        pygame.draw.circle(self.surface, (255, 255, 255), (x, y), radius, 2)

    def draw_text(self, x: int, y: int, text: str):
        img = self.font.render(text, True, self.COLORS["text"])
        self.surface.blit(img, (x, y))

    def flip(self):
        pygame.display.flip()

    def tick(self, fps=FPS):
        self.clock.tick(fps)


# --------------------------------------------------------------------------
# 3D WORKER AND MANAGER
# --------------------------------------------------------------------------
def plot3d_worker(data_q: mp.Queue, cmd_q: mp.Queue):
    import matplotlib
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    plt.ion()
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    try:
        fig.canvas.manager.set_window_title("ILC 3D Trajectories")
    except Exception:
        pass

    def redraw(payload):
        ax.clear()
        ref = payload["ref"]
        completed = payload["completed"]
        current = payload["current"]
        dz = float(payload["dz"])
        errlvl = float(payload["error_level"])
        layer_z_world = payload.get("layer_z_world", None)
        dome_start_index = payload.get("dome_start_index", None)

        ax.set_title("ILC Trajectories (z = layer height or iteration × dz)")

        if ref:
            rx, ry = zip(*ref)
            ax.plot(
                rx, ry, [0.0] * len(ref), color="#22c55e", lw=2.2, label="Reference"
            )

        for k, path in enumerate(completed):
            if len(path) < 2:
                continue
            x, y = zip(*path)

            # Decide z for this layer
            if layer_z_world and dome_start_index is not None and k >= dome_start_index:
                idx = k - dome_start_index
                if 0 <= idx < len(layer_z_world):
                    z_val = layer_z_world[idx]
                else:
                    z_val = (k + 1) * dz
            else:
                z_val = (k + 1) * dz

            z = [z_val] * len(path)
            color = "#2563eb" if errlvl == 0 else "#dc2626"
            ax.plot(x, y, z, color=color, lw=1.8, alpha=0.95)

        if len(current) >= 2:
            x, y = zip(*current)

            if (
                layer_z_world
                and dome_start_index is not None
                and len(completed) >= dome_start_index
            ):
                idx = len(completed) - dome_start_index
                if 0 <= idx < len(layer_z_world):
                    z_val = layer_z_world[idx]
                else:
                    z_val = (len(completed) + 1) * dz
            else:
                z_val = (len(completed) + 1) * dz

            z = [z_val] * len(current)
            ax.plot(x, y, z, color="#f59e0b", lw=2.1, alpha=0.95, label="Current")

        ax.set_xlabel("x (world)")
        ax.set_ylabel("y (world)")
        ax.set_zlabel("z (world)")
        ax.set_xlim(WORLD_MIN, WORLD_MAX)
        ax.set_ylim(WORLD_MIN, WORLD_MAX)

        # Correct z-limits
        if layer_z_world and len(layer_z_world) > 0:
            zmin = 0.0
            zmax_dome = max(layer_z_world)
            max_fallback_z = (len(completed) + (1 if len(current) >= 2 else 0)) * dz
            zmax = 1.05 * max(zmax_dome, max_fallback_z)
        else:
            zmin = 0.0
            zmax = (len(completed) + (1 if len(current) >= 2 else 0) + 1) * dz

        ax.set_zlim(zmin, max(dz, zmax))
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.canvas.draw_idle()

    try:
        plt.show(block=False)
    except Exception:
        pass

    while True:
        try:
            msg = cmd_q.get_nowait()
            if msg == "quit":
                break
        except pyqueue.Empty:
            pass

        payload = None
        try:
            payload = data_q.get(timeout=0.05)
            while True:
                payload = data_q.get_nowait()
        except pyqueue.Empty:
            pass

        if payload is not None:
            redraw(payload)

        plt.pause(0.01)

    plt.close(fig)


class Plot3DManager:
    def __init__(self, default_dz: float = LAYER_DZ_WORLD):
        self.proc: mp.Process | None = None
        self.data_q: mp.Queue | None = None
        self.cmd_q: mp.Queue | None = None
        self.dz = default_dz

    def start(self) -> None:
        if self.proc is not None and self.proc.is_alive():
            return
        self.data_q = mp.Queue(maxsize=8)
        self.cmd_q = mp.Queue(maxsize=4)
        self.proc = mp.Process(
            target=plot3d_worker, args=(self.data_q, self.cmd_q), daemon=True
        )
        self.proc.start()

    def stop(self) -> None:
        if self.proc is None:
            return
        try:
            if self.cmd_q is not None:
                self.cmd_q.put("quit", block=False)
        except Exception:
            pass
        self.proc.join(timeout=1.0)
        if self.proc.is_alive():
            self.proc.terminate()
        self.proc = None
        self.data_q = None
        self.cmd_q = None

    def is_running(self) -> bool:
        return self.proc is not None and self.proc.is_alive()

    def publish(
        self,
        ref_xy,
        completed_xy,
        current_xy,
        iteration: int,
        error_level: float,
        dz: float | None = None,
        layer_z_world: list[float] | None = None,
        dome_start_index: int | None = None,
    ):
        if not self.is_running():
            return
        payload = {
            "ref": ref_xy,
            "completed": completed_xy,
            "current": current_xy,
            "dz": self.dz if dz is None else float(dz),
            "iteration": int(iteration),
            "error_level": float(error_level),
            "layer_z_world": layer_z_world,
            "dome_start_index": dome_start_index,
        }
        try:
            if self.data_q.full():
                _ = self.data_q.get_nowait()
            self.data_q.put_nowait(payload)
        except Exception:
            pass


# --------------------------------------------------------------------------
# COMMAND SERVER
# --------------------------------------------------------------------------
class CommandServer(threading.Thread):
    def __init__(
        self,
        ilc: ILCController,
        run_flag: threading.Event,
        lock: threading.Lock,
        plot3d_mgr: Plot3DManager,
    ):
        super().__init__(daemon=True)
        self.ilc = ilc
        self.run_flag = run_flag
        self.lock = lock
        self._stop_event = threading.Event()
        self.plot3d_mgr = plot3d_mgr

    def run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("0.0.0.0", SERVER_PORT))
            s.listen(1)
            print(f"[SERVER] Listening on port {SERVER_PORT}")
            s.settimeout(1.0)
            while not self._stop_event.is_set():
                try:
                    client, addr = s.accept()
                except socket.timeout:
                    continue
                print("[SERVER] Client connected:", addr)
                try:
                    self.handle_client(client)
                finally:
                    client.close()

    def stop(self):
        self._stop_event.set()

    def handle_client(self, client_sock: socket.socket):
        client_sock.settimeout(1.0)
        buffer = b""
        while not self._stop_event.is_set():
            try:
                data = client_sock.recv(1024)
                if not data:
                    break
                buffer += data
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    cmd = line.decode("utf-8", errors="ignore").strip()
                    resp = self.process_command(cmd)
                    client_sock.sendall(resp.encode("utf-8"))
            except socket.timeout:
                continue
            except OSError:
                break

    def process_command(self, cmd: str) -> str:
        with self.lock:
            parts = cmd.strip().split()
            if not parts:
                return "ERROR: Empty command\n"
            action = parts[0].lower()

            if action == "start":
                if self.ilc.is_dome_complete():
                    return "ERROR: Dome already complete, reset or change shape first\n"
                self.run_flag.set()
                return "OK: Simulation started\n"

            if action == "stop":
                self.run_flag.clear()
                return "OK: Simulation stopped\n"

            if action == "reset":
                self.ilc.reset()
                return "OK: ILC reset\n"

            if action == "error":
                if len(parts) < 2:
                    return "ERROR: usage: error <level>\n"
                try:
                    level = float(parts[1])
                except ValueError:
                    return "ERROR: invalid number\n"
                self.ilc.induce_error(level)
                return f"OK: Induced error set to {level}\n"

            if action == "lr":
                if len(parts) < 2:
                    return "ERROR: usage: lr <rate>\n"
                try:
                    lr = float(parts[1])
                except ValueError:
                    return "ERROR: invalid number\n"
                self.ilc.set_learning_rate(lr)
                return f"OK: Learning rate set to {lr}\n"

            if action == "smooth":
                if len(parts) < 2:
                    return "ERROR: usage: smooth <alpha>\n"
                try:
                    alpha = float(parts[1])
                except ValueError:
                    return "ERROR: invalid number\n"
                self.ilc.set_smoothing_factor(alpha)
                return f"OK: Smoothing factor set to {alpha}\n"

            if action == "noise":
                if len(parts) < 2:
                    return "ERROR: usage: noise on|off\n"
                state = parts[1].lower()
                enabled = state in ("on", "1", "true", "yes")
                self.ilc.set_noise(enabled)
                return "OK: Noise {}\n".format("enabled" if enabled else "disabled")

            if action == "shape":
                if len(parts) < 2:
                    return "ERROR: usage: shape <circle|ellipse|square|star> [params]\n"
                shape = parts[1].lower()

                # Optional lr override: accept lr=0.3, lr = 0.3, or ["lr","=","0.3"]
                tokens = [t.strip().lower() for t in parts[2:]]
                lr_override = None

                for t in tokens:
                    if t.startswith("lr="):
                        try:
                            lr_override = float(t.split("=", 1)[1])
                        except Exception:
                            pass
                        break

                if lr_override is None:
                    for i in range(len(tokens) - 2):
                        if tokens[i] == "lr" and tokens[i + 1] == "=":
                            try:
                                lr_override = float(tokens[i + 2])
                            except Exception:
                                pass
                            break

                if shape == "circle":
                    radius = float(parts[2]) if len(parts) >= 3 else 1.0
                    raw = ShapeGenerator.generate_circle(NUM_POINTS, radius)
                elif shape == "ellipse":
                    a = float(parts[2]) if len(parts) >= 3 else 1.2
                    b = float(parts[3]) if len(parts) >= 4 else 0.7
                    raw = ShapeGenerator.generate_ellipse(NUM_POINTS, a, b)
                elif shape == "square":
                    side = float(parts[2]) if len(parts) >= 3 else 1.6
                    raw = ShapeGenerator.generate_square(NUM_POINTS, side)
                elif shape == "star":
                    outer = float(parts[2]) if len(parts) >= 3 else 1.0
                    inner = float(parts[3]) if len(parts) >= 4 else 0.4
                    raw = ShapeGenerator.generate_star(NUM_POINTS, outer, inner)
                else:
                    return "ERROR: Unknown shape type\n"

                cur = self.ilc.get_reference()
                aligned = align_reference(cur, raw)
                self.ilc.set_reference(aligned)
                # Reset dome / sphere state when shape changes
                self.ilc.dome_active = False
                self.ilc.dome_complete = False
                self.ilc.last_observed_diameter = 0.0
                self.ilc.last_ref_area = 0.0
                self.ilc.sphere_R_mm = 0.0
                self.ilc.completed_layers_z_mm = []
                self.ilc.completed_layers_z_world = []
                self.ilc.dome_start_index = None
                self.ilc.dome_base_offset_world = 0.0

                # Default LR for shape commands is 0.1, unless overridden via lr=<v>
                if lr_override is not None:
                    self.ilc.set_learning_rate(lr_override)
                    return (
                        f"OK: Shape switched to {shape} with alignment, "
                        f"learning rate overridden to {lr_override}\n"
                    )
                else:
                    self.ilc.set_learning_rate(0.1)
                    return (
                        f"OK: Shape switched to {shape} with alignment, "
                        "learning rate set to 0.1\n"
                    )

            if action == "dome":
                if len(parts) < 2:
                    return "ERROR: usage: dome <start|stop|reset|status>\n"
                sub = parts[1].lower()

                # Optional lr override for 'dome start'
                lr_override = None
                tokens = [t.strip().lower() for t in parts[2:]]

                for t in tokens:
                    if t.startswith("lr="):
                        try:
                            lr_override = float(t.split("=", 1)[1])
                        except ValueError:
                            pass
                        break

                if lr_override is None:
                    for i in range(len(tokens) - 2):
                        if tokens[i] == "lr" and tokens[i + 1] == "=":
                            try:
                                lr_override = float(tokens[i + 2])
                            except Exception:
                                pass
                            break

                if sub == "start":
                    if self.ilc.is_dome_complete():
                        return "ERROR: Dome already complete, reset or change shape first\n"
                    self.ilc.dome_active = True
                    self.ilc.dome_complete = False
                    self.ilc.last_observed_diameter = 0.0
                    self.ilc.last_ref_area = 0.0
                    self.ilc.sphere_R_mm = 0.0
                    self.ilc.completed_layers_z_mm = []
                    self.ilc.completed_layers_z_world = []
                    # dome layers start after current completed_paths
                    self.ilc.dome_start_index = len(self.ilc.get_completed_paths())
                    # base offset so dome is stacked on top of existing layers
                    self.ilc.dome_base_offset_world = (
                        self.ilc.dome_start_index * LAYER_DZ_WORLD
                    )
                    # Default LR for dome start is 0.1, unless overridden via lr=<v>
                    if lr_override is not None:
                        self.ilc.set_learning_rate(lr_override)
                        return (
                            "OK: Dome mode started (spherical reference shrinks after convergence), "
                            f"learning rate overridden to {lr_override}\n"
                        )
                    else:
                        self.ilc.set_learning_rate(0.1)
                        return (
                            "OK: Dome mode started (spherical reference shrinks after convergence), "
                            "learning rate set to 0.1\n"
                        )
                if sub == "stop":
                    self.ilc.dome_active = False
                    self.run_flag.clear()
                    return "OK: Dome mode stopped and simulation paused\n"
                if sub == "reset":
                    self.ilc.dome_active = False
                    self.ilc.dome_complete = False
                    self.ilc.last_observed_diameter = 0.0
                    self.ilc.last_ref_area = 0.0
                    self.ilc.sphere_R_mm = 0.0
                    self.ilc.completed_layers_z_mm = []
                    self.ilc.completed_layers_z_world = []
                    self.ilc.dome_start_index = None
                    self.ilc.dome_base_offset_world = 0.0
                    return "OK: Dome state reset (shape and ILC unchanged)\n"
                if sub == "status":
                    state = (
                        "active"
                        if self.ilc.dome_active
                        else ("complete" if self.ilc.dome_complete else "off")
                    )
                    d = self.ilc.get_last_observed_diameter()
                    area = self.ilc.last_ref_area
                    R_mm = self.ilc.sphere_R_mm
                    return (
                        f"Dome: {state}, "
                        f"reference diameter={d:.6f} (world), "
                        f"area={area:.8f} (world^2), "
                        f"min_area={MIN_AREA_WORLD:.8f}, "
                        f"sphere_R={R_mm:.3f} mm\n"
                    )
                return "ERROR: unknown dome command\n"

            if action == "status":
                s = []
                s.append(f"Iteration: {self.ilc.get_iteration()}")
                s.append(f"RMS Error: {self.ilc.get_rms_error():.6f}")
                s.append(f"Mean Error: {self.ilc.get_mean_error():.6f}")
                s.append(f"Induced Error: {self.ilc.get_error_level() * 100:.0f}%")
                s.append(f"Noise: {'ON' if self.ilc.get_noise_enabled() else 'OFF'}")
                s.append(f"Avg Correction: {self.ilc.get_avg_correction():.6f}")
                if self.ilc.get_last_observed_diameter() > 0.0:
                    s.append(
                        f"Reference diameter: {self.ilc.get_last_observed_diameter():.6f} (world)"
                    )
                if self.ilc.last_ref_area > 0.0:
                    s.append(
                        f"Reference area: {self.ilc.last_ref_area:.8f} (world^2), "
                        f"min_area={MIN_AREA_WORLD:.8f}"
                    )
                dome_state = (
                    "active"
                    if self.ilc.dome_active
                    else ("complete" if self.ilc.dome_complete else "off")
                )
                s.append(f"Dome: {dome_state}")
                s.append(f"Running: {'YES' if self.run_flag.is_set() else 'NO'}")
                return "\n".join(s) + "\n"

            if action == "plot3d":
                dz = LAYER_DZ_WORLD
                if len(parts) >= 2:
                    try:
                        dz = float(parts[1])
                    except ValueError:
                        pass
                ref_xy = [(p.x, p.y) for p in self.ilc.get_reference()]
                completed_xy = [
                    [(p.x, p.y) for p in path]
                    for path in self.ilc.get_completed_paths()
                ]
                current_xy = [(p.x, p.y) for p in self.ilc.get_current_traj()]
                it = self.ilc.get_iteration()
                errlvl = self.ilc.get_error_level()
                layer_z_world = self.ilc.get_completed_layers_z_world()
                dome_start_index = self.ilc.get_dome_start_index()
                if not self.plot3d_mgr.is_running():
                    self.plot3d_mgr.dz = dz
                    self.plot3d_mgr.start()
                self.plot3d_mgr.publish(
                    ref_xy,
                    completed_xy,
                    current_xy,
                    it,
                    errlvl,
                    dz=dz,
                    layer_z_world=layer_z_world,
                    dome_start_index=dome_start_index,
                )
                return f"OK: 3-D plot running (dz={dz})\n"

            if action == "stl":
                if len(parts) < 2:
                    out_filename = "ilc_trajectories.stl"
                else:
                    out_filename = parts[1]
                export_trajectories_as_stl(
                    self.ilc,
                    out_path=out_filename,
                    dz=LAYER_DZ_WORLD,
                    include_current=False,
                    mm_per_unit=MM_PER_WORLD,
                    line_w_mm=1.2,
                    thick_mm=0.6,
                )
                return f"STL output requested (file:{out_filename})\n"

            if action == "help":
                return (
                    "Commands:\n"
                    " start\n"
                    " stop\n"
                    " reset\n"
                    " error <level>\n"
                    " lr <rate>\n"
                    " smooth <alpha>\n"
                    " noise <on|off>\n"
                    " shape <circle|ellipse|square|star> [params]\n"
                    " dome <start|stop|reset|status>\n"
                    " status\n"
                    " plot3d [dz]\n"
                    " stl [filename]\n"
                )

            return "ERROR: Unknown command\n"


# --------------------------------------------------------------------------
# SNAPSHOT 3D PLOT (manual key P)
# --------------------------------------------------------------------------
def plot_3d_paths(ilc, dz=LAYER_DZ_WORLD, include_current=True):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    ref = ilc.get_reference()
    completed = ilc.get_completed_paths()
    current = ilc.get_current_traj()
    layer_z_world = ilc.get_completed_layers_z_world() or []
    dome_start_index = ilc.get_dome_start_index()

    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("ILC Trajectories over Iterations (spherical dome aware)")

    rx = [p.x for p in ref]
    ry = [p.y for p in ref]
    rz = [0.0] * len(ref)
    ax.plot(rx, ry, rz, color="#22c55e", linewidth=2.5, label="Reference")

    for k, path in enumerate(completed):
        if len(path) < 2:
            continue
        x = [p.x for p in path]
        y = [p.y for p in path]

        if layer_z_world and dome_start_index is not None and k >= dome_start_index:
            idx = k - dome_start_index
            if 0 <= idx < len(layer_z_world):
                z_val = layer_z_world[idx]
            else:
                z_val = (k + 1) * dz
        else:
            z_val = (k + 1) * dz

        z = [z_val] * len(path)
        color = "#2563eb" if ilc.get_error_level() == 0 else "#dc2626"
        ax.plot(x, y, z, color=color, linewidth=1.8, alpha=0.9)

    if include_current and len(current) >= 2:
        x = [p.x for p in current]
        y = [p.y for p in current]

        if (
            layer_z_world
            and dome_start_index is not None
            and len(completed) >= dome_start_index
        ):
            idx = len(completed) - dome_start_index
            if 0 <= idx < len(layer_z_world):
                z_val = layer_z_world[idx]
            else:
                z_val = (len(completed) + 1) * dz
        else:
            z_val = (len(completed) + 1) * dz

        z = [z_val] * len(current)
        ax.plot(x, y, z, color="#f59e0b", linewidth=2.2, alpha=0.9, label="Current")

    ax.set_xlabel("x (world)")
    ax.set_ylabel("y (world)")
    ax.set_zlabel("z (world)")
    ax.set_xlim(WORLD_MIN, WORLD_MAX)
    ax.set_ylim(WORLD_MIN, WORLD_MAX)

    # -------- NEW CORRECT Z-LIMIT LOGIC --------
    zmin = 0.0

    # Height for pre-dome (normal cylinder)
    if dome_start_index is None:
        num_cyl_layers = len(completed)  # all layers are cylinder
    else:
        num_cyl_layers = min(dome_start_index, len(completed))

    max_cyl_height = num_cyl_layers * dz

    # Dome height (if dome layers exist)
    max_dome_height = max(layer_z_world) if layer_z_world else 0.0

    # True printed height
    zmax_true = max(max_cyl_height, max_dome_height)

    # Add 5% padding
    zmax = 1.05 * max(zmax_true, dz)

    ax.set_zlim(zmin, zmax)
    # -------------------------------------------

    ax.legend(loc="upper right")
    ax.grid(True, which="both", alpha=0.3)
    plt.tight_layout()
    plt.show()


# --------------------------------------------------------------------------
# MAIN LOOP
# --------------------------------------------------------------------------
def main():
    ilc = ILCController(NUM_POINTS, learning_rate=0.4)
    viz = Visualizer()
    run_flag = threading.Event()
    lock = threading.Lock()
    plot3d_mgr = Plot3DManager(default_dz=LAYER_DZ_WORLD)

    server = CommandServer(ilc, run_flag, lock, plot3d_mgr)
    server.start()

    print("[READY] Pygame window opened")
    print(f"[READY] Command server listening on port {SERVER_PORT}")
    print("[READY] Use ilc_client.py to control the simulation")

    path_index_time = 0.0
    robot_pos = Point2D(1.0, 0.0)
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_p:
                with lock:
                    plot_3d_paths(ilc, dz=LAYER_DZ_WORLD, include_current=True)

        snap_for_3d = None

        with lock:
            if run_flag.is_set():
                path_index_time += DT_INDEX
                idx = int(path_index_time) % NUM_POINTS
                robot_pos = ilc.get_current_position(idx)

                if (
                    idx == 0
                    and path_index_time > NUM_POINTS
                    and len(ilc.get_current_traj()) >= NUM_POINTS
                ):
                    ilc.complete_iteration()
                    ref_xy = [(p.x, p.y) for p in ilc.get_reference()]
                    completed_xy = [
                        [(p.x, p.y) for p in path] for path in ilc.get_completed_paths()
                    ]
                    current_xy = [(p.x, p.y) for p in ilc.get_current_traj()]
                    it = ilc.get_iteration()
                    errlvl = ilc.get_error_level()
                    layer_z_world = ilc.get_completed_layers_z_world()
                    dome_start_index = ilc.get_dome_start_index()

                    if ilc.is_dome_complete() and run_flag.is_set():
                        run_flag.clear()
                        print(
                            "[AUTO-STOP] Dome complete at iteration "
                            f"{it}, diameter={ilc.get_last_observed_diameter():.6f} (world)"
                        )

                    snap_for_3d = (
                        ref_xy,
                        completed_xy,
                        current_xy,
                        it,
                        errlvl,
                        layer_z_world,
                        dome_start_index,
                    )
                    path_index_time = 0.0

        if snap_for_3d is not None:
            (
                ref_xy,
                completed_xy,
                current_xy,
                it,
                errlvl,
                layer_z_world,
                dome_start_index,
            ) = snap_for_3d
            if not plot3d_mgr.is_running():
                plot3d_mgr.start()
            plot3d_mgr.publish(
                ref_xy,
                completed_xy,
                current_xy,
                it,
                errlvl,
                dz=LAYER_DZ_WORLD,
                layer_z_world=layer_z_world,
                dome_start_index=dome_start_index,
            )

        viz.clear()
        viz.draw_grid()

        with lock:
            viz.draw_path(ilc.get_reference(), viz.COLORS["ref"], 3, closed=True)
            color_paths = (
                viz.COLORS["err"] if ilc.get_error_level() > 0 else viz.COLORS["traj"]
            )
            for path in ilc.get_completed_paths():
                viz.draw_path(path, color_paths, 2, closed=True)
            cur = ilc.get_current_traj()
            if len(cur) > 2:
                viz.draw_path(cur, color_paths, 3, closed=False)
            if run_flag.is_set():
                viz.draw_robot(robot_pos, color_paths, 10)
            status = (
                f"Iteration: {ilc.get_iteration()}  "
                f"RMS Error: {ilc.get_rms_error():.5f}  "
                f"Mean Error: {ilc.get_mean_error():.5f}  "
                f"Induced Error: {int(ilc.get_error_level() * 100)}%"
            )
            viz.draw_text(20, 20, status)

        viz.flip()
        viz.tick(FPS)

    server.stop()
    pygame.quit()
    plot3d_mgr.stop()


if __name__ == "__main__":
    mp.freeze_support()
    main()
