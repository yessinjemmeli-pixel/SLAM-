"""Tests de la grille d'occupation (module pur numpy, sans ROS)."""
import os
import tempfile

import numpy as np

from robot.occupancy_grid import OccupancyGridMap, grid_to_gray, write_png

MIN_R, MAX_R = 0.05, 6.0


def room_scan(x, y, theta, x0=-2.0, x1=3.0, y0=-1.5, y1=2.5, n=720):
    """Scan synthetique d'une piece rectangulaire (murs x0,x1,y0,y1) vu depuis (x, y, theta)."""
    angles = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    wa = angles + theta
    c, s = np.cos(wa), np.sin(wa)
    with np.errstate(divide="ignore", invalid="ignore"):
        tx = np.where(c > 0, (x1 - x) / c, np.where(c < 0, (x0 - x) / c, np.inf))
        ty = np.where(s > 0, (y1 - y) / s, np.where(s < 0, (y0 - y) / s, np.inf))
    return angles, np.minimum(tx, ty)


def build_room(g=None, **kw):
    g = g or OccupancyGridMap(resolution=0.05, size_m=20.0)
    for _ in range(3):
        a, d = room_scan(0.0, 0.0, 0.0, **kw)
        g.update(0.0, 0.0, 0.0, a, d, MIN_R, MAX_R)
    return g


def test_walls_occupied_interior_free_outside_unknown():
    g = build_room()
    assert g.probability_at(3.0, 0.0) > 0.6          # mur x1
    assert g.probability_at(-2.0, 0.5) > 0.6         # mur x0
    assert g.probability_at(0.0, 2.5) > 0.6          # mur y1
    assert g.probability_at(1.0, 1.0) < 0.3          # interieur libre
    assert g.probability_at(0.0, 0.0) < 0.3
    assert g.probability_at(6.0, 6.0) is None        # derriere les murs : inconnu
    assert g.n_known_cells > 0


def test_angle_conventions():
    def hit(theta, angle):
        g = OccupancyGridMap(resolution=0.05, size_m=10.0)
        g.update(0.0, 0.0, theta, [angle], [2.0], MIN_R, MAX_R)
        return g

    g = hit(0.0, 0.0)                    # devant, robot oriente +x
    assert g.probability_at(2.0, 0.0) > 0.6
    g = hit(np.pi / 2, 0.0)              # robot oriente +y
    assert g.probability_at(0.0, 2.0) > 0.6
    g = hit(0.0, np.pi / 2)              # faisceau a 90 deg, robot +x
    assert g.probability_at(0.0, 2.0) > 0.6


def test_dynamic_obstacle_gets_cleared():
    g = OccupancyGridMap(resolution=0.05, size_m=10.0)
    for _ in range(3):
        g.update(0.0, 0.0, 0.0, [0.0], [1.0], MIN_R, MAX_R)
    assert g.probability_at(1.0, 0.0) > 0.8
    for _ in range(12):                  # l'obstacle a disparu : le mur est a 3 m
        g.update(0.0, 0.0, 0.0, [0.0], [3.0], MIN_R, MAX_R)
    assert g.probability_at(1.0, 0.0) < 0.3


def test_range_gating_and_invalid_values():
    g = OccupancyGridMap(resolution=0.05, size_m=20.0)
    ang = np.array([0.0, 0.5, 1.0, 1.5, 2.0])
    dist = np.array([np.inf, np.nan, 0.01, 7.0, 6.0])   # tous invalides
    assert g.update(0, 0, 0, ang, dist, MIN_R, MAX_R) == (0, 0)
    assert g.n_known_cells == 0


def test_export_shape_and_origin():
    g = build_room()
    assert OccupancyGridMap().export_ros() is None
    data, w, h, ox, oy = g.export_ros(margin_cells=5)
    assert data.dtype == np.int8 and data.size == w * h
    assert -1 in data and data.max() <= 100
    # le mur x = 3 m doit tomber sur la bonne colonne du message
    d2 = data.reshape(h, w)
    ix = int((3.0 - ox) / g.resolution)
    iy = int((0.0 - oy) / g.resolution)
    assert d2[iy, ix] > 60
    assert ox <= -2.0 and oy <= -1.5


def test_reset():
    g = build_room()
    g.reset()
    assert g.n_known_cells == 0 and g.export_ros() is None


def test_out_of_bounds_does_not_crash():
    g = OccupancyGridMap(resolution=0.05, size_m=4.0)       # +/-2 m
    ang = np.linspace(0, 2 * np.pi, 360, endpoint=False)
    g.update(1.9, 1.9, 0.0, ang, np.full(360, 5.0), MIN_R, MAX_R)
    g.update(50.0, 50.0, 0.0, ang, np.full(360, 5.0), MIN_R, MAX_R)


def test_png_and_save():
    with tempfile.TemporaryDirectory() as tmp:
        g = build_room()
        data, w, h, _, _ = g.export_ros(margin_cells=5)
        gray = grid_to_gray(data.reshape(h, w))
        assert gray.shape == (h, w)
        p = os.path.join(tmp, "a.png")
        write_png(p, gray, scale=2)
        with open(p, "rb") as f:
            raw = f.read()
        assert raw[:8] == b"\x89PNG\r\n\x1a\n" and b"IEND" in raw
        res = g.save(os.path.join(tmp, "maps"))
        assert res and all(os.path.exists(r) for r in res)
        assert OccupancyGridMap().save(tmp) is None
