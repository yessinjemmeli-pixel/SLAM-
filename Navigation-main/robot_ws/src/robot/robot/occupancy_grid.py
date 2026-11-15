"""Grille d'occupation 2D (log-odds) construite a partir de scans lidar.

Module pur numpy : il n'importe ni ROS ni Open3D, donc il se teste sur un PC
sans rien installer d'autre (voir test/test_occupancy_grid.py).

Principe : pour chaque scan et la pose (x, y, theta) du robot au moment du
scan, chaque faisceau lidar fait deux choses :
  - les cellules traversees par le rayon, entre le robot et le point touche,
    deviennent "plus libres" (leur log-odds baisse de l_free) ;
  - la cellule du point touche devient "plus occupee" (log-odds + l_occ).
Les cellules qu'aucun rayon n'a jamais traversees restent "inconnues".
Les scores sont bornes ([l_min, l_max]) pour qu'une cellule puisse changer
d'avis si l'environnement bouge (obstacle deplace, porte ouverte...).

Conventions (identiques a LocalizationNode._scan_to_global_pcd) :
  - un faisceau d'angle a (rad) et de distance d touche le point monde
    (x + d*cos(theta + a), y + d*sin(theta + a)) ;
  - le tableau est indexe [iy, ix] : x croit avec les colonnes, y avec les
    lignes ; la cellule (0, 0) a son coin bas-gauche en (origin_x, origin_y) ;
  - le robot demarre au centre de la grille, qui est donc la zone cartographiable
    de +/- size_m / 2 autour du point de depart.
"""
import os
import struct
import time
import zlib

import numpy as np

UNKNOWN = -1  # valeur "inconnu" dans la convention nav_msgs/OccupancyGrid


class OccupancyGridMap:
    """Grille d'occupation 2D en log-odds, centree sur la pose de depart."""

    def __init__(self, resolution=0.05, size_m=40.0, l_occ=0.85, l_free=-0.40,
                 l_min=-4.0, l_max=4.0):
        self.resolution = float(resolution)
        self.cells = int(round(size_m / self.resolution))
        self.origin_x = -0.5 * self.cells * self.resolution
        self.origin_y = -0.5 * self.cells * self.resolution
        self.l_occ = float(l_occ)
        self.l_free = float(l_free)
        self.l_min = float(l_min)
        self.l_max = float(l_max)
        self.logodds = np.zeros((self.cells, self.cells), dtype=np.float32)
        self.observed = np.zeros((self.cells, self.cells), dtype=bool)
        self.n_updates = 0

    # ------------------------------------------------------------------ etat

    def reset(self):
        """Efface toute la carte (nouvelle session ou resetpos)."""
        self.logodds[:] = 0.0
        self.observed[:] = False
        self.n_updates = 0

    @property
    def n_known_cells(self):
        return int(self.observed.sum())

    def world_to_cell(self, x, y):
        """Indices (ix, iy) de la cellule contenant le point monde (x, y)."""
        ix = int(np.floor((x - self.origin_x) / self.resolution))
        iy = int(np.floor((y - self.origin_y) / self.resolution))
        return ix, iy

    def probability_at(self, x, y):
        """Probabilite d'occupation (0..1) au point monde, None si inconnu."""
        ix, iy = self.world_to_cell(x, y)
        if not (0 <= ix < self.cells and 0 <= iy < self.cells):
            return None
        if not self.observed[iy, ix]:
            return None
        return float(1.0 / (1.0 + np.exp(-float(self.logodds[iy, ix]))))

    # ------------------------------------------------------------ mise a jour

    def update(self, x, y, theta, angles, distances, min_range, max_range):
        """Integre un scan lidar pris depuis la pose (x, y, theta).

        angles / distances : tableaux de meme taille (rad, m), dans le repere
        du robot. Les mesures non finies ou hors de ]min_range, max_range[ sont
        ignorees : un faisceau sans retour ne dit rien sur l'espace libre (le
        C1 ne renvoie rien sur une surface noire au-dela de ~6 m), on ne le
        marque donc ni libre ni occupe.

        Retourne (nb_cellules_libres_maj, nb_cellules_occupees_maj).
        """
        angles = np.asarray(angles, dtype=np.float64).ravel()
        dists = np.asarray(distances, dtype=np.float64).ravel()
        n = min(angles.size, dists.size)
        angles, dists = angles[:n], dists[:n]

        valid = (np.isfinite(angles) & np.isfinite(dists)
                 & (dists > min_range) & (dists < max_range))
        if not valid.any():
            return 0, 0
        a = angles[valid] + theta
        d = dists[valid]
        cos_a, sin_a = np.cos(a), np.sin(a)

        res = self.resolution
        w = self.cells

        # -- cellules touchees (occupees)
        hit_ix = np.floor((x + d * cos_a - self.origin_x) / res).astype(np.int64)
        hit_iy = np.floor((y + d * sin_a - self.origin_y) / res).astype(np.int64)
        hit_ok = (hit_ix >= 0) & (hit_ix < w) & (hit_iy >= 0) & (hit_iy < w)
        occ_flat = np.unique(hit_iy[hit_ok] * w + hit_ix[hit_ok])

        # -- cellules traversees (libres) : on echantillonne chaque rayon tous
        # les res/2, ce qui garantit de ne sauter aucune cellule traversee
        step = 0.5 * res
        k_max = int(d.max() / step) + 1
        t = np.arange(k_max, dtype=np.float64) * step           # (K,)
        along = t[None, :] < d[:, None]                           # (N, K)
        free_ix = np.floor((x + cos_a[:, None] * t[None, :] - self.origin_x) / res).astype(np.int64)
        free_iy = np.floor((y + sin_a[:, None] * t[None, :] - self.origin_y) / res).astype(np.int64)
        free_ok = along & (free_ix >= 0) & (free_ix < w) & (free_iy >= 0) & (free_iy < w)
        free_flat = np.unique(free_iy[free_ok] * w + free_ix[free_ok])
        # un mur touche par un faisceau ne doit pas etre efface par un autre
        # faisceau rasant du meme scan
        free_flat = np.setdiff1d(free_flat, occ_flat, assume_unique=True)

        lo = self.logodds.reshape(-1)
        obs = self.observed.reshape(-1)
        if free_flat.size:
            lo[free_flat] = np.clip(lo[free_flat] + self.l_free, self.l_min, self.l_max)
            obs[free_flat] = True
        if occ_flat.size:
            lo[occ_flat] = np.clip(lo[occ_flat] + self.l_occ, self.l_min, self.l_max)
            obs[occ_flat] = True

        self.n_updates += 1
        return int(free_flat.size), int(occ_flat.size)

    # ---------------------------------------------------------------- export

    def export_ros(self, margin_cells=20):
        """Exporte la zone exploree au format nav_msgs/OccupancyGrid.

        On ne publie que le rectangle englobant les cellules observees (plus
        une marge), pas toute la grille : le message reste petit.

        Retourne None si rien n'a ete observe, sinon
        (data, width, height, origin_x, origin_y) ou data est un tableau
        numpy int8 a 1 dimension de taille width*height, ligne par ligne
        (index = iy * width + ix), avec -1 = inconnu et 0..100 = probabilite
        d'occupation en %. (origin_x, origin_y) est la position monde du coin
        bas-gauche de la cellule (0, 0) du message.
        """
        if not self.observed.any():
            return None
        rows = np.flatnonzero(self.observed.any(axis=1))
        cols = np.flatnonzero(self.observed.any(axis=0))
        r0 = max(int(rows[0]) - margin_cells, 0)
        r1 = min(int(rows[-1]) + margin_cells + 1, self.cells)
        c0 = max(int(cols[0]) - margin_cells, 0)
        c1 = min(int(cols[-1]) + margin_cells + 1, self.cells)

        lo = self.logodds[r0:r1, c0:c1].astype(np.float64)
        obs = self.observed[r0:r1, c0:c1]
        prob = np.rint(100.0 / (1.0 + np.exp(-lo))).astype(np.int8)
        data = np.where(obs, prob, np.int8(UNKNOWN)).astype(np.int8)

        origin_x = self.origin_x + c0 * self.resolution
        origin_y = self.origin_y + r0 * self.resolution
        return data.ravel(), c1 - c0, r1 - r0, origin_x, origin_y

    def save(self, directory, prefix="map", scale=4):
        """Sauvegarde la carte : une image PNG (a regarder) et un .npz (a recharger).

        Retourne (chemin_png, chemin_npz), ou None si la carte est vide.
        """
        exported = self.export_ros(margin_cells=10)
        if exported is None:
            return None
        data, w, h, _ox, _oy = exported
        os.makedirs(directory, exist_ok=True)
        base = os.path.join(directory, "%s_%s" % (prefix, time.strftime("%Y%m%d_%H%M%S")))
        np.savez_compressed(
            base + ".npz",
            logodds=self.logodds,
            observed=self.observed,
            resolution=np.float64(self.resolution),
            origin=np.array([self.origin_x, self.origin_y]),
        )
        write_png(base + ".png", grid_to_gray(data.reshape(h, w)), scale=scale)
        return base + ".png", base + ".npz"


# ---------------------------------------------------------------- images


def grid_to_gray(data2d):
    """Convertit une grille int8 (-1 inconnu, 0..100) en image niveaux de gris.

    Memes teintes que map_server : inconnu = gris (205), libre = blanc (255),
    occupe = noir (0). La ligne du haut de l'image est le y le plus grand,
    comme sur une carte (l'axe y du tableau va vers le haut, pas celui d'une image).
    """
    d = np.asarray(data2d, dtype=np.int16)
    gray = np.where(d < 0, 205, 255 - (d * 255) // 100).astype(np.uint8)
    return np.flipud(gray)


def write_png(path, gray, scale=1):
    """Ecrit une image PNG 8 bits en niveaux de gris, sans dependance externe."""
    gray = np.asarray(gray, dtype=np.uint8)
    if scale > 1:
        gray = np.repeat(np.repeat(gray, scale, axis=0), scale, axis=1)
    h, w = gray.shape
    raw = np.hstack([np.zeros((h, 1), dtype=np.uint8), gray]).tobytes()  # filtre 0 / ligne

    def chunk(tag, payload):
        body = tag + payload
        return struct.pack(">I", len(payload)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 6))
           + chunk(b"IEND", b""))
    with open(path, "wb") as f:
        f.write(png)
