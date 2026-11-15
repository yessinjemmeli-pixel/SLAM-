"""
Cartographie en direct sur PC : RPLidar C1 branche en USB -> grille d'occupation,
sans Raspberry Pi, sans ROS2, sans Open3D.

Installation (une seule fois) :
    pip install rplidar-roboticia pyserial matplotlib numpy

Utilisation :
    python live_grid.py --port COM5        # lidar reel (tenu a la main ou pose sur le robot)
    python live_grid.py --sim              # test sans lidar : piece simulee
    python live_grid.py --selftest         # test automatique sans fenetre (erreur de pose)

Dans la fenetre :  touche S = sauvegarder la carte (PNG + NPZ)   touche R = repartir de zero
Les cartes sont ecrites dans le dossier ./maps a cote de ce script.

Comment ca marche : sans encodeurs, la position est estimee uniquement avec le lidar.
A chaque tour, on teste de petits deplacements/rotations autour de la derniere position
et on garde celui qui colle le mieux le scan sur la carte deja construite (scan matching
par correlation), puis on ajoute le scan a la carte. Deplacez le lidar LENTEMENT
(marche lente, rotations douces) dans un endroit avec des murs / des formes variees :
un couloir droit ou une grande salle vide font deriver l'estimation.
"""
import argparse
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
# On charge occupancy_grid.py directement par son chemin. Surtout PAS via sys.path : le dossier
# robot/robot contient un serial.py qui masquerait la librairie pyserial (et exige rclpy).
import importlib.util  # noqa: E402
_spec = importlib.util.spec_from_file_location(
    "occupancy_grid",
    os.path.join(HERE, "..", "..", "Navigation-main", "robot_ws", "src", "robot", "robot", "occupancy_grid.py"))
_og = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_og)
OccupancyGridMap, grid_to_gray = _og.OccupancyGridMap, _og.grid_to_gray

MIN_RANGE, MAX_RANGE = 0.05, 5.9
BAUDRATE = 460800


# ------------------------------------------------------------------ estimation de pose + carte

class Mapper:
    """Scan matching (correlation sur la grille) + mise a jour de la grille."""

    def __init__(self, max_omega=0.6, accept_score=0.3):
        self.grid = OccupancyGridMap(resolution=0.05, size_m=40.0)
        self.max_omega = max_omega
        self.accept_score = accept_score
        self.reset()

    def reset(self):
        self.grid.reset()
        self.x = self.y = self.theta = 0.0
        self.trail = [(0.0, 0.0)]
        self.lost = 0
        self.skipped = 0
        self.last_score = 0.0
        self._t_prev = None
        self._vel = (0.0, 0.0, 0.0)   # dernier deplacement entre deux scans (prediction)

    # -- scoring
    def _smoothed_occ(self):
        """Carte d'occupation lissee (flou gaussien 3x3 binomial) : donne un pic lisse autour des murs,
        que l'on echantillonne en bilineaire -> precision bien meilleure que la taille d'une cellule."""
        occ = np.maximum(self.grid.logodds, 0.0)
        k = (0.25, 0.5, 0.25)
        t = k[1] * occ
        t[1:, :] += k[0] * occ[:-1, :]
        t[:-1, :] += k[2] * occ[1:, :]
        u = k[1] * t
        u[:, 1:] += k[0] * t[:, :-1]
        u[:, :-1] += k[2] * t[:, 1:]
        return u

    def _score(self, occ, px, py, cx, cy, cth):
        """Score moyen (interpolation bilineaire) de chaque pose candidate."""
        g = self.grid
        c, s = np.cos(cth)[:, None], np.sin(cth)[:, None]
        X = cx[:, None] + px[None, :] * c - py[None, :] * s
        Y = cy[:, None] + px[None, :] * s + py[None, :] * c
        fx = (X - g.origin_x) / g.resolution - 0.5          # centre de cellule en (i + 0.5)
        fy = (Y - g.origin_y) / g.resolution - 0.5
        ix, iy = np.floor(fx).astype(np.int64), np.floor(fy).astype(np.int64)
        wx, wy = fx - ix, fy - iy
        ok = (ix >= 0) & (ix < g.cells - 1) & (iy >= 0) & (iy < g.cells - 1)
        ix, iy = np.clip(ix, 0, g.cells - 2), np.clip(iy, 0, g.cells - 2)
        v = (occ[iy, ix] * (1 - wx) * (1 - wy) + occ[iy, ix + 1] * wx * (1 - wy)
             + occ[iy + 1, ix] * (1 - wx) * wy + occ[iy + 1, ix + 1] * wx * wy)
        return np.where(ok, v, 0.0).mean(axis=1)

    def _search(self, occ, px, py, cx0, cy0, cth0, half_xy, step_xy, half_th, step_th):
        dxy = np.arange(-half_xy, half_xy + 1e-9, step_xy)
        dth = np.arange(-half_th, half_th + 1e-9, step_th)
        DX, DY, DT = np.meshgrid(dxy, dxy, dth, indexing="ij")
        DX, DY, DT = DX.ravel(), DY.ravel(), DT.ravel()
        sc = self._score(occ, px, py, cx0 + DX, cy0 + DY, cth0 + DT)
        # petite preference pour "ne pas bouger" en cas d'egalite
        sc = sc - 0.003 * (np.hypot(DX, DY) / 0.1 + np.abs(DT) / 0.1)
        k = int(np.argmax(sc))
        return cx0 + DX[k], cy0 + DY[k], cth0 + DT[k], float(sc[k])

    # -- un scan : angles en degres (sens horaire, convention RPLidar), distances en mm
    def process(self, angles_deg, dists_mm, now=None):
        now = time.monotonic() if now is None else now
        a = -np.radians(np.asarray(angles_deg, dtype=np.float64))   # -> antihoraire (x devant, y a gauche)
        d = np.asarray(dists_mm, dtype=np.float64) / 1000.0
        ok = np.isfinite(d) & (d > MIN_RANGE) & (d < MAX_RANGE)
        a, d = a[ok], d[ok]
        if a.size < 30:
            return
        status_new = (self.x, self.y, self.theta)

        if self.grid.n_updates > 0:
            px, py = d * np.cos(a), d * np.sin(a)
            occ = self._smoothed_occ()
            vx, vy, vth = self._vel
            x, y, th, _ = self._search(occ, px, py, self.x + vx, self.y + vy, self.theta + vth,
                                       0.12, 0.03, math.radians(6), math.radians(1.5))
            x, y, th, sc = self._search(occ, px, py, x, y, th, 0.03, 0.01, math.radians(1.5), math.radians(0.5))
            self.last_score = sc
            if sc >= self.accept_score:
                status_new = (x, y, th)
                self.lost = 0
            else:
                self.lost += 1
                return                      # scan non fiable : on ne pollue pas la carte

        x, y, th = status_new
        omega = 0.0
        if self._t_prev is not None and now - self._t_prev > 1e-3:
            omega = abs((th - self.theta + math.pi) % (2 * math.pi) - math.pi) / (now - self._t_prev)
        self._t_prev = now
        self._vel = (x - self.x, y - self.y, th - self.theta) if self.grid.n_updates > 1 else (0.0, 0.0, 0.0)
        self.x, self.y, self.theta = x, y, th
        self.trail.append((x, y))
        if self.max_omega > 0 and omega > self.max_omega:
            self.skipped += 1               # scan deforme par la rotation
            return
        self.grid.update(x, y, th, a, d, MIN_RANGE, MAX_RANGE)


# ------------------------------------------------------------------ sources de scans

class SimLidar:
    """Piece simulee 6 x 4 m avec une boite et un pilier ; le 'robot' fait un tour de la piece."""
    SEGS = [((-3, -2), (3, -2)), ((3, -2), (3, 2)), ((3, 2), (-3, 2)), ((-3, 2), (-3, -2)),            # murs
            ((0.5, -0.8), (1.5, -0.8)), ((1.5, -0.8), (1.5, 0.2)), ((1.5, 0.2), (0.5, 0.2)), ((0.5, 0.2), (0.5, -0.8)),  # boite
            ((-1.6, 0.9), (-1.2, 0.9)), ((-1.2, 0.9), (-1.2, 1.3)), ((-1.2, 1.3), (-1.6, 1.3)), ((-1.6, 1.3), (-1.6, 0.9))]

    def __init__(self, n_beams=500, noise=0.01, seed=1):
        self.t = 0
        self.n = n_beams
        self.noise = noise
        self.rng = np.random.default_rng(seed)
        self.truth = (-2.0, -1.2, 0.0)

    @staticmethod
    def _make_path():
        """Avance de 2.5 cm par scan puis rotation sur place de 3 deg par scan a chaque coin."""
        per = [(-2.0, -1.2), (2.2, -1.2), (2.2, 1.3), (-2.0, 1.3), (-2.0, -1.2)]
        poses = []
        th = math.atan2(per[1][1] - per[0][1], per[1][0] - per[0][0])
        for (x0, y0), (x1, y1) in zip(per[:-1], per[1:]):
            target = math.atan2(y1 - y0, x1 - x0)
            while abs((target - th + math.pi) % (2 * math.pi) - math.pi) > 1e-6:      # rotation
                dth = (target - th + math.pi) % (2 * math.pi) - math.pi
                th += max(-math.radians(3), min(math.radians(3), dth))
                poses.append((x0, y0, th))
            L = math.hypot(x1 - x0, y1 - y0)
            for k in range(1, int(L / 0.025) + 1):                                    # translation
                poses.append((x0 + k * 0.025 * math.cos(target), y0 + k * 0.025 * math.sin(target), target))
        return poses

    def true_pose(self, t):
        if not hasattr(self, "_path"):
            self._path = self._make_path()
        return self._path[min(t, len(self._path) - 1)]

    def poll(self):
        x, y, th = self.true_pose(self.t)
        self.truth = (x, y, th)
        self.t += 1
        adeg = np.linspace(0, 360, self.n, endpoint=False)
        wa = th - np.radians(adeg)                     # angles horaires -> monde
        dirs = np.stack([np.cos(wa), np.sin(wa)], axis=1)
        best = np.full(self.n, np.inf)
        for (x0, y0), (x1, y1) in self.SEGS:
            ex, ey = x1 - x0, y1 - y0
            den = dirs[:, 0] * ey - dirs[:, 1] * ex
            with np.errstate(divide="ignore", invalid="ignore"):
                t = ((x0 - x) * ey - (y0 - y) * ex) / den
                u = ((x0 - x) * dirs[:, 1] - (y0 - y) * dirs[:, 0]) / den
            hit = (t > 0) & (u >= 0) & (u <= 1)
            best = np.where(hit & (t < best), t, best)
        best = best + self.rng.normal(0, self.noise, self.n)
        best[~np.isfinite(best)] = 0
        return [(adeg, best * 1000.0)]

    def close(self):
        pass


class RealLidar:
    """RPLidar C1 : lecture par gros blocs (voir visualize_lidar.py pour les explications)."""

    def __init__(self, port):
        from rplidar import RPLidar, RPLidarException, _process_scan
        self._exc, self._proc = RPLidarException, _process_scan
        self.lidar = RPLidar(port, baudrate=BAUDRATE, timeout=3)
        print("Info  :", self.lidar.get_info())
        print("Sante :", self.lidar.get_health())
        self.lidar._serial.reset_input_buffer()
        self.lidar.start("normal")
        self.buf = bytearray()
        self.cur = []

    def poll(self):
        scans = []
        n = self.lidar._serial.in_waiting
        if n >= 5:
            self.buf.extend(self.lidar._serial.read(n))
        nb = len(self.buf) // 5
        for i in range(nb):
            try:
                new_scan, _q, angle, dist = self._proc(bytes(self.buf[i * 5:(i + 1) * 5]))
            except self._exc:
                continue
            if new_scan:
                if len(self.cur) > 30:
                    arr = np.array(self.cur)
                    scans.append((arr[:, 0], arr[:, 1]))
                self.cur = []
            if dist > 0:
                self.cur.append((angle, dist))
        self.buf = self.buf[nb * 5:]
        return scans

    def close(self):
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()
        print("Lidar arrete proprement.")


# ------------------------------------------------------------------ programme

def selftest():
    sim = SimLidar()
    m = Mapper()
    # le repere de la carte est celui de la 1re pose : on compare a la verite exprimee dans ce repere
    x0, y0, th0 = sim.true_pose(0)
    errs, now = [], 0.0
    for _ in range(len(sim._make_path()) - 1):
        for adeg, dmm in sim.poll():
            now += 0.1
            m.process(adeg, dmm, now=now)
        tx, ty, tth = sim.truth
        c, s = math.cos(-th0), math.sin(-th0)
        rx, ry = c * (tx - x0) - s * (ty - y0), s * (tx - x0) + c * (ty - y0)
        errs.append(math.hypot(m.x - rx, m.y - ry))
    print(f"scans: {sim.t}, erreur de position : moyenne {np.mean(errs)*100:.1f} cm, "
          f"max {np.max(errs)*100:.1f} cm, finale {errs[-1]*100:.1f} cm")
    print(f"scans perdus: {m.lost}, saute (rotation): {m.skipped}, cellules connues: {m.grid.n_known_cells}")
    res = m.grid.save(os.path.join(HERE, "maps"), prefix="selftest")
    print("carte ecrite :", res[0] if res else None)
    return np.max(errs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        import matplotlib
        sys.exit(0 if selftest() < 0.15 else 1)

    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    src = SimLidar() if args.sim else RealLidar(args.port)
    mapper = Mapper()
    save_dir = os.path.join(HERE, "maps")

    fig, ax = plt.subplots(figsize=(8, 8))
    img = ax.imshow(np.full((2, 2), 205, dtype=np.uint8), cmap="gray", vmin=0, vmax=255,
                    origin="lower", extent=[-1, 1, -1, 1], interpolation="nearest")
    trail, = ax.plot([], [], "b-", lw=1)
    robot, = ax.plot([], [], "r^", ms=10)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    title = ax.set_title("En attente du premier tour...")
    state = {"last_draw": 0.0, "msg": ""}

    def on_key(ev):
        if ev.key == "s":
            res = mapper.grid.save(save_dir)
            state["msg"] = "sauvegarde : " + os.path.basename(res[0]) if res else "carte vide"
            print(state["msg"], "->", save_dir)
        elif ev.key == "r":
            mapper.reset()
            state["msg"] = "carte remise a zero"

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_f):
        for adeg, dmm in src.poll():
            mapper.process(adeg, dmm)
        now = time.monotonic()
        if now - state["last_draw"] < 0.3:
            return
        state["last_draw"] = now
        exp = mapper.grid.export_ros(margin_cells=20)
        if exp is not None:
            data, w, h, ox, oy = exp
            res = mapper.grid.resolution
            img.set_data(np.flipud(grid_to_gray(data.reshape(h, w))))
            img.set_extent([ox, ox + w * res, oy, oy + h * res])
            ax.set_xlim(ox, ox + w * res)
            ax.set_ylim(oy, oy + h * res)
        tr = np.array(mapper.trail[-3000:])
        trail.set_data(tr[:, 0], tr[:, 1])
        robot.set_data([mapper.x], [mapper.y])
        flag = "  [POSITION PERDUE : ralentis / reviens en arriere]" if mapper.lost > 3 else ""
        title.set_text(f"x={mapper.x:.2f} y={mapper.y:.2f} th={math.degrees(mapper.theta):.0f} deg | "
                       f"score={mapper.last_score:.2f} | S=sauver R=reset{flag} {state['msg']}")

    ani = animation.FuncAnimation(fig, update, interval=30, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        src.close()


if __name__ == "__main__":
    main()
