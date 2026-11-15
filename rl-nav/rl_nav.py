"""
Navigation par apprentissage par renforcement (methode TD) : aller d'un point de depart
a un point d'arrivee en evitant des obstacles, a partir d'un lidar.

Uniquement numpy + matplotlib (pas de PyTorch). L'entrainement se fait dans un simulateur 2D
integre (robot omnidirectionnel, lidar 24 faisceaux, arene 6 x 6 m, obstacles ronds aleatoires).

    python rl_nav.py train --updates 1500            # entraine, ecrit policy.npz (quelques minutes)
    python rl_nav.py run --start -2.5 -2.5 --goal 2.5 2.5          # fenetre : le robot va du depart a l'arrivee
    python rl_nav.py run --start -2.5 -2.5 --goal 2.5 2.5 --obstacle 0 0 0.5 --obstacle 1 -1 0.4 --png essai.png --no-gui
    python rl_nav.py eval --episodes 300             # taux de reussite sur des scenes aleatoires
    python rl_nav.py compare --updates 800           # TD(0) / TD(lambda) / Monte Carlo, meme budget
    python rl_nav.py selftest                        # verifie les gradients (sans entrainement)

Methode : acteur-critique avec cible TD(lambda) (GAE).
    delta_t = r_t + gamma * V(s_{t+1}) - V(s_t)          <- l'erreur de difference temporelle
    A_t     = delta_t + gamma * lambda * A_{t+1}
  --lam 0   : TD(0) pur (un pas, biaise mais peu de variance)
  --lam 1   : equivalent Monte Carlo (retour complet, sans biais mais tres bruite)
  --lam 0.9 : compromis (defaut)

Conventions : le cap du robot est fixe (le robot est omnidirectionnel, il translate sans tourner),
donc repere robot = repere du monde. Action = vitesse (vx, vy) demandee, normalisee dans [-1, 1].
"""
import argparse
import math
import sys
import time

import numpy as np

# ------------------------------------------------------------------ simulateur


class NavEnv:
    """n robots simules en parallele (tout est vectorise avec numpy)."""

    def __init__(self, n, seed=0, arena=3.0, n_beams=24, max_range=3.0, dt=0.1, vmax=0.4, acc=1.0,
                 radius=0.15, goal_tol=0.2, max_steps=250, k_max=6):
        self.n, self.A, self.B = n, arena, n_beams
        self.max_range, self.dt, self.vmax, self.acc = max_range, dt, vmax, acc
        self.radius, self.goal_tol, self.max_steps, self.K = radius, goal_tol, max_steps, k_max
        self.rng = np.random.default_rng(seed)
        ang = 2 * np.pi * np.arange(n_beams) / n_beams
        self.dirs = np.stack([np.cos(ang), np.sin(ang)], axis=1)              # (B, 2)
        self.obs_dim = n_beams + 5
        self.pos = np.zeros((n, 2)); self.vel = np.zeros((n, 2)); self.goal = np.zeros((n, 2))
        self.centers = np.full((n, k_max, 2), 100.0); self.radii = np.zeros((n, k_max))
        self.t = np.zeros(n, dtype=int); self.prev_d = np.zeros(n)

    # -- scenes
    def _clear(self, p, cen, rad, margin):
        d = np.linalg.norm(p[:, None, :] - cen, axis=2)
        return np.all(d > rad + self.radius + margin, axis=1)

    def reset(self, idx, difficulty=1.0):
        """Nouvelle scene aleatoire pour les robots d'indices idx.
        Une bonne partie des obstacles est posee SUR le trajet depart -> arrivee : sinon le robot
        apprend a aller tout droit et ne rencontre presque jamais d'obstacle a eviter."""
        m, K, A = len(idx), self.K, self.A
        n_obs = self.rng.integers(0, int(round(difficulty * K)) + 1, size=m)
        active = np.arange(K)[None, :] < n_obs[:, None]
        s = np.zeros((m, 2)); g = np.zeros((m, 2)); cen = np.full((m, K, 2), 100.0); rad = np.zeros((m, K))
        todo = np.arange(m)
        for _ in range(200):
            k_ = len(todo)
            s[todo] = self.rng.uniform(-A + 0.4, A - 0.4, (k_, 2))
            g[todo] = self.rng.uniform(-A + 0.4, A - 0.4, (k_, 2))
            seg = g[todo] - s[todo]
            L = np.linalg.norm(seg, axis=1, keepdims=True) + 1e-9
            u = seg / L; nrm = np.stack([-u[:, 1], u[:, 0]], axis=1)
            frac = self.rng.uniform(0.15, 0.85, (k_, K))
            lat = self.rng.normal(0.0, 0.3, (k_, K))
            on_path = s[todo][:, None, :] + (frac * L)[..., None] * u[:, None, :] + lat[..., None] * nrm[:, None, :]
            anywhere = self.rng.uniform(-A + 0.5, A - 0.5, (k_, K, 2))
            c = np.where((self.rng.random((k_, K)) < 0.65)[..., None], on_path, anywhere)
            c = np.clip(c, -A + 0.4, A - 0.4)
            r = self.rng.uniform(0.2, 0.55, (k_, K))
            cen[todo] = np.where(active[todo][..., None], c, 100.0)
            rad[todo] = np.where(active[todo], r, 0.0)
            ok = (self._clear(s[todo], cen[todo], rad[todo], 0.3) & self._clear(g[todo], cen[todo], rad[todo], 0.3)
                  & (L[:, 0] > 1.5))
            todo = todo[~ok]
            if len(todo) == 0:
                break
        self.centers[idx], self.radii[idx] = cen, rad
        self.pos[idx], self.goal[idx], self.vel[idx] = s, g, 0.0
        self.t[idx] = 0
        self.prev_d[idx] = np.linalg.norm(s - g, axis=1)

    def set_scene(self, start, goal, circles):
        """Scene fixe (n=1) : circles = liste de (x, y, r)."""
        assert self.n == 1 and len(circles) <= self.K
        self.centers[:] = 100.0; self.radii[:] = 0.0
        for k, (x, y, r) in enumerate(circles):
            self.centers[0, k], self.radii[0, k] = (x, y), r
        self.pos[0], self.goal[0], self.vel[0] = start, goal, 0.0
        self.t[0] = 0
        self.prev_d[0] = np.linalg.norm(np.array(start) - np.array(goal))

    # -- perception
    def lidar(self):
        p = self.pos[:, None, None, :]
        oc = p - self.centers[:, :, None, :]                                  # (n,K,1,2)
        b = (oc * self.dirs[None, None]).sum(-1)                              # (n,K,B)
        cc = (oc ** 2).sum(-1) - self.radii[:, :, None] ** 2
        disc = b * b - cc
        t = -b - np.sqrt(np.maximum(disc, 0.0))
        t = np.where((disc >= 0) & (t > 0), t, np.inf).min(axis=1)            # (n,B)
        with np.errstate(divide="ignore", invalid="ignore"):
            ux, uy = self.dirs[None, :, 0], self.dirs[None, :, 1]
            px, py = self.pos[:, 0:1], self.pos[:, 1:2]
            tx = np.where(ux > 0, (self.A - px) / ux, np.where(ux < 0, (-self.A - px) / ux, np.inf))
            ty = np.where(uy > 0, (self.A - py) / uy, np.where(uy < 0, (-self.A - py) / uy, np.inf))
        return np.minimum(np.minimum(t, np.minimum(tx, ty)), self.max_range)

    def observe(self, scan=None):
        scan = self.lidar() if scan is None else scan
        gv = self.goal - self.pos
        d = np.linalg.norm(gv, axis=1, keepdims=True)
        return np.concatenate([scan / self.max_range, gv / 4.0, d / 4.0, self.vel / self.vmax], axis=1)

    def collided(self):
        d = np.linalg.norm(self.pos[:, None, :] - self.centers, axis=2)
        hit = np.any(d < self.radii + self.radius, axis=1)
        return hit | (np.abs(self.pos) > self.A - self.radius).any(axis=1)

    # -- dynamique
    def step(self, action):
        """Retourne (reward, terminal, truncated, reached, collided). Ne remet rien a zero."""
        a = np.clip(action, -1.0, 1.0)
        dv = np.clip(a * self.vmax - self.vel, -self.acc * self.dt, self.acc * self.dt)
        self.vel += dv
        self.pos += self.vel * self.dt
        self.t += 1
        d = np.linalg.norm(self.goal - self.pos, axis=1)
        scan = self.lidar()
        clear = scan.min(axis=1) - self.radius
        reached = d < self.goal_tol
        crash = self.collided()
        r = 2.0 * (self.prev_d - d) - 0.01 - 0.1 * np.maximum(0.0, 0.3 - clear) / 0.3
        r = np.where(reached, r + 10.0, r)
        r = np.where(crash, -10.0, r)
        self.prev_d = d
        terminal = reached | crash
        truncated = (self.t >= self.max_steps) & ~terminal
        return r, terminal, truncated, reached, crash


# ------------------------------------------------------------------ reseaux

class MLP:
    """Perceptron multicouche (tanh) avec retropropagation ecrite a la main."""

    def __init__(self, sizes, rng, last_scale=1.0):
        self.W, self.b = [], []
        for i, (m, n) in enumerate(zip(sizes[:-1], sizes[1:])):
            s = math.sqrt(2.0 / m) * (last_scale if i == len(sizes) - 2 else 1.0)
            self.W.append(rng.normal(0.0, s, (m, n)))
            self.b.append(np.zeros(n))

    def forward(self, x):
        self.acts = [x]
        for i, (W, b) in enumerate(zip(self.W, self.b)):
            x = x @ W + b
            if i < len(self.W) - 1:
                x = np.tanh(x)
            self.acts.append(x)
        return x

    def backward(self, dout):
        gW, gb = [None] * len(self.W), [None] * len(self.W)
        d = dout
        for i in reversed(range(len(self.W))):
            gW[i] = self.acts[i].T @ d
            gb[i] = d.sum(axis=0)
            if i > 0:
                d = (d @ self.W[i].T) * (1.0 - self.acts[i] ** 2)
        return [g for pair in zip(gW, gb) for g in pair]

    def params(self):
        return [p for pair in zip(self.W, self.b) for p in pair]


class Adam:
    def __init__(self, params, lr):
        self.p, self.lr, self.t = params, lr, 0
        self.m = [np.zeros_like(p) for p in params]; self.v = [np.zeros_like(p) for p in params]

    def step(self, grads, clip=0.5):
        norm = math.sqrt(sum(float((g ** 2).sum()) for g in grads))
        if norm > clip:
            grads = [g * clip / (norm + 1e-8) for g in grads]
        self.t += 1
        for i, (p, g) in enumerate(zip(self.p, grads)):
            self.m[i] = 0.9 * self.m[i] + 0.1 * g
            self.v[i] = 0.999 * self.v[i] + 0.001 * g * g
            mh, vh = self.m[i] / (1 - 0.9 ** self.t), self.v[i] / (1 - 0.999 ** self.t)
            p -= self.lr * mh / (np.sqrt(vh) + 1e-8)


class Agent:
    def __init__(self, obs_dim, seed=0, hidden=64):
        rng = np.random.default_rng(seed)
        self.pi = MLP([obs_dim, hidden, hidden, 2], rng, last_scale=0.01)
        self.vf = MLP([obs_dim, hidden, hidden, 1], rng)
        self.log_std = np.full(2, -0.7)
        self.opt_pi = Adam(self.pi.params() + [self.log_std], 5e-4)
        self.opt_vf = Adam(self.vf.params(), 1e-3)

    def act(self, obs, rng, deterministic=False):
        mu = self.pi.forward(obs)
        if deterministic:
            return mu
        return mu + np.exp(self.log_std) * rng.standard_normal(mu.shape)

    def value(self, obs):
        return self.vf.forward(obs)[:, 0]

    def losses_and_grads(self, obs, act, adv, ret, ent_coef=0.003):
        """Perte acteur-critique + gradients (separes pour l'acteur et le critique)."""
        N = len(obs)
        mu = self.pi.forward(obs)
        std = np.exp(self.log_std)
        z = (act - mu) / std
        d_mu = -(adv[:, None] * z / std) / N
        d_ls = -(adv[:, None] * (z ** 2 - 1.0)).sum(axis=0) / N - ent_coef
        g_pi = self.pi.backward(d_mu) + [d_ls]
        v = self.vf.forward(obs)[:, 0]
        g_vf = self.vf.backward(((v - ret) / N)[:, None])
        logp = (-0.5 * z ** 2 - self.log_std - 0.5 * math.log(2 * math.pi)).sum(axis=1)
        loss_pi = float(-(adv * logp).mean() - ent_coef * (self.log_std + 1.4189).sum())
        loss_vf = float(0.5 * ((v - ret) ** 2).mean())
        return loss_pi, loss_vf, g_pi, g_vf

    def save(self, path):
        d = {f"pi_W{i}": w for i, w in enumerate(self.pi.W)} | {f"pi_b{i}": b for i, b in enumerate(self.pi.b)}
        d |= {f"vf_W{i}": w for i, w in enumerate(self.vf.W)} | {f"vf_b{i}": b for i, b in enumerate(self.vf.b)}
        np.savez(path, log_std=self.log_std, **d)

    @staticmethod
    def load(path, obs_dim):
        z = np.load(path)
        a = Agent(obs_dim)
        for i in range(len(a.pi.W)):
            a.pi.W[i][...], a.pi.b[i][...] = z[f"pi_W{i}"], z[f"pi_b{i}"]
            a.vf.W[i][...], a.vf.b[i][...] = z[f"vf_W{i}"], z[f"vf_b{i}"]
        a.log_std[...] = z["log_std"]
        return a


# ------------------------------------------------------------------ entrainement

def train(updates=1500, n_envs=64, horizon=32, lam=0.9, gamma=0.99, seed=0, out="policy.npz", verbose=True):
    env = NavEnv(n_envs, seed=seed)
    agent = Agent(env.obs_dim, seed=seed)
    rng = np.random.default_rng(seed + 1)
    diff = 0.0
    env.reset(np.arange(n_envs), diff)
    obs = env.observe()
    ep_ret = np.zeros(n_envs)
    recent = []                         # (retour, reussi, collision)
    t0 = time.time()
    for it in range(updates):
        diff = min(1.0, it / (0.4 * updates))          # curriculum : de plus en plus d'obstacles
        O = np.zeros((horizon, n_envs, env.obs_dim)); A = np.zeros((horizon, n_envs, 2))
        R = np.zeros((horizon, n_envs)); V = np.zeros((horizon, n_envs)); NV = np.zeros((horizon, n_envs))
        DONE = np.zeros((horizon, n_envs), dtype=bool)
        for t in range(horizon):
            v = agent.value(obs)
            a = agent.act(obs, rng)
            r, term, trunc, reached, crash = env.step(a)
            ep_ret += r
            done = term | trunc
            nobs = env.observe()
            nv = np.zeros(n_envs)
            if trunc.any():                                   # fin de temps : on amorce avec V(s')
                nv[trunc] = agent.value(nobs[trunc])
            O[t], A[t], R[t], V[t], DONE[t], NV[t] = obs, a, r, v, done, nv
            for i in np.flatnonzero(done):
                recent.append((ep_ret[i], bool(reached[i]), bool(crash[i])))
                ep_ret[i] = 0.0
            if done.any():
                env.reset(np.flatnonzero(done), diff)
                nobs = env.observe()
            obs = nobs
        last_v = agent.value(obs)
        # cibles TD(lambda) / GAE, calculees a rebours
        adv = np.zeros((horizon, n_envs)); gae = np.zeros(n_envs)
        for t in reversed(range(horizon)):
            nxt = last_v if t == horizon - 1 else V[t + 1]
            nxt = np.where(DONE[t], NV[t], nxt)
            delta = R[t] + gamma * nxt - V[t]                 # erreur de difference temporelle
            gae = delta + gamma * lam * np.where(DONE[t], 0.0, 1.0) * gae
            adv[t] = gae
        ret = adv + V
        o, a_, ad, rt = O.reshape(-1, env.obs_dim), A.reshape(-1, 2), adv.ravel(), ret.ravel()
        ad = (ad - ad.mean()) / (ad.std() + 1e-8)
        _, _, g_pi, g_vf = agent.losses_and_grads(o, a_, ad, rt)
        agent.opt_pi.step(g_pi)
        agent.opt_vf.step(g_vf)
        if verbose and (it + 1) % 50 == 0 and recent:
            w = recent[-300:]
            print(f"maj {it + 1:5d}/{updates}  difficulte={diff:.2f}  retour={np.mean([x[0] for x in w]):7.2f}  "
                  f"reussite={np.mean([x[1] for x in w]) * 100:5.1f}%  collision={np.mean([x[2] for x in w]) * 100:5.1f}%  "
                  f"({time.time() - t0:.0f}s)", flush=True)
    agent.save(out)
    if verbose:
        print("politique sauvegardee :", out)
    return agent


def evaluate(agent, episodes=300, seed=123, difficulty=1.0):
    env = NavEnv(episodes, seed=seed)
    env.reset(np.arange(episodes), difficulty)
    ok = np.zeros(episodes, dtype=bool); crash = np.zeros(episodes, dtype=bool)
    alive = np.ones(episodes, dtype=bool); steps = np.zeros(episodes)
    d0 = env.prev_d.copy(); path = np.zeros(episodes)
    for _ in range(env.max_steps):
        obs = env.observe()
        before = env.pos.copy()
        _, term, trunc, reached, c = env.step(agent.act(obs, None, deterministic=True))
        path += np.where(alive, np.linalg.norm(env.pos - before, axis=1), 0.0)
        steps += alive
        ok |= alive & reached; crash |= alive & c
        alive &= ~(term | trunc)
        if not alive.any():
            break
    eff = np.where(ok, d0 / np.maximum(path, 1e-6), np.nan)
    return ok.mean(), crash.mean(), steps[ok].mean() * env.dt if ok.any() else float("nan"), np.nanmean(eff) if ok.any() else float("nan")


# ------------------------------------------------------------------ essai : depart -> arrivee

def parse_xy(s):
    x, y = s.split(",")
    return float(x), float(y)


def parse_circles(s):
    return [tuple(float(v) for v in item.split(",")) for item in s.split(";") if item.strip()]


def rollout(agent, start, goal, circles):
    env = NavEnv(1, k_max=max(6, len(circles)))
    env.set_scene(start, goal, circles)
    traj, scans = [env.pos[0].copy()], []
    result = "temps ecoule"
    for _ in range(env.max_steps):
        obs = env.observe(); scans.append(env.lidar()[0].copy())
        _, term, trunc, reached, crash = env.step(agent.act(obs, None, deterministic=True))
        traj.append(env.pos[0].copy())
        if reached[0]:
            result = "ARRIVE"; break
        if crash[0]:
            result = "COLLISION"; break
    scans.append(env.lidar()[0].copy())
    return env, np.array(traj), np.array(scans), result


def draw_scene(ax, env, start, goal):
    import matplotlib.patches as mp
    A = env.A
    ax.set_xlim(-A, A); ax.set_ylim(-A, A); ax.set_aspect("equal"); ax.grid(alpha=0.3)
    for c, r in zip(env.centers[0], env.radii[0]):
        if r > 0:
            ax.add_patch(mp.Circle(c, r, color="0.35"))
    ax.plot(*start, "go", ms=9, label="depart")
    ax.plot(*goal, "r*", ms=15, label="arrivee")
    ax.add_patch(mp.Circle(goal, env.goal_tol, fill=False, ec="r", ls="--"))


def run(args):
    import matplotlib
    if args.no_gui:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    import matplotlib.patches as mp
    start, goal = tuple(args.start), tuple(args.goal)
    circles = [tuple(c) for c in args.obstacle] if args.obstacle else [(0, 0, 0.5), (1.2, -1.2, 0.4), (-1.2, 1.2, 0.4)]
    agent = Agent.load(args.policy, NavEnv(1).obs_dim)
    env, traj, scans, result = rollout(agent, start, goal, circles)
    length = float(np.linalg.norm(np.diff(traj, axis=0), axis=1).sum())
    print(f"Resultat : {result} en {(len(traj) - 1) * env.dt:.1f} s, trajet {length:.2f} m "
          f"(ligne droite : {np.linalg.norm(np.array(goal) - np.array(start)):.2f} m)")
    fig, ax = plt.subplots(figsize=(7, 7))
    draw_scene(ax, env, start, goal)
    ax.plot(traj[:, 0], traj[:, 1], "b-", lw=1.5, label="trajectoire")
    ax.legend(loc="upper left"); ax.set_title(f"{result} - {length:.2f} m")
    if args.png:
        fig.savefig(args.png, dpi=110)
        print("image :", args.png)
    if not args.no_gui:
        robot = mp.Circle(traj[0], env.radius, color="tab:blue", alpha=0.7); ax.add_patch(robot)
        rays = [ax.plot([], [], "-", color="orange", lw=0.5)[0] for _ in range(env.B)]

        def update(i):
            robot.center = traj[i]
            for k, ln in enumerate(rays):
                d = scans[min(i, len(scans) - 1)][k]
                ln.set_data([traj[i][0], traj[i][0] + d * env.dirs[k, 0]], [traj[i][1], traj[i][1] + d * env.dirs[k, 1]])
            return rays

        ani = animation.FuncAnimation(fig, update, frames=len(traj), interval=50, repeat=False)
        plt.show()


# ------------------------------------------------------------------ tests de coherence

def selftest():
    rng = np.random.default_rng(0)
    ag = Agent(29, seed=1, hidden=8)
    obs = rng.normal(size=(20, 29)); act = rng.normal(size=(20, 2))
    adv = rng.normal(size=20); ret = rng.normal(size=20)
    lp, lv, gpi, gvf = ag.losses_and_grads(obs, act, adv, ret)
    worst = 0.0
    for params, grads, which in ((ag.pi.params() + [ag.log_std], gpi, "pi"), (ag.vf.params(), gvf, "vf")):
        for p, g in zip(params, grads):
            flat = p.reshape(-1)
            for j in rng.choice(flat.size, size=min(4, flat.size), replace=False):
                old = flat[j]
                flat[j] = old + 1e-5; l1 = ag.losses_and_grads(obs, act, adv, ret)[0 if which == "pi" else 1]
                flat[j] = old - 1e-5; l2 = ag.losses_and_grads(obs, act, adv, ret)[0 if which == "pi" else 1]
                flat[j] = old
                num = (l1 - l2) / 2e-5
                worst = max(worst, abs(num - g.reshape(-1)[j]) / max(1e-6, abs(num) + abs(g.reshape(-1)[j])))
    print(f"gradients : ecart relatif max = {worst:.2e}", "OK" if worst < 1e-4 else "ERREUR")
    # lidar : un robot au centre, mur a 3 m et un cercle a 1 m devant (+x)
    env = NavEnv(1); env.set_scene((0, 0), (2, 2), [(1.0, 0.0, 0.3)])
    s = env.lidar()[0]
    print(f"lidar : faisceau +x = {s[0]:.2f} m (attendu 0.70), faisceau -x = {s[12]:.2f} m (attendu 3.00)")
    ok = abs(s[0] - 0.7) < 1e-6 and abs(s[12] - 3.0) < 1e-6 and worst < 1e-4
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["train", "run", "eval", "compare", "selftest"])
    ap.add_argument("--updates", type=int, default=1500)
    ap.add_argument("--envs", type=int, default=64)
    ap.add_argument("--lam", type=float, default=0.9)
    ap.add_argument("--gamma", type=float, default=0.99)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--policy", default="policy.npz")
    ap.add_argument("--start", type=float, nargs=2, default=[-2.5, -2.5], metavar=("X", "Y"))
    ap.add_argument("--goal", type=float, nargs=2, default=[2.5, 2.5], metavar=("X", "Y"))
    ap.add_argument("--obstacle", type=float, nargs=3, action="append", metavar=("X", "Y", "R"),
                    help="obstacle rond (centre x y, rayon r) ; repeter l'option pour en mettre plusieurs")
    ap.add_argument("--episodes", type=int, default=300)
    ap.add_argument("--png", default="")
    ap.add_argument("--no-gui", action="store_true")
    a = ap.parse_args()

    if a.cmd == "selftest":
        sys.exit(0 if selftest() else 1)
    if a.cmd == "train":
        ag = train(a.updates, a.envs, lam=a.lam, gamma=a.gamma, seed=a.seed, out=a.policy)
        print("evaluation (scenes aleatoires, difficulte max) : reussite %.1f%%, collisions %.1f%%, temps moyen %.1f s, "
              "efficacite du trajet %.2f" % tuple(np.array(evaluate(ag)) * [100, 100, 1, 1]))
    elif a.cmd == "eval":
        ag = Agent.load(a.policy, NavEnv(1).obs_dim)
        r = evaluate(ag, a.episodes)
        print("reussite %.1f%%, collisions %.1f%%, temps moyen %.1f s, efficacite du trajet %.2f (1 = ligne droite)"
              % (r[0] * 100, r[1] * 100, r[2], r[3]))
    elif a.cmd == "run":
        run(a)
    elif a.cmd == "compare":
        print("lambda | reussite | collisions | efficacite")
        for lam in (0.0, 0.5, 0.9, 1.0):
            ag = train(a.updates, a.envs, lam=lam, gamma=a.gamma, seed=a.seed, out=f"policy_lam{lam}.npz", verbose=False)
            r = evaluate(ag)
            print(f"  {lam:.1f}  |  {r[0] * 100:5.1f}%  |  {r[1] * 100:5.1f}%  |  {r[3]:.2f}", flush=True)


if __name__ == "__main__":
    main()
