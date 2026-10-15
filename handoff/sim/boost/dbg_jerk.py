import numpy as np, boostsim as B
M = B.M
T0 = 8.0
mb, cfgb = M.load_ref('t2_lead2_phantom_fix')
for al in (1.0, 1.6):
  for seed in range(1, 4):
    bm = B.load_boost('WT')
    r = B.run(mb, cfgb, bm, T0 + 10, B.launch(T0, al), gap0=4.5, mode='vision', seed=seed, noise_kw=dict(sd=0.3, sv=0.4, sa=0.3))
    post = np.where((r.t >= T0) & (r.v > 0.1))[0]
    d = np.diff(r.a_cmd[post[0]:]) / 0.05
    k = np.argmax(np.abs(d)) + post[0]
    print(al, seed, r.events, f"max jerk {d[np.argmax(np.abs(d))]:+.2f} at t={r.t[k]-T0:.2f}: a_cmd {r.a_cmd[k]:.2f}->{r.a_cmd[k+1]:.2f} boost {r.boost[k]:.2f}->{r.boost[k+1]:.2f} mpc {r.a_mpc[k]:.2f}->{r.a_mpc[k+1]:.2f} clip {r.clip_hi[k]:.2f}")
