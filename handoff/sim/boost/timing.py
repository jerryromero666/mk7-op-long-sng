import time, numpy as np, boostsim as B
M = B.M
mb, cfgb = M.load_ref('t2_lead2_phantom_fix')
bm = B.load_boost('WT')
t = time.time()
r = B.run(mb, cfgb, bm, 60.0, B.stationary(), gap0=4.5, mode='vision', seed=1, noise_kw=dict(sv=0.5))
print('60s run', time.time() - t, r.events, r.stop_to_pid)
lb = None
