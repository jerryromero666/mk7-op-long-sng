import sys, numpy as np, boostsim as B
M = B.M
T0 = 8.0
al = float(sys.argv[1]); gap0 = float(sys.argv[2]); OV = eval(sys.argv[3]) if len(sys.argv) > 3 else {}
mode = sys.argv[4] if len(sys.argv) > 4 else 'vision'
mb, cfgb = M.load_ref('t2_lead2_phantom_fix')
bm = B.load_boost('WT', OV)
log = []
def probe(t, pl, plant, rs):
  lb = pl.lb
  if T0 - 0.1 <= t <= T0 + 5:
    st = K = None
    if lb.d_stop is not None:
      st = bm.K_SPEED * (lb.v_f - plant.v - bm.V_MARGIN); K = bm.K_GAP * (lb.d_f - lb.d_stop - bm.D_GAP0)
    log.append((t - T0, pl.a_mpc, pl.boost, pl.output_a_target, plant.a, plant.v, lb.v_f, lb.a_f, (lb.d_f - lb.d_stop) if lb.d_stop is not None else np.nan, st, K, rs.leadOne.dRel))
r = B.run(mb, cfgb, bm, T0 + 6, B.launch(T0, al), gap0=gap0, mode=mode, seed=1, noise_kw=dict(sd=0.3, sv=0.4, sa=0.3), probe=probe)
print(r.events)
print("   t    a_mpc boost  a_out  a_act  v_ego  v_f   a_f  opened spdT  gapT  dRel")
for l in log[::2]:
  print(" ".join(f"{x:5.2f}" if x is not None else "  -  " for x in l))
