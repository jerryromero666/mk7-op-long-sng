# Pure-python checks of launch_boost.py from a ref (state machine edges + live file parsing).
import sys, os, numpy as np, boostsim as B
from types import SimpleNamespace as NS
REF = sys.argv[1] if len(sys.argv) > 1 else 'WT'
LB = B.load_boost(REF)
LIVE = os.path.join(os.path.dirname(B.WT), 'livefile')
ok = True
def check(name, cond):
  global ok
  ok &= bool(cond)
  print(('PASS ' if cond else 'FAIL ') + name)
# live file: missing/garbage -> default, clamped, 0 disables
for f, exp in (('empty', LB.BOOST_MAX), ('garbage', LB.BOOST_MAX), ('nan', LB.BOOST_MAX), ('inf', LB.BOOST_MAX),
               ('binary', LB.BOOST_MAX), ('two', LB.BOOST_MAX), ('comma', LB.BOOST_MAX)):
  LB.LIVE_FILE = os.path.join(LIVE, f)
  check(f"live file '{f}' -> default", LB.read_live_max() == exp)
tmp = os.path.join(os.path.dirname(os.path.abspath(__file__)), '_live_tmp')
for txt, exp in (('1.2\n', 1.2), ('9', LB.BOOST_MAX_LIMIT), ('-3', 0.0), ('0', 0.0)):
  open(tmp, 'w').write(txt); LB.LIVE_FILE = tmp
  check(f"live file {txt.strip()!r} -> {exp}", abs(LB.read_live_max() - exp) < 1e-9)
LB.LIVE_FILE = '/nonexistent/x'
check("missing live file -> default", LB.read_live_max() == LB.BOOST_MAX)

def lead(d, v, a=0.0, st=True): return NS(status=st, dRel=d, vLead=v, aLeadK=a)
NOL = NS(status=False, dRel=0.0, vLead=0.0, aLeadK=0.0)
def drive(lb, frames, **kw):
  out = []
  for k in range(frames):
    p = {k2: (v(k) if callable(v) else v) for k2, v in kw.items()}
    out.append((lb.update(p.get('act', True), p.get('v', 0.0), p.get('gas', False), p.get('brake', False),
                          p['l1'], p.get('l2', NOL), p.get('mpc', 0.3), p.get('crash', 0)), lb.event))
  return out
# 1) departing lead before ARM_TIME of standstill: never fires
lb = LB.LaunchBoost()
o = drive(lb, 15, l1=lambda k: lead(5 + 0.02 * k * k, 0.1 * k, 1.5))
check("no fire before armed (0.75 s stopped)", all(b == 0 for b, _ in o))
# 2) armed, then departing lead: fires, boost > 0, ramps <= J_UP
lb = LB.LaunchBoost()
drive(lb, 30, l1=lead(5.0, 0.0))
o = drive(lb, 30, l1=lambda k: lead(5 + 0.04 * k * k, 0.08 * (k + 10), 1.5))
b = np.array([x for x, _ in o])
check("fires on clear departure", any(e.startswith('fired') for _, e in o) and b.max() > 0.5)
check("planner output max(MPC 0.3, boost) ramps within J_UP", np.max(np.diff(np.maximum(b, 0.3))) <= LB.J_UP * 0.05 + 1e-9)
check("boost <= BOOST_MAX", b.max() <= LB.BOOST_MAX + 1e-9)
# 3) MPC still asking to stop (a <= A_MPC_GO): never fires
lb = LB.LaunchBoost(); drive(lb, 30, l1=lead(5.0, 0.0))
o = drive(lb, 30, l1=lambda k: lead(5 + 0.04 * k * k, 0.08 * (k + 10), 1.5), mpc=0.05)
check("no fire while MPC says stop", all(b == 0 for b, _ in o))
# 4) brake press mid-boost: 0 at once, no re-arm until ego drove off
lb = LB.LaunchBoost(); drive(lb, 30, l1=lead(5.0, 0.0))
drive(lb, 10, l1=lambda k: lead(5 + 0.04 * k * k, 0.08 * (k + 10), 1.5))
o = drive(lb, 40, l1=lead(5.0, 0.0), brake=lambda k: k < 2)
check("brake aborts at once", o[0][0] == 0 and 'abort' in o[0][1])
check("no re-arm at same stop", all(b == 0 for b, _ in o) and not lb.armed)
# 5) leadTwo closer -> abort ; lead lost -> abort
for name, kw in (('leadTwo closer', dict(l2=lead(3.0, 0.0))), ('lead lost', dict(l1=NOL))):
  lb = LB.LaunchBoost(); drive(lb, 30, l1=lead(5.0, 0.0))
  drive(lb, 10, l1=lambda k: lead(5 + 0.04 * k * k, 0.08 * (k + 10), 1.5))
  if 'l1' not in kw: kw['l1'] = lead(9.5, 2.0, 1.5)
  o = drive(lb, 1, v=0.3, **kw)
  check(f"{name} aborts at once", o[0][0] == 0 and 'abort' in o[0][1])
# 6) boost 0 with live max 0
open(tmp, 'w').write('0'); LB.LIVE_FILE = tmp
lb = LB.LaunchBoost(); drive(lb, 30, l1=lead(5.0, 0.0))
o = drive(lb, 30, l1=lambda k: lead(5 + 0.04 * k * k, 0.08 * (k + 10), 1.5), mpc=0.3)
check("live max 0 -> no accel added", max(b for b, _ in o) == 0.0)
os.remove(tmp)
print('ALL PASS' if ok else 'SOME FAILED')
