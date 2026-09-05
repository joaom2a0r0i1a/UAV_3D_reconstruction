#!/usr/bin/env python3
# Pre-flight check: the active gain box, both planner boxes and the eval box must agree.
# Switching environment means editing four places; this catches a half-done switch.
import os, sys, yaml, re

CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'config')
KEYS = ('min_x', 'max_x', 'min_y', 'max_y', 'min_z', 'max_z')


def box(path, key):
    d = yaml.safe_load(open(path))
    if key not in d:
        return None
    return tuple(float(d[key][k]) for k in KEYS)


def label(path, key):
    # first line of the contiguous comment block directly above the active block
    block = []
    for line in open(path):
        if re.match(r'^%s:' % key, line):
            return block[0].lstrip('#').strip() if block else '(unlabelled)'
        if line.startswith('#'):
            block.append(line.rstrip())
        else:
            block = []
    return '(none)'


def fmt(b):
    return 'x[%g, %g] y[%g, %g] z[%g, %g]' % b if b else '(absent)'


gain_f = os.path.join(CFG, 'GainConfigReal.yaml')
aep_f = os.path.join(CFG, 'AEPlannerReal.yaml')
nbv_f = os.path.join(CFG, 'NBVplannerReal.yaml')
# Default comes from eval_real.sh, so this check can never disagree with what eval actually uses.
def default_eval_config():
    sh = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'eval_real.sh')
    m = re.search(r'EVAL_CONFIG="\$\{EVAL_CONFIG:-([^}]+)\}"', open(sh).read())
    return m.group(1) if m else 'PatioLamp.yaml'


eval_name = sys.argv[1] if len(sys.argv) > 1 else os.environ.get('EVAL_CONFIG') or default_eval_config()
eval_f = os.path.join(CFG, eval_name)

gain, aep, nbv = box(gain_f, 'gain_evaluation'), box(aep_f, 'bounded_box'), box(nbv_f, 'bounded_box')
ev = box(eval_f, 'bounded_box') if os.path.exists(eval_f) else None

print('  gain     %-34s %s' % (label(gain_f, 'gain_evaluation'), fmt(gain)))
print('  AEP bbx  %-34s %s' % (label(aep_f, 'bounded_box'), fmt(aep)))
print('  NBV bbx  %-34s %s' % (label(nbv_f, 'bounded_box'), fmt(nbv)))
print('  eval     %-34s %s' % (eval_name, fmt(ev)))

bad = []
if aep != nbv:
    bad.append('AEP and NBV planner boxes differ')
if ev is None:
    bad.append('eval box %s not found' % eval_name)
elif ev != gain:
    bad.append('eval box != gain box (coverage would be scored over a different volume)')
# Gain may extend past the planner box - the drone observes those voxels from inside it
# (uav_radius insets, ground below the flight floor). Only flag gain further out than the
# sensor can ever reach.
RANGE = float(yaml.safe_load(open(gain_f))['camera_intrinsics']['max_distance'])
if gain and aep:
    for i, ax in enumerate('xyz'):
        lo, hi = 2 * i, 2 * i + 1
        if gain[hi] > aep[hi] + RANGE:
            bad.append('gain max_%s (%g) is more than %g m (sensor range) beyond planner max_%s (%g)'
                       % (ax, gain[hi], RANGE, ax, aep[hi]))
        if gain[lo] < aep[lo] - RANGE:
            bad.append('gain min_%s (%g) is more than %g m (sensor range) below planner min_%s (%g)'
                       % (ax, gain[lo], RANGE, ax, aep[lo]))
print()
if bad:
    print('  MISMATCH:')
    for b in bad:
        print('    - ' + b)
    sys.exit(1)
print('  OK - gain, planner and eval boxes are consistent')
