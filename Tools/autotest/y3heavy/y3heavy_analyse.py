#!/usr/bin/env python3
"""
Analyse a y3heavy_flight.py run: per phase attitude and altitude
errors, motor and tail servo ranges, time at the motor limits and in
pitch PID limit; writes y3_flight.png and y3_steps.png in the work dir.

  y3heavy_analyse.py WORKDIR
"""
import glob
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from pymavlink import mavutil

work = sys.argv[1]
logs = sorted(glob.glob(os.path.join(work, 'logs', '*.BIN')), key=os.path.getmtime)
path = logs[-1]
phases = [l.rstrip('\n').split('\t') for l in open(os.path.join(work, 'phases.txt'))]
phases = [(n, int(a), int(b)) for n, a, b in phases]

want = {'ATT', 'RCOU', 'CTUN', 'RATE', 'PIDR', 'PIDP', 'PIDY', 'MSG', 'PARM', 'BAT', 'POS', 'NTUN', 'PSCN', 'PSCE'}
data = {k: [] for k in want}
params = {}
mlog = mavutil.mavlink_connection(path)
while True:
    msg = mlog.recv_match(type=list(want))
    if msg is None:
        break
    t = msg.get_type()
    if t == 'MSG':
        if 'Frame' in msg.Message or 'TRI' in msg.Message:
            print('MSG:', msg.Message)
        continue
    if t == 'PARM':
        params[msg.Name] = msg.Value
        continue
    d = msg.to_dict()
    d['t'] = msg.TimeUS * 1e-6
    data[t].append(d)


def arr(t, *fields):
    rows = data[t]
    return [np.array([r[f] for r in rows]) for f in ('t',) + fields]


t_att, droll, roll, dpitch, pitch, dyaw, yaw = arr('ATT', 'DesRoll', 'Roll', 'DesPitch', 'Pitch', 'DesYaw', 'Yaw')
t_rc, c1, c2, c4, c7 = arr('RCOU', 'C1', 'C2', 'C4', 'C7')
t_ct, tho, thh, alt, dalt = arr('CTUN', 'ThO', 'ThH', 'Alt', 'DAlt')
t_rate, rdes, r, pdes, p, ydes, y, rout, pout, yout = arr('RATE', 'RDes', 'R', 'PDes', 'P', 'YDes', 'Y', 'ROut', 'POut', 'YOut')
t_pp, p_i, p_lim = arr('PIDP', 'I', 'Flags') if data['PIDP'] else (np.array([]),) * 3
t_pr, r_i, r_lim = arr('PIDR', 'I', 'Flags') if data['PIDR'] else (np.array([]),) * 3
t_py, y_i, y_lim = arr('PIDY', 'I', 'Flags') if data['PIDY'] else (np.array([]),) * 3

pwm_min, pwm_max = params.get('MOT_PWM_MIN', 1050), params.get('MOT_PWM_MAX', 1950)
spin_min, spin_max = params.get('MOT_SPIN_MIN', 0.15), params.get('MOT_SPIN_MAX', 0.95)
pwm_lo = pwm_min + spin_min * (pwm_max - pwm_min)
pwm_hi = pwm_min + spin_max * (pwm_max - pwm_min)
print('frame params: MOT_TRI_MIX %s MOT_TRI_RHOV %s MOT_THST_HOVER %s' % (
    params.get('MOT_TRI_MIX'), params.get('MOT_TRI_RHOV'), params.get('MOT_THST_HOVER')))
print('motor pwm floor %.0f ceiling %.0f' % (pwm_lo, pwm_hi))

# phase times are boot ms from the GCS side, log times are boot us
print('\n%-20s %7s %7s %7s %7s | %5s %5s | %9s %9s %9s %7s | %4s %4s %4s' % (
    'phase', 'rollE', 'rollMx', 'pitE', 'pitMx', 'yawE', 'altE',
    'C1 range', 'C2 range', 'C4 range', 'C7', 'satF', 'satR', 'limP'))


def sel(t, a, b):
    return (t >= a / 1000) & (t <= b / 1000)


def yaw_err(d, a):
    return (d - a + 180) % 360 - 180


summary = []
for name, a, b in phases:
    s = sel(t_att, a, b)
    sr = sel(t_rc, a, b)
    sc = sel(t_ct, a, b)
    if not s.any() or not sr.any():
        continue
    re = droll[s] - roll[s]
    pe = dpitch[s] - pitch[s]
    ye = yaw_err(dyaw[s], yaw[s])
    ae = (dalt[sc] - alt[sc]) if sc.any() else np.array([0])
    front_sat = np.mean((c1[sr] >= pwm_hi - 1) | (c1[sr] <= pwm_lo + 1) | (c2[sr] >= pwm_hi - 1) | (c2[sr] <= pwm_lo + 1)) * 100
    rear_sat = np.mean((c4[sr] >= pwm_hi - 1) | (c4[sr] <= pwm_lo + 1)) * 100
    sp = sel(t_pp, a, b)
    limp = np.mean((p_lim[sp].astype(int) & 1) > 0) * 100 if sp.any() else 0
    print('%-20s %7.2f %7.2f %7.2f %7.2f | %5.1f %5.2f | %4.0f-%4.0f %4.0f-%4.0f %4.0f-%4.0f %3.0f-%3.0f | %4.0f %4.0f %4.0f' % (
        name, np.sqrt(np.mean(re**2)), np.max(np.abs(re)), np.sqrt(np.mean(pe**2)), np.max(np.abs(pe)),
        np.sqrt(np.mean(ye**2)), np.sqrt(np.mean(ae**2)),
        c1[sr].min(), c1[sr].max(), c2[sr].min(), c2[sr].max(), c4[sr].min(), c4[sr].max(),
        c7[sr].min() - 1500, c7[sr].max() - 1500, front_sat, rear_sat, limp))
    summary.append((name, a, b))

# hover operating point
for name, a, b in phases:
    if name in ('hover', 'payload 6 kg on', 'payload drain 20 s', 'drained', 'payload 6 kg on 2', 'loaded hover', 'payload dropped', 'fast north 10 m/s'):
        s = sel(t_rc, a + (b - a) / 2, b)
        sc = sel(t_ct, a + (b - a) / 2, b)
        sp = sel(t_pp, a + (b - a) / 2, b)
        print('%-16s C1 %.0f C2 %.0f C4 %.0f C7 %.0f  ThO %.3f ThH %.3f  pitch I %.3f' % (
            name, c1[s].mean(), c2[s].mean(), c4[s].mean(), c7[s].mean(), tho[sc].mean(), thh[sc].mean(),
            p_i[sp].mean() if sp.any() else float('nan')))

# plots
t0 = phases[0][1] / 1000
fig, ax = plt.subplots(6, 1, figsize=(14, 18), sharex=True)
ax[0].plot(t_att - t0, droll, 'k--', lw=0.8, label='DesRoll')
ax[0].plot(t_att - t0, roll, lw=1, label='Roll')
ax[0].set_ylabel('roll deg')
ax[1].plot(t_att - t0, dpitch, 'k--', lw=0.8, label='DesPitch')
ax[1].plot(t_att - t0, pitch, lw=1, label='Pitch')
ax[1].set_ylabel('pitch deg')
ax[2].plot(t_att - t0, yaw_err(dyaw, yaw), lw=1, label='yaw error')
ax[2].plot(t_rate - t0, ydes, 'k--', lw=0.8, label='YDes deg/s')
ax[2].plot(t_rate - t0, y, lw=0.8, label='Y deg/s')
ax[2].set_ylabel('yaw')
ax[3].plot(t_rc - t0, c1, lw=0.8, label='C1 right X6')
ax[3].plot(t_rc - t0, c2, lw=0.8, label='C2 left X6')
ax[3].plot(t_rc - t0, c4, lw=0.8, label='C4 rear')
ax[3].axhline(pwm_lo, color='grey', lw=0.5)
ax[3].axhline(pwm_hi, color='grey', lw=0.5)
ax[3].set_ylabel('pwm')
ax[4].plot(t_rc - t0, c7, lw=0.8, label='C7 tail servo')
ax[4].set_ylabel('pwm')
ax[5].plot(t_ct - t0, dalt, 'k--', lw=0.8, label='DAlt')
ax[5].plot(t_ct - t0, alt, lw=1, label='Alt')
ax[5].set_ylabel('alt m')
for a_ in ax:
    for name, a, b in phases:
        a_.axvspan(a / 1000 - t0, b / 1000 - t0, alpha=0.04 if phases.index((name, a, b)) % 2 else 0.0, color='k')
    a_.legend(loc='upper right', fontsize=7)
    a_.grid(alpha=0.3)
for name, a, b in phases:
    ax[0].text(a / 1000 - t0, ax[0].get_ylim()[1], name, rotation=90, fontsize=6, va='top')
ax[-1].set_xlabel('s')
fig.tight_layout()
out = os.path.join(work, 'y3_flight.png')
fig.savefig(out, dpi=90)
print('plot', out)

# zoom on the steps
fig, ax = plt.subplots(3, 1, figsize=(12, 9))
for k, (lbl, td, des, act) in enumerate([('roll', t_att, droll, roll), ('pitch', t_att, dpitch, pitch)]):
    for name, a, b in phases:
        if name.startswith(lbl) and 'centre' not in name:
            s = sel(td, a - 500, a + 3000)
            ax[k].plot(td[s] - a / 1000, des[s], 'k--', lw=0.8)
            ax[k].plot(td[s] - a / 1000, act[s], lw=1.2, label=name)
    ax[k].set_ylabel(lbl + ' deg')
    ax[k].legend(fontsize=8)
    ax[k].grid(alpha=0.3)
for name, a, b in phases:
    if name == 'yaw right':
        s = sel(t_rate, a - 500, b + 3000)
        ax[2].plot(t_rate[s] - a / 1000, ydes[s], 'k--', lw=0.8, label='YDes')
        ax[2].plot(t_rate[s] - a / 1000, y[s], lw=1.2, label='Y')
ax[2].set_ylabel('yaw rate deg/s')
ax[2].legend(fontsize=8)
ax[2].grid(alpha=0.3)
ax[2].set_xlabel('s from step')
fig.tight_layout()
out = os.path.join(work, 'y3_steps.png')
fig.savefig(out, dpi=90)
print('plot', out)
