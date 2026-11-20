#!/usr/bin/env python3
"""
Y3Heavy rate and angle loop design from the physical model.

For each axis: control effectiveness K (rad/s^2 per unit of ArduPilot
output) from masses, geometry and actuator curves, then the rate loop

  L(s) = PID(s) * Ferr(s) * K * A(s) * exp(-T s) * Gyro(s) / s

with A the actuator lag (front rotors, rear rotor, tail servo), Gyro the
2nd order INS_GYRO_FILTER, Ferr the error filter (yaw FLTE) and T the
loop and output delay. P and D are the fastest that keep the phase and
gain margins at nominal and in the worst case (slower actuators, K from
0.8 to 1.3 of nominal, 6 kg payload). I = P (roll, pitch), P/10 (yaw).

  y3heavy_pid_design.py [--rshare 0.10] [--ftau 0.07] [--rtau 0.035]
"""
import argparse
import cmath
import math

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
p.add_argument('--rshare', type=float, default=0.10)
p.add_argument('--ftau', type=float, default=0.07)
p.add_argument('--rtau', type=float, default=0.035)
p.add_argument('--stau', type=float, default=0.025, help='tail servo small signal lag, s')
p.add_argument('--payload', type=float, default=0.0)
p.add_argument('--quiet', action='store_true')
args = p.parse_args()

G = 9.80665
SPAN, ARM = 0.375, 0.72
M_FRONT, M_REST, M_REAR = 0.748, 3.5, 0.45
SV_ANGLE = math.radians(60)
SPIN_MIN, SPIN_MAX = 0.22, 0.95
DACT_DT = SPIN_MAX - SPIN_MIN          # actuator per unit of thrust output
REAR_TMAX = 34.95                      # N at 8000 rpm, CT 0.135, 13 in
X6 = [(5.5, 0), (33, 1736), (35, 2017), (37, 2318), (39, 2634), (42, 3131), (45, 3646), (48, 4174), (51, 4712),
      (54, 5256), (57, 5807), (60, 6364), (63, 6927), (66, 7495), (69, 8068), (72, 8644), (75, 9219),
      (78, 9786), (81, 10337), (84, 10861), (87, 11342), (90, 11762), (100, 12526)]
X6_J = 1.8e-3


def x6_slope_at_thrust(t_newton):
    """thrust slope, N per unit of actuator (0..1), at a thrust"""
    g = t_newton / G * 1000
    for (a0, t0), (a1, t1) in zip(X6, X6[1:]):
        if g <= t1:
            return (t1 - t0) / (a1 - a0) * 100 * G / 1000, a0 + (g - t0) / (t1 - t0) * (a1 - a0)
    raise ValueError


def vehicle(rshare, payload):
    m_rest = M_REST - M_REAR
    m_empty = 2 * M_FRONT + M_REST
    x_rest = (M_REAR * ARM - m_empty * rshare * ARM) / m_rest
    pts = [(M_FRONT, 0, SPAN), (M_FRONT, 0, -SPAN), (M_REAR, -ARM, 0), (m_rest, x_rest, 0), (payload, 0, 0)]
    m = m_empty + payload
    cg = sum(mi * x for mi, x, _ in pts) / m
    ixx = sum(mi * y * y for mi, x, y in pts)
    iyy = sum(mi * (x - cg) ** 2 for mi, x, y in pts)
    izz = ixx + iyy
    box = [(m_rest, (0.30, 0.20, 0.12)), (payload, (0.25, 0.25, 0.25))]
    for mi, (a, b, c) in box:
        ixx += mi * (b * b + c * c) / 12
        iyy += mi * (a * a + c * c) / 12
        izz += mi * (a * a + b * b) / 12
    ixx += X6_J
    iyy += X6_J
    izz += 2 * X6_J
    rear_t = rshare * m_empty * G
    front_t = (m * G - rear_t) / 2
    slope, front_pct = x6_slope_at_thrust(front_t)
    arm_r = ARM + cg
    k_roll = SPAN * slope * DACT_DT / ixx
    a_rear = math.sqrt(rear_t / REAR_TMAX)
    k_pitch = arm_r * 2 * math.sqrt(rear_t * REAR_TMAX) * DACT_DT / iyy
    k_yaw = arm_r * rear_t * math.tan(SV_ANGLE) / izz
    rear_min = REAR_TMAX * SPIN_MIN ** 2
    return dict(m=m, cg=cg, ixx=ixx, iyy=iyy, izz=izz, front_t=front_t, front_pct=front_pct, rear_t=rear_t,
                a_rear=a_rear, k_roll=k_roll, k_pitch=k_pitch, k_yaw=k_yaw,
                nose_down=(rear_t - rear_min) * arm_r / iyy, nose_up=(REAR_TMAX - rear_t) * arm_r / iyy,
                yaw_max=k_yaw, rhov=((a_rear - SPIN_MIN) / DACT_DT))


T_DELAY = 0.005            # loop, output and ESC update
GYRO_HZ = 20.0
FLTD_HZ = 20.0


def loop(w, P, I, D, K, tau, flte_hz=0.0):
    s = 1j * w
    wd = 2 * math.pi * FLTD_HZ
    c = P + I / s + D * s / (1 + s / wd)
    if flte_hz > 0:
        c /= (1 + s / (2 * math.pi * flte_hz))
    wn = 2 * math.pi * GYRO_HZ
    gyro = wn * wn / (s * s + math.sqrt(2) * wn * s + wn * wn)
    return c * K / (tau * s + 1) * cmath.exp(-s * T_DELAY) * gyro / s


def margins(P, I, D, K, tau, flte_hz=0.0):
    ws = [10 ** (k / 400) for k in range(-400, 1400)]
    pm, wc, gm, w180 = None, None, None, None
    prev = None
    for w in ws:
        lval = loop(w, P, I, D, K, tau, flte_hz)
        mag, ph = abs(lval), math.degrees(cmath.phase(lval))
        if prev is not None:
            if prev[1] >= 1 > mag and wc is None:
                wc, pm = w, 180 + ph if ph < 0 else ph - 180
            # phase crossing of -180 deg: sign change of imag with real < 0
            if prev[2].imag < 0 <= lval.imag and lval.real < 0 and w180 is None:
                w180, gm = w, -20 * math.log10(mag)
        prev = (w, mag, lval)
    return pm, wc, gm, w180


def design(K_nom, tau_nom, worst, i_ratio, flte_hz, td_max=0.04, pm_nom=50, pm_worst=38, gm_min=8):
    best = None
    for td in [k * 0.0025 for k in range(int(td_max / 0.0025) + 1)]:
        lo, hi = 0.001, 5.0
        for _ in range(50):
            P = math.sqrt(lo * hi)
            ok = True
            pm, wc, gm, _ = margins(P, P * i_ratio, P * td, K_nom, tau_nom, flte_hz)
            if pm is None or pm < pm_nom or (gm is not None and gm < gm_min):
                ok = False
            for K, tau in worst:
                pmw, _, gmw, _ = margins(P, P * i_ratio, P * td, K, tau, flte_hz)
                if pmw is None or pmw < pm_worst or (gmw is not None and gmw < gm_min - 2):
                    ok = False
            if ok:
                lo = P
            else:
                hi = P
        P = lo
        pm, wc, gm, _ = margins(P, P * i_ratio, P * td, K_nom, tau_nom, flte_hz)
        if best is None or wc > best['wc'] * 1.03:
            best = dict(P=P, I=P * i_ratio, D=P * td, td=td, pm=pm, wc=wc, gm=gm)
    worst_m = [margins(best['P'], best['I'], best['D'], K, tau, flte_hz) for K, tau in worst]
    best['worst_pm'] = min(w[0] for w in worst_m)
    best['worst_gm'] = min(w[2] for w in worst_m if w[2] is not None)
    return best


def angle_p(rate, K, tau, flte_hz=0.0, pm_target=60):
    """angle P: outer loop P_ang * T_rate(s) / s with the given phase margin"""
    def outer(w, pa):
        lval = loop(w, rate['P'], rate['I'], rate['D'], K, tau, flte_hz)
        t_rate = lval / (1 + lval)
        return pa * t_rate / (1j * w)
    lo, hi = 0.2, 20
    for _ in range(40):
        pa = math.sqrt(lo * hi)
        w = 0.01
        while abs(outer(w, pa)) > 1:
            w *= 1.01
        pm = 180 + math.degrees(cmath.phase(outer(w, pa)))
        if pm >= pm_target:
            lo = pa
        else:
            hi = pa
    return lo


nom = vehicle(args.rshare, 0.0)
full = vehicle(args.rshare, 6.0)
if not args.quiet:
    for name, v in (('empty', nom), ('6 kg', full)):
        print('%-6s m %.2f kg  CG %.3f m behind front axis  I %.3f %.3f %.3f  front %.1f N (%.0f%%)  rear %.2f N  '
              'K roll %.0f pitch %.1f yaw %.1f  nose-down %.1f up %.0f rad/s2  RHOV %.3f' % (
                  name, v['m'], -v['cg'], v['ixx'], v['iyy'], v['izz'], v['front_t'], v['front_pct'], v['rear_t'],
                  v['k_roll'], v['k_pitch'], v['k_yaw'], v['nose_down'], v['nose_up'], v['rhov']))

axes = {
    'RLL': (nom['k_roll'], args.ftau, [(nom['k_roll'] * 1.3, args.ftau * 1.5), (full['k_roll'] * 0.8, args.ftau * 1.5),
                                       (nom['k_roll'], args.ftau * 2.0)], 1.0, 0.0),
    'PIT': (nom['k_pitch'], args.rtau, [(nom['k_pitch'] * 1.3, args.rtau * 1.5), (full['k_pitch'] * 0.8, args.rtau * 1.5),
                                        (nom['k_pitch'], args.rtau * 2.0)], 1.0, 0.0),
    'YAW': (nom['k_yaw'], args.stau, [(nom['k_yaw'] * 1.3, args.stau * 1.5), (full['k_yaw'] * 0.8, args.stau * 1.5),
                                      (nom['k_yaw'], args.stau * 2.0)], 0.1, 5.0),
}
for ax, (K, tau, worst, i_ratio, flte) in axes.items():
    r = design(K, tau, worst, i_ratio, flte, td_max=0.0 if ax == 'YAW' else 0.04)
    pa = angle_p(r, K, tau, flte)
    print('%s P %.4f I %.4f D %.4f  (D/P %.3f s)  crossover %.1f rad/s  PM %.0f deg  GM %.1f dB  '
          'worst PM %.0f GM %.1f  ANG_P %.1f' % (ax, r['P'], r['I'], r['D'], r['td'], r['wc'], r['pm'], r['gm'],
                                                  r['worst_pm'], r['worst_gm'], pa))
