#!/usr/bin/env python3
"""
Y3Heavy SITL test flight: takeoff, attitude steps in ALT_HOLD, payload
added, drained and dropped in hover, 10 m/s translation, landing.
Starts the SITL binary itself; analyse the result with y3heavy_analyse.py.

  y3heavy_flight.py WORKDIR [--params FILE] [--payload KG] [--speedup N]

--params sets extra parameters after boot (SIM_Y3_*, SIM_WIND_*, gains);
--payload is the payload at takeoff (then the flight drops it in hover
instead of the add/drain/drop sequence).
"""
import argparse
import os
import subprocess
import time

from pymavlink import mavutil

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument('work', help='directory for eeprom, logs and phases.txt')
parser.add_argument('--binary', default=os.path.join(ROOT, 'build', 'sitl', 'bin', 'arducopter'))
parser.add_argument('--params', default=None, help='extra parameter file, NAME VALUE per line')
parser.add_argument('--payload', type=float, default=0, help='payload at takeoff, kg')
parser.add_argument('--speedup', type=int, default=8)
args = parser.parse_args()

work = os.path.abspath(args.work)
extra_params = args.params
initial_payload = args.payload
os.makedirs(work, exist_ok=True)

sitl_log = open(os.path.join(work, 'sitl.log'), 'w')
proc = subprocess.Popen([args.binary, '--model', 'y3heavy', '--speedup', str(args.speedup), '-w',
                         '--home', '-35.363261,149.165230,584,0'],
                        cwd=work, stdout=sitl_log, stderr=subprocess.STDOUT)

phases = []   # (name, t_start_ms, t_end_ms)


def log(msg):
    print(msg, flush=True)


try:
    m = None
    for _ in range(50):
        try:
            m = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255)
            break
        except Exception:
            time.sleep(0.2)
    m.wait_heartbeat(timeout=30)
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    state = {'t': 0, 'alt': 0.0, 'armed': False, 'mode': '', 'gps': False}

    def pump(timeout=0.05):
        msg = m.recv_match(blocking=True, timeout=timeout)
        while msg is not None:
            t = msg.get_type()
            if t == 'SYSTEM_TIME':
                state['t'] = msg.time_boot_ms
            elif t == 'ATTITUDE':
                state['t'] = msg.time_boot_ms
            elif t == 'GLOBAL_POSITION_INT':
                state['alt'] = msg.relative_alt * 1e-3
                state['lat'], state['lon'] = msg.lat, msg.lon
            elif t == 'HEARTBEAT' and msg.get_srcSystem() == m.target_system:
                state['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                state['mode'] = mavutil.mode_string_v10(msg)
            elif t == 'STATUSTEXT':
                log('  AP: %s' % msg.text)
                if 'is using GPS' in msg.text:
                    state['gps'] = True
            msg = m.recv_match(blocking=False)

    def sim_sleep(sec, rc=None):
        pump()
        t0 = state['t']
        while state['t'] - t0 < sec * 1000:
            if rc is not None:
                m.mav.rc_channels_override_send(m.target_system, m.target_component, *rc)
            pump(0.02)

    def wait(cond, sec, what):
        pump()
        t0 = state['t']
        while not cond():
            pump(0.02)
            if state['t'] - t0 > sec * 1000:
                raise RuntimeError('timeout waiting for ' + what)

    def set_param(name, value):
        m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(value),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        # the autopilot echoes a set parameter; no echo = no such parameter
        deadline = time.time() + 3
        while time.time() < deadline:
            msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
            if msg is not None and msg.param_id == name:
                return
        raise RuntimeError('parameter %s not set: no such parameter?' % name)

    def get_param(name):
        m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
        deadline = time.time() + 3
        while time.time() < deadline:
            msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
            if msg is not None and msg.param_id == name:
                return msg.param_value
        return float('nan')

    def set_mode(mode):
        m.set_mode(mode)
        wait(lambda: state['mode'] == mode, 10, 'mode ' + mode)

    def phase(name, sec, rc=None):
        t0 = state['t']
        sim_sleep(sec, rc)
        phases.append((name, t0, state['t']))
        log('phase %-22s %6.1f s  alt %.1f  MOT_TRI_RHOV %.3f' % (name, (state['t'] - t0) / 1000, state['alt'],
                                                                get_param('MOT_TRI_RHOV')))

    def goto(north_m, east_m, alt):
        import math
        lat = -35.363261 + north_m / 111319.5
        lon = 149.165230 + east_m / (111319.5 * math.cos(math.radians(-35.363261)))
        m.mav.set_position_target_global_int_send(
            0, m.target_system, m.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b0000111111111000,
            int(lat * 1e7), int(lon * 1e7), alt, 0, 0, 0, 0, 0, 0, 0, 0)

    sim_sleep(2)
    set_param('LOG_DISARMED', 0)
    set_param('LOG_BITMASK', 180222)   # default plus RCOUT, IMU fast enough for this
    if extra_params:
        for line in open(extra_params):
            line = line.split('#')[0].split()
            if len(line) >= 2:
                set_param(line[0], line[1])

    set_param('SIM_Y3_PAYLOAD', initial_payload)
    log('waiting for EKF and GPS')
    wait(lambda: state['gps'], 120, 'EKF using GPS')
    set_mode('GUIDED')
    armed = False
    for _ in range(60):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        sim_sleep(2)
        if state['armed']:
            armed = True
            break
    if not armed:
        raise RuntimeError('could not arm')
    log('armed')

    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
    t0 = state['t']
    wait(lambda: state['alt'] > 9.5, 60, 'takeoff')
    phases.append(('takeoff', t0, state['t']))
    phase('hover', 10)

    set_mode('ALT_HOLD')
    c = [1500, 1500, 1500, 1500, 0, 0, 0, 0]

    def rc(**kw):
        r = list(c)
        for k, v in kw.items():
            r[int(k[2:]) - 1] = v
        return r

    phase('settle', 3, rc())
    phase('roll right 15', 3, rc(ch1=1750))
    phase('roll centre', 3, rc())
    phase('roll left 15', 3, rc(ch1=1250))
    phase('roll centre 2', 3, rc())
    phase('pitch fwd 15', 3, rc(ch2=1250))
    phase('pitch centre', 3, rc())
    phase('pitch back 15', 3, rc(ch2=1750))
    phase('pitch centre 2', 3, rc())
    phase('yaw right', 3, rc(ch4=1800))
    phase('yaw centre', 4, rc())

    # release the override and go back to GUIDED
    m.mav.rc_channels_override_send(m.target_system, m.target_component, *([0] * 8))
    set_mode('GUIDED')
    goto(0, 0, 10)
    phase('guided hover', 5)
    if initial_payload == 0:
        set_param('SIM_Y3_PAYLOAD', 6)
        phase('payload 6 kg on', 12)
        # spraying: 6 kg to 0 in 20 s
        t0 = state['t']
        for k in range(1, 41):
            set_param('SIM_Y3_PAYLOAD', 6 * (1 - k / 40))
            sim_sleep(0.3)
        phases.append(('payload drain 20 s', t0, state['t']))
        log('phase payload drain 20 s')
        phase('drained', 5)
        set_param('SIM_Y3_PAYLOAD', 6)
        phase('payload 6 kg on 2', 10)
        set_param('SIM_Y3_PAYLOAD', 0)
        phase('payload dropped', 10)
    else:
        phase('loaded hover', 10)
        set_param('SIM_Y3_PAYLOAD', 0)
        phase('payload dropped', 10)

    set_param('WP_SPD', 10)
    goto(150, 0, 10)
    phase('fast north 10 m/s', 25)
    goto(0, 0, 10)
    phase('fast back', 25)

    set_mode('LAND')
    t0 = state['t']
    wait(lambda: not state['armed'], 90, 'landing')
    phases.append(('land', t0, state['t']))
    log('landed and disarmed')
finally:
    with open(os.path.join(work, 'phases.txt'), 'w') as f:
        for name, a, b in phases:
            f.write('%s\t%d\t%d\n' % (name, a, b))
    time.sleep(1)
    proc.terminate()
    proc.wait()
