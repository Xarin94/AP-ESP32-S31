# Y3Heavy: split tricopter mixer and SITL model

A tricopter whose two front rotors carry about 90% of the weight and the whole payload, with a small tilting rear rotor:

- front: 2 x Hobbywing X6 Plus G2 (150 KV, MFP 24x8.0), counter-rotating, 750 mm apart;
- rear: 350 KV, 13x10 three blade prop, CW, on a +-60 deg tilt servo, 720 mm behind the front axis (812 mm from each front motor);
- 12S; everything but the front units weighs 3.5 kg, battery included.

## Mixer: `MOT_TRI_MIX 1`

`FRAME_CLASS 7` (Tri) with `MOT_TRI_MIX 1` splits the axes between the actuators ([AP_MotorsTri.cpp](../../../libraries/AP_Motors/AP_MotorsTri.cpp), `output_armed_stabilizing_split()`):

```text
right  = T - 0.5 roll                    T: throttle, front pair only
left   = T + 0.5 roll
V      = RHOV * ramp - pitch             rear vertical thrust (nose up = less)
tilt   = atan(yaw * tan(SV_ANGLE) * RHOV / V)
rear   = V / cos(tilt)
```

- `MOT_TRI_RHOV`: rear thrust in hover, on the `MOT_THST_HOVER` scale. With the payload on the front axis the rear needs the same thrust whatever the payload, so in flight the feed-forward is just `RHOV`. `ramp` brings it in with the front throttle up to half `MOT_THST_HOVER`, so on the ground the rear never lifts the tail first. Start it low rather than high (default 0.15).
- `MOT_TRI_RH_LEARN` (default 2, learn and save): in a steady level hover, in the modes and conditions where `MOT_HOVER_LEARN` works (not STABILIZE or ACRO), the pitch output is what `RHOV` misses, and `RHOV` takes it over with a 5 s time constant; saved on disarm.
- Yaw is scaled by `RHOV / V`: the side force, and so the yaw moment, does not change with the thrust the rear carries for pitch.
- Priorities: on the front pair roll before throttle; on the rear pitch before yaw (the tilt is cut so `rear <= 1`). The `limit` flags are set, so the integrators stop at saturation.
- `MOT_TRI_MIX 0` (default) is the standard tricopter mixer, unchanged.

A first feed-forward scaled the rear by `throttle / MOT_THST_HOVER`. It pitched the vehicle 24 deg when 6 kg were added in hover, because `MOT_THST_HOVER` takes about 10 s to follow the payload. The constant feed-forward replaced it.

## SITL model: `y3heavy`

[SIM_Y3Heavy.cpp](../../../libraries/SITL/SIM_Y3Heavy.cpp). Each rotor has its own thrust and torque curve and its own speed lag. The tail servo is rate limited. Mass properties come from the components and are recomputed every step, so `SIM_Y3_PAYLOAD` can change in flight.

| Item | Value | Source |
|---|---|---|
| X6 throttle to rpm | Hobbywing 12S table, 33-100%; extended linearly to 0 rpm at 5.5% | Hobbywing X6 Plus G2 page |
| X6 thrust, torque | 3.35e-4 w^2 N, 9.78e-6 w^2 Nm (w in rad/s) | fitted on the table, spread < 2.5% |
| X6 ESC | speed held (12S and 14S tables match), fixed 1050-1950 us | Hobbywing |
| X6 unit mass | 0.748 kg with prop and cables | Hobbywing |
| X6 speed lag | 0.07 s (`SIM_Y3_FTAU`) | **estimate**: FOC speed loop, J about 1.5e-3 kg m^2, mechanical constant about 37 ms |
| Rear prop | CT 0.135, CP 0.088 (`SIM_Y3_R_CT`, `R_CP`) | **estimate** for a 13x10 three blade |
| Rear full speed | 8000 rpm at full command on 50.4 V (`SIM_Y3_RRPM`) | **placeholder**: set the real limit |
| Rear lag | 0.035 s (`SIM_Y3_RTAU`) | **estimate**: J about 3.5e-4 kg m^2, mechanical constant about 28 ms |
| Tail servo | 0.025 s small signal, 0.10 s per 60 deg (`SIM_Y3_SV_TAU`, `SV_T60`) | **estimates** |
| Rear unit mass | 0.45 kg, part of the 3.5 kg (`SIM_Y3_RMASS`) | **estimate** |
| CG | rear carries 10% with no payload (`SIM_Y3_RSHARE`): 72 mm behind the front axis | from the 90/10 split |
| Inertia, no payload | Ixx 0.226, Iyy 0.236, Izz 0.455 kg m^2 (4.996 kg) | point masses and boxes |
| Inertia, 6 kg payload | Ixx 0.289, Iyy 0.313, Izz 0.531 kg m^2 (10.996 kg) | |

Also modelled: rotor reaction torque including the rotor acceleration, gyroscopic moments, thrust change with axial speed (roll and pitch damping), momentum drag, body drag, 12S battery sag.

Rear rotor at the model's numbers: hover 4.90 N at 3000 rpm (53 W), idle at `MOT_SPIN_MIN` 1.69 N, full 34.95 N at 8000 rpm (1 kW). Nose down authority is the drop from hover to idle: 3.2 N x 0.65 m = 2.1 Nm.

## Parameters

[copter-y3heavy.parm](../default_params/copter-y3heavy.parm), loaded by `sim_vehicle.py -f y3heavy`:

- X6 thrust curve fitted on the Hobbywing table: `MOT_PWM_MIN 1050`, `MOT_PWM_MAX 1950`, `MOT_SPIN_MIN 0.22`, `MOT_SPIN_MAX 0.95`, `MOT_THST_EXPO 0` (linear above spin min).
- `MOT_BAT_VOLT_MAX 0`: the X6 ESC holds speed, so there is no voltage compensation.
- `MOT_THST_HOVER` 0.2 empty, about 0.44 with 6 kg. `MOT_TRI_RHOV` starts at 0.15 and is learned (the model needs 0.212).
- Rate and angle gains from `y3heavy_pid_design.py` (rear 10%): roll P/I 0.142 D 0.0057, pitch P/I 0.551 D 0.0152, yaw P 0.79 I 0.079; angle P 6.5 / 9 / 3; `ATC_ACC_R/P/Y_MAX` 600 / 200 / 200 deg/s/s (in this ArduPilot the old `ATC_ACCEL_*` names do not exist). `y3heavy-first-flight.parm` is the same at 60% for a first hover; `y3heavy-rear15.parm` is the set for 15% on the rear rotor.

The rear ESC must be calibrated to the same 1050-1950 us range as the X6. Limit its top speed in the ESC, not with `SERVOx_MAX`, so the mixer knows when the rear saturates.

## Build and run

The build needs `empy` (the ESP-IDF 6.1 py3.14 environment has it). The lwip submodule is not checked out, so networking is left out:

```sh
PY=~/.espressif/python_env/idf6.1_py3.14_env/bin/python
$PY ./waf configure --board sitl --disable-networking
$PY ./waf copter

# interactive, with MAVProxy console and map
Tools/autotest/sim_vehicle.py -v ArduCopter -f y3heavy -N --console --map

# rate and angle loop gains from the model (RSHARE 0.15: 15% of the weight on the rear)
Tools/autotest/y3heavy/y3heavy_pid_design.py --rshare 0.10

# scripted test flight and analysis (pymavlink, numpy, matplotlib)
Tools/autotest/y3heavy/y3heavy_flight.py /tmp/y3run
Tools/autotest/y3heavy/y3heavy_flight.py /tmp/y3loaded --payload 6
Tools/autotest/y3heavy/y3heavy_flight.py /tmp/y3stress --params stress.parm
Tools/autotest/y3heavy/y3heavy_analyse.py /tmp/y3run
```

Configuring for SITL replaces the board configuration: run `./waf configure --board esp32s31mini14` again before building the firmware.

## Results, 2026-11-20

SITL only. Gains from `y3heavy_pid_design.py` for each rear share; the same acceleration limits in both (`ATC_ACC_R/P/Y_MAX` 600 / 200 / 200 deg/s/s). Errors are desired minus actual attitude, maximum of roll and pitch over the phase. Stress: actuator lags x1.5, wind 8 m/s with turbulence 3.

| Test | 10% (defaults) | 15% (`y3heavy-rear15.parm`, same limits) |
|---|---|---|
| Roll steps 15 deg | 2.2 deg | 2.2 deg |
| Pitch steps 15 deg | 0.7 deg | 0.7 deg |
| 6 kg added at once | 1.8 deg | 2.7 deg |
| 6 kg dropped at once | 1.6 deg | 3.0 deg |
| 10 m/s cruise | 2.2 deg | 2.2 deg |
| Stress: roll / pitch steps | 2.9 / 1.5 deg | 2.9 / 2.0 deg |
| Stress: 6 kg added / dropped | 2.9 / 2.0 deg | 4.3 / 5.2 deg |
| Stress: 10 m/s | 2.8 deg | 2.6 deg |

- The rear reaches idle only when 6 kg arrive at once, 1-2% of that phase, in both cases; never in the pitch steps.
- Moving 5% of the weight aft (15% on the rear rotor) raises the nose down authority by 80% and the yaw authority by 47%, but gains nothing with these limits and costs 27 W in an empty hover. Every front thrust change pitches the vehicle in proportion to the CG distance from the front axis (108 mm against 72 mm), so payload changes get worse. Stay at 10%.
- `MOT_TRI_RH_LEARN`: from 0.15 the 10% vehicle reaches 0.192 after 10 s of hover and 0.212 (the model value) after about 30 s; from 0.30 it comes down to 0.215. At 15%, 0.25 goes to 0.327. Takeoff from the low start: 1.2 deg of pitch error.
- First-flight set (`y3heavy-first-flight.parm`, rate gains at 60%, `MOT_TRI_RHOV` learned from 0.15): up to 3.1 deg in the roll steps, 3.2 deg at the payload drop, 2.8 deg at takeoff.

## Open points

- Model numbers marked as estimates above: the X6 lag and the real rear top speed matter most.
- Nose down authority is small at 10% (2.1 Nm), so `ATC_ACC_P_MAX` stays at 200 deg/s/s. If real flights show the rear at `MOT_SPIN_MIN` often, 15% on the rear rotor nearly doubles it (`y3heavy-rear15.parm`). `MOT_SPIN_MIN` and the thrust curve are shared by all three motors.
- On the ground the SITL model keeps the vehicle level, so it cannot show a tail lift or nose-over at takeoff.
