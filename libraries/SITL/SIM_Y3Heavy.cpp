/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Y3Heavy tricopter simulator, see SIM_Y3Heavy.h
*/

#include "SIM_Y3Heavy.h"

#if AP_SIM_Y3HEAVY_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "SITL.h"

using namespace SITL;

const AP_Param::GroupInfo Y3Heavy::var_info[] = {
    // @Param: PAYLOAD
    // @DisplayName: Y3Heavy payload
    // @Description: Payload mass, carried on the front rotor axis. Can be changed in flight
    // @Units: kg
    // @Range: 0 15
    // @User: Advanced
    AP_GROUPINFO("PAYLOAD", 1, Y3Heavy, payload_kg, 0),

    // @Param: RSHARE
    // @DisplayName: Y3Heavy rear weight share
    // @Description: Share of the weight the rear rotor carries in hover with no payload. Sets where the CG is
    // @Range: 0.02 0.4
    // @User: Advanced
    AP_GROUPINFO("RSHARE", 2, Y3Heavy, rear_share, 0.10),

    // @Param: FMASS
    // @DisplayName: Y3Heavy front unit mass
    // @Description: Mass of each front propulsion unit (motor, ESC, prop, cables)
    // @Units: kg
    // @User: Advanced
    AP_GROUPINFO("FMASS", 3, Y3Heavy, front_unit_kg, 0.748),

    // @Param: BMASS
    // @DisplayName: Y3Heavy rest mass
    // @Description: Mass of everything but the front units, battery and rear unit included
    // @Units: kg
    // @User: Advanced
    AP_GROUPINFO("BMASS", 4, Y3Heavy, rest_kg, 3.5),

    // @Param: RMASS
    // @DisplayName: Y3Heavy rear unit mass
    // @Description: Mass of the rear unit (motor, prop, tilt servo, mount), part of SIM_Y3_BMASS
    // @Units: kg
    // @User: Advanced
    AP_GROUPINFO("RMASS", 5, Y3Heavy, rear_unit_kg, 0.45),

    // @Param: FSPAN
    // @DisplayName: Y3Heavy front half span
    // @Description: Distance of each front rotor from the centre line
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("FSPAN", 6, Y3Heavy, front_half_span, 0.375),

    // @Param: RARM
    // @DisplayName: Y3Heavy rear arm
    // @Description: Distance of the rear rotor behind the front rotor axis
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("RARM", 7, Y3Heavy, rear_arm, 0.72),

    // @Param: RRPM
    // @DisplayName: Y3Heavy rear rotor full speed
    // @Description: Rear rotor speed at full command on a full 12S pack, where the rear ESC throttle range is limited. Scales with battery voltage
    // @Units: RPM
    // @User: Advanced
    AP_GROUPINFO("RRPM", 8, Y3Heavy, rear_rpm_max, 8000),

    // @Param: FTAU
    // @DisplayName: Y3Heavy front rotor time constant
    // @Description: First order time constant of the front rotor speed. 0.07 s is an estimate for a speed-controlled FOC ESC on a 24 inch prop
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("FTAU", 9, Y3Heavy, front_tau, 0.07),

    // @Param: RTAU
    // @DisplayName: Y3Heavy rear rotor time constant
    // @Description: First order time constant of the rear rotor speed. 0.035 s is an estimate
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("RTAU", 10, Y3Heavy, rear_tau, 0.035),

    // @Param: SV_T60
    // @DisplayName: Y3Heavy tail servo speed
    // @Description: Time the tail servo takes to move 60 degrees
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("SV_T60", 11, Y3Heavy, servo_time_60, 0.10),

    // @Param: R_CT
    // @DisplayName: Y3Heavy rear prop thrust coefficient
    // @Description: Static thrust coefficient of the rear prop, T = CT rho n^2 D^4
    // @User: Advanced
    AP_GROUPINFO("R_CT", 12, Y3Heavy, rear_ct, 0.135),

    // @Param: R_CP
    // @DisplayName: Y3Heavy rear prop power coefficient
    // @Description: Static power coefficient of the rear prop, P = CP rho n^3 D^5
    // @User: Advanced
    AP_GROUPINFO("R_CP", 13, Y3Heavy, rear_cp, 0.088),

    // @Param: R_DIAM
    // @DisplayName: Y3Heavy rear prop diameter
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("R_DIAM", 14, Y3Heavy, rear_diameter, 0.3302),

    // @Param: FSPIN
    // @DisplayName: Y3Heavy front spin directions
    // @Description: Spin directions of the front pair, seen from above. The rear rotor turns clockwise
    // @Values: 0:right CCW left CW,1:right CW left CCW
    // @User: Advanced
    AP_GROUPINFO("FSPIN", 15, Y3Heavy, front_spin, 0),

    // @Param: SV_TAU
    // @DisplayName: Y3Heavy tail servo time constant
    // @Description: First order time constant of the tail servo for small moves, before its speed limit SIM_Y3_SV_T60
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("SV_TAU", 16, Y3Heavy, servo_tau, 0.025),

    AP_GROUPEND
};

namespace {

/*
  Hobbywing X6 Plus G2 with MFP 24x8.0 at 12S (46V), Hobbywing load
  test table: throttle in percent to speed in rpm. The 12S and 14S
  tables give the same speed at the same throttle, so the ESC holds
  speed rather than duty. The table starts at 33%; below that the
  speed is extended linearly to zero at 5.5%
 */
const struct {
    float pct;
    float rpm;
} x6_rpm_table[] = {
    {  5.5,    0 }, { 33, 2169 }, { 35, 2327 }, { 37, 2486 }, { 39, 2646 },
    { 42, 2881 }, { 45, 3107 }, { 48, 3325 }, { 51, 3534 }, { 54, 3735 },
    { 57, 3929 }, { 60, 4116 }, { 63, 4298 }, { 66, 4474 }, { 69, 4645 },
    { 72, 4810 }, { 75, 4968 }, { 78, 5119 }, { 81, 5260 }, { 84, 5391 },
    { 87, 5508 }, { 90, 5610 }, { 100, 5760 },
};

// thrust and torque over speed squared, fitted on the same table at
// sea level (spread under 2.5% over the whole table)
constexpr float X6_KT = 3.35e-4f;          // N / (rad/s)^2
constexpr float X6_KQ = 9.78e-6f;          // N m / (rad/s)^2
// the 100% row, 5760 rpm at 46 V (150 KV), is the most the voltage allows
constexpr float X6_SPEED_PER_VOLT = 5760.0f / 46.0f;
// the X6 ESC has a fixed throttle range, no calibration
constexpr float X6_PWM_MIN = 1050.0f;
constexpr float X6_PWM_MAX = 1950.0f;
constexpr float X6_PROP_RADIUS = 0.3048f;  // 24 inch
constexpr float X6_ROTOR_J = 1.8e-3f;      // kg m^2, prop and motor bell, estimated from the 107 g prop

// rear ESC calibrated to the same range as the X6
constexpr float REAR_PWM_MIN = X6_PWM_MIN;
constexpr float REAR_PWM_MAX = X6_PWM_MAX;
constexpr float REAR_FULL_VOLTAGE = 50.4f; // SIM_Y3_RRPM is defined on a full 12S pack
constexpr float REAR_ROTOR_J = 5.0e-4f;    // kg m^2, estimated

// electrical to shaft power, from the X6 table at 60% throttle
constexpr float PROPULSION_EFFICIENCY = 0.86f;

// rotor drag in edgewise flow, times sqrt(T rho A)
constexpr float MOMENTUM_DRAG_COEF = 0.15f;

// body drag areas times Cd, m^2
const Vector3f BODY_CDA { 0.06f, 0.08f, 0.25f };

// the rest of the mass and the payload are boxes, m
const Vector3f BODY_BOX { 0.30f, 0.20f, 0.12f };
const Vector3f PAYLOAD_BOX { 0.25f, 0.25f, 0.25f };

// tail servo: 1000 to 2000 us is +60 to -60 deg of tilt, as for the
// SITL tri frame; SERVO7 pwm above trim yaws the nose right
constexpr float TAIL_PWM_TRIM = 1500.0f;
constexpr float TAIL_PWM_HALF_RANGE = 500.0f;
constexpr float TAIL_ANGLE_MAX_DEG = 60.0f;

// moments of inertia of a solid box about its own centre
Vector3f box_inertia(float mass, const Vector3f &size)
{
    return Vector3f(mass * (sq(size.y) + sq(size.z)) / 12,
                    mass * (sq(size.x) + sq(size.z)) / 12,
                    mass * (sq(size.x) + sq(size.y)) / 12);
}

// moments of inertia of a point mass about the origin
Vector3f point_inertia(float mass, const Vector3f &pos)
{
    return Vector3f(mass * (sq(pos.y) + sq(pos.z)),
                    mass * (sq(pos.x) + sq(pos.z)),
                    mass * (sq(pos.x) + sq(pos.y)));
}

}  // namespace

Y3Heavy::Y3Heavy(const char *frame_str) :
    Aircraft(frame_str)
{
    AP::sitl()->models.y3heavy_ptr = this;
    AP_Param::setup_object_defaults(this, var_info);

    // 12S pack; SIM_BATT_VOLTAGE has to be set to 50.4, or the reset
    // in update() brings the pack down to its 12.6 V default
    battery.setup(16.0f, 0.015f, 50.4f, ambient_outside_temperature_degC());

    frame_height = 0.25f;
    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;
    lock_step_scheduled = true;

    update_mass_properties();
    mass = total_mass;
}

void Y3Heavy::update_mass_properties()
{
    const float m_front = MAX(front_unit_kg.get(), 0.05f);
    const float m_rear = MAX(rear_unit_kg.get(), 0.01f);
    const float m_rest = MAX(rest_kg.get() - m_rear, 0.1f);
    const float m_payload = MAX(payload_kg.get(), 0.0f);
    const float span = front_half_span;
    const float arm = MAX(rear_arm.get(), 0.1f);
    const float m_empty = 2 * m_front + m_rear + m_rest;

    // put the rest of the mass where the rear rotor carries rear_share
    // of the empty weight: CG rear_share * arm behind the front axis
    const float share = constrain_float(rear_share, 0.0f, 0.9f);
    const float x_rest = (m_rear * arm - m_empty * share * arm) / m_rest;

    const Vector3f p_right(0, span, 0);
    const Vector3f p_left(0, -span, 0);
    const Vector3f p_rear(-arm, 0, 0);
    const Vector3f p_rest(x_rest, 0, 0);
    const Vector3f p_payload(0, 0, 0);

    total_mass = m_empty + m_payload;
    cg = (p_right * m_front + p_left * m_front + p_rear * m_rear +
          p_rest * m_rest + p_payload * m_payload) / total_mass;

    inertia = point_inertia(m_front, p_right - cg) +
              point_inertia(m_front, p_left - cg) +
              point_inertia(m_rear, p_rear - cg) +
              point_inertia(m_rest, p_rest - cg) + box_inertia(m_rest, BODY_BOX) +
              point_inertia(m_payload, p_payload - cg) + box_inertia(m_payload, PAYLOAD_BOX);

    // the front props about their own axes; a two blade prop has about
    // half that about a diameter on average
    inertia += Vector3f(X6_ROTOR_J, X6_ROTOR_J, 2 * X6_ROTOR_J);
}

// steady front rotor speed for a pwm, rpm
float Y3Heavy::front_rpm_demand(uint16_t pwm, float voltage) const
{
    const float pct = 100 * constrain_float((pwm - X6_PWM_MIN) / (X6_PWM_MAX - X6_PWM_MIN), 0, 1);
    float speed = 0;
    if (pct > x6_rpm_table[0].pct) {
        speed = x6_rpm_table[ARRAY_SIZE(x6_rpm_table)-1].rpm;
        for (uint8_t i = 1; i < ARRAY_SIZE(x6_rpm_table); i++) {
            if (pct <= x6_rpm_table[i].pct) {
                speed = linear_interpolate(x6_rpm_table[i-1].rpm, x6_rpm_table[i].rpm, pct,
                                           x6_rpm_table[i-1].pct, x6_rpm_table[i].pct);
                break;
            }
        }
    }
    // the ESC holds speed, until the voltage runs out
    return MIN(speed, X6_SPEED_PER_VOLT * voltage);
}

// steady rear rotor speed for a pwm, rpm: a duty-cycle ESC, so the speed
// follows command times voltage
float Y3Heavy::rear_rpm_demand(uint16_t pwm, float voltage) const
{
    const float command = constrain_float((pwm - REAR_PWM_MIN) / (REAR_PWM_MAX - REAR_PWM_MIN), 0, 1);
    return command * rear_rpm_max * voltage / REAR_FULL_VOLTAGE;
}

void Y3Heavy::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel, float &current_amp)
{
    const float dt = frame_time_us * 1.0e-6f;
    const float voltage = MAX(battery_voltage, 1.0f);
    const float density_ratio = air_density / SSL_AIR_DENSITY;

    // rotor speeds: first order lag towards the ESC demand
    const float rpm_demand[NUM_ROTORS] {
        front_rpm_demand(input.servos[0], voltage),   // SERVO1, motor 1, right
        front_rpm_demand(input.servos[1], voltage),   // SERVO2, motor 2, left
        rear_rpm_demand(input.servos[3], voltage),    // SERVO4, motor 4, rear
    };
    const float tau[NUM_ROTORS] { front_tau, front_tau, rear_tau };
    for (uint8_t i = 0; i < NUM_ROTORS; i++) {
        const float target = rpm_demand[i] * float(M_PI / 30);
        omega_dot[i] = (target - omega[i]) / MAX(tau[i], 0.005f);
        omega[i] = MAX(omega[i] + omega_dot[i] * dt, 0.0f);
    }

    // tail servo: first order for small moves, rate limited for large
    // ones. No pulse: the servo holds where it is
    if (input.servos[6] != 0) {
        const float demand = -radians(TAIL_ANGLE_MAX_DEG) *
            constrain_float((input.servos[6] - TAIL_PWM_TRIM) / TAIL_PWM_HALF_RANGE, -1, 1);
        const float lag_step = (demand - tail_angle) * MIN(dt / MAX(servo_tau.get(), dt), 1.0f);
        const float max_step = radians(60) / MAX(servo_time_60.get(), 0.01f) * dt;
        tail_angle += constrain_float(lag_step, -max_step, max_step);
    }

    // rotor geometry about the CG, body frame
    const float span = front_half_span;
    const float arm = rear_arm;
    const Vector3f pos[NUM_ROTORS] {
        Vector3f(0, span, 0) - cg,
        Vector3f(0, -span, 0) - cg,
        Vector3f(-arm, 0, 0) - cg,
    };
    // thrust direction
    const Vector3f axis[NUM_ROTORS] {
        Vector3f(0, 0, -1),
        Vector3f(0, 0, -1),
        Vector3f(0, sinf(tail_angle), -cosf(tail_angle)),
    };
    // spin seen from above: +1 counter-clockwise, -1 clockwise
    const float front_right_spin = front_spin == 0 ? 1 : -1;
    const float spin[NUM_ROTORS] { front_right_spin, -front_right_spin, -1 };

    const float rear_d = MAX(rear_diameter.get(), 0.05f);
    const float rear_kt = rear_ct * SSL_AIR_DENSITY * powf(rear_d, 4) / float(sq(M_2PI));
    const float rear_kq = rear_cp * SSL_AIR_DENSITY * powf(rear_d, 5) / float(M_2PI * M_2PI * M_2PI);
    const float kt[NUM_ROTORS] { X6_KT, X6_KT, rear_kt };
    const float kq[NUM_ROTORS] { X6_KQ, X6_KQ, rear_kq };
    const float rotor_j[NUM_ROTORS] { X6_ROTOR_J, X6_ROTOR_J, REAR_ROTOR_J };
    const float disc_area[NUM_ROTORS] {
        float(M_PI) * sq(X6_PROP_RADIUS),
        float(M_PI) * sq(X6_PROP_RADIUS),
        float(M_PI) * sq(0.5f * rear_d),
    };

    Vector3f force;
    Vector3f moment;
    Vector3f rotor_momentum;
    float shaft_power = 0;

    for (uint8_t i = 0; i < NUM_ROTORS; i++) {
        const float w2 = sq(omega[i]);
        float thrust = kt[i] * w2 * density_ratio;

        // air velocity at the rotor: along the axis it changes the
        // thrust (momentum theory, linearised about hover; this is what
        // damps roll and pitch), edgewise it adds momentum drag
        const Vector3f v_rotor = velocity_air_bf + (gyro % pos[i]);
        const float v_axial = v_rotor * axis[i];
        const Vector3f v_edge = v_rotor - axis[i] * v_axial;
        if (thrust > 0) {
            const float v_induced = sqrtf(thrust / (2 * air_density * disc_area[i]));
            thrust *= constrain_float(1 - v_axial / (4 * MAX(v_induced, 1.0f)), 0.5f, 1.5f);
        }
        const Vector3f rotor_force = axis[i] * thrust -
            v_edge * (MOMENTUM_DRAG_COEF * sqrtf(MAX(thrust, 0.0f) * air_density * disc_area[i]));

        force += rotor_force;
        moment += pos[i] % rotor_force;

        // the motor turns the prop against its drag and accelerates
        // it; the body takes the opposite torque
        const float aero_torque = kq[i] * w2 * density_ratio;
        moment -= axis[i] * (spin[i] * (aero_torque + rotor_j[i] * omega_dot[i]));
        rotor_momentum += axis[i] * (spin[i] * rotor_j[i] * omega[i]);

        shaft_power += aero_torque * omega[i];
        rpm[i == REAR ? 3 : i] = omega[i] * float(30 / M_PI);
    }

    // body drag
    const Vector3f &v = velocity_air_bf;
    force -= Vector3f(BODY_CDA.x * v.x * fabsf(v.x),
                      BODY_CDA.y * v.y * fabsf(v.y),
                      BODY_CDA.z * v.z * fabsf(v.z)) * (0.5f * air_density);

    body_accel = force / total_mass;

    // Euler's equations, with the rotors' angular momentum
    const Vector3f body_momentum(inertia.x * gyro.x, inertia.y * gyro.y, inertia.z * gyro.z);
    const Vector3f net_moment = moment - (gyro % (body_momentum + rotor_momentum));
    rot_accel = Vector3f(net_moment.x / inertia.x,
                         net_moment.y / inertia.y,
                         net_moment.z / inertia.z);

    current_amp = shaft_power / PROPULSION_EFFICIENCY / voltage;
}

/*
  update the simulation by one time step
 */
void Y3Heavy::update(const struct sitl_input &input)
{
    update_mass_properties();
    mass = total_mass;

    update_wind(input);

    motor_mask |= (1U << 0) | (1U << 1) | (1U << 3);

    Vector3f rot_accel;
    float current_amp = 0;
    calculate_forces(input, rot_accel, accel_body, current_amp);

    add_shove_forces(rot_accel, accel_body);
    add_twist_forces(rot_accel);
    add_external_forces(accel_body);

    // simulated clamp holding vehicle down
    if (clamp.clamped(*this, input)) {
        rot_accel.zero();
        accel_body.zero();
    }

    battery.maybe_reset(sitl->batt_voltage, sitl->batt_capacity_ah, sitl->batt_resistance);
    battery_voltage = battery.get_voltage();
    battery_current = current_amp;
    battery_temperature_degC = battery.get_temperature_degC();
    battery.consume_energy(current_amp, AP_HAL::micros64());

    update_dynamics(rot_accel);
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

#endif  // AP_SIM_Y3HEAVY_ENABLED
