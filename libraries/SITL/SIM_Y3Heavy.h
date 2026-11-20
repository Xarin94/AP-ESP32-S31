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
  Y3Heavy: tricopter whose two front rotors (Hobbywing X6 Plus G2,
  MFP 24x8) carry most of the weight and the whole payload, with a
  small tilting rear rotor (350KV, 13x10 three blade) for pitch and
  yaw. For the split tricopter mixer, MOT_TRI_MIX 1.

  Each rotor has its own thrust and torque curve and its own speed
  time constant, the tail servo is rate limited, and the mass
  properties come from the component masses, so SIM_Y3_PAYLOAD can
  change in flight.
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_Y3HEAVY_ENABLED

#include "SIM_Aircraft.h"
#include <AP_Param/AP_Param.h>

namespace SITL {

class Y3Heavy : public Aircraft {
public:
    Y3Heavy(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW Y3Heavy(frame_str);
    }

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Float payload_kg;
    AP_Float rear_share;
    AP_Float front_unit_kg;
    AP_Float rest_kg;
    AP_Float rear_unit_kg;
    AP_Float front_half_span;
    AP_Float rear_arm;
    AP_Float rear_rpm_max;
    AP_Float front_tau;
    AP_Float rear_tau;
    AP_Float servo_time_60;
    AP_Float rear_ct;
    AP_Float rear_cp;
    AP_Float rear_diameter;
    AP_Int8  front_spin;

    enum Rotor : uint8_t {
        RIGHT = 0,
        LEFT  = 1,
        REAR  = 2,
        NUM_ROTORS
    };

    // rotor speed, rad/s, and its rate of change
    float omega[NUM_ROTORS];
    float omega_dot[NUM_ROTORS];

    // tail tilt about body x, rad: positive gives a side force to the right
    float tail_angle;

    // mass properties, recomputed every step. cg is in the design
    // frame: origin at the middle of the front rotor axis, x forward,
    // y right, z down. inertia holds the moments about the cg (the
    // products are zero: the masses are symmetric in y and all at z=0)
    float total_mass;
    Vector3f cg;
    Vector3f inertia;

    void update_mass_properties();
    float front_rpm_demand(uint16_t pwm, float voltage) const;
    float rear_rpm_demand(uint16_t pwm, float voltage) const;
    void calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel, float &current_amp);
};

} // namespace SITL

#endif  // AP_SIM_Y3HEAVY_ENABLED
