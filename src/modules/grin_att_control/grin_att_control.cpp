/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_att_control_main.cpp
 * AERO4BOAT attitude controller.
 *
 * @author Mathaus Ferreira		<Mathaus.silva@engenharia.ufjf.br>
 */

#include "grin_att_control.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

float_t Fx= 0.0f;
float_t Fy= 0.0f;
float_t TN= 0.0f;

GrinAttitudeControl::GrinAttitudeControl(bool vtol) :
    ModuleParams(nullptr),
    WorkItem(MODULE_NAME, px4::wq_configurations::attitude_ctrl),
    _vehicle_attitude_setpoint_pub(ORB_ID(vehicle_attitude_setpoint)),
    _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
    _vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING; //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    /* initialize quaternions in messages to be valid */
    _v_att.q[0] = 1.f;
    _v_att_sp.q_d[0] = 1.f;

    parameters_updated();
}

GrinAttitudeControl::~GrinAttitudeControl()
{
    perf_free(_loop_perf);
}

bool
GrinAttitudeControl::init()
{
    if (!_vehicle_attitude_sub.registerCallback()) {
        PX4_ERR("vehicle_attitude callback registration failed! Mathaus ");
        return false;
    }

    return true;
}

void
GrinAttitudeControl::parameters_updated()
{
    // Store some of the parameters in a more convenient way & precompute often-used values
    _attitude_control.setProportionalGain(Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get()),_param_mc_yaw_weight.get());

    // angular rate limits
    using math::radians;


    _man_tilt_max = math::radians(_param_mpc_man_tilt_max.get());
}

float
GrinAttitudeControl::throttle_curve(float throttle_stick_input)
{
    float throttle_min = _vehicle_land_detected.landed ? 0.0f : _param_mpc_manthr_min.get();

    // throttle_stick_input is in range [0, 1]
    switch (_param_mpc_thr_curve.get()) {
    case 1: // no rescaling to hover throttle
        return throttle_min + throttle_stick_input * (_param_mpc_thr_max.get() - throttle_min);

    default: // 0 or other: rescale to hover throttle at 0.5 stick
        if (throttle_stick_input < 0.5f) {
            return (_param_mpc_thr_hover.get() - throttle_min) / 0.5f * throttle_stick_input +
                   throttle_min;

        } else {
            return (_param_mpc_thr_max.get() - _param_mpc_thr_hover.get()) / 0.5f * (throttle_stick_input - 1.0f) +
                   _param_mpc_thr_max.get();
        }
    }
}
void
GrinAttitudeControl::generate_attitude_setpoint(float dt, bool reset_yaw_sp)
{
    vehicle_attitude_setpoint_s attitude_setpoint{};
    const float yaw = Eulerf(Quatf(_v_att.q)).psi();

    /* reset yaw setpoint to current position if needed */
    if (reset_yaw_sp) {
        _man_yaw_sp = yaw;

    } else if (_manual_control_sp.z > 0.05f || _param_mc_airmode.get() == (int32_t)Mixer::Airmode::roll_pitch_yaw) {

        const float yaw_rate = math::radians(_param_mpc_man_y_max.get());
        attitude_setpoint.yaw_sp_move_rate = _manual_control_sp.r * yaw_rate;
        _man_yaw_sp = wrap_pi(_man_yaw_sp + attitude_setpoint.yaw_sp_move_rate * dt);
    }

    /*
     * Input mapping for roll & pitch setpoints
     * ----------------------------------------
     * We control the following 2 angles:
     * - tilt angle, given by sqrt(x*x + y*y)
     * - the direction of the maximum tilt in the XY-plane, which also defines the direction of the motion
     *
     * This allows a simple limitation of the tilt angle, the vehicle flies towards the direction that the stick
     * points to, and changes of the stick input are linear.
     */
    const float x = _manual_control_sp.x;// * _man_tilt_max;
    const float y = _manual_control_sp.y;// * _man_tilt_max;

    // we want to fly towards the direction of (x, y), so we use a perpendicular axis angle vector in the XY-plane
    Vector2f v = Vector2f(y, -x);
    float v_norm = v.norm(); // the norm of v defines the tilt angle

    if (v_norm > 1.0f) { // limit to the configured maximum tilt angle
        v *= 1.f / v_norm;
    }

    Quatf q_sp_rpy = AxisAnglef(v(0), v(1), 0.f);
    Eulerf euler_sp = q_sp_rpy;
    attitude_setpoint.roll_body = euler_sp(0);  //<< mathaus : zerar?
    attitude_setpoint.pitch_body = euler_sp(1); //<< mathaus : zerar?
    // The axis angle can change the yaw as well (noticeable at higher tilt angles).
    // This is the formula by how much the yaw changes:
    //   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
    //   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
    attitude_setpoint.yaw_body = _man_yaw_sp + euler_sp(2);

    /* modify roll/pitch only if we're a VTOL */
    if (_vehicle_status.is_vtol) {
        // Construct attitude setpoint rotation matrix. Modify the setpoints for roll
        // and pitch such that they reflect the user's intention even if a large yaw error
        // (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
        // from the pure euler angle setpoints will lead to unexpected attitude behaviour from
        // the user's view as the euler angle sequence uses the  yaw setpoint and not the current
        // heading of the vehicle.
        // However there's also a coupling effect that causes oscillations for fast roll/pitch changes
        // at higher tilt angles, so we want to avoid using this on multicopters.
        // The effect of that can be seen with:
        // - roll/pitch into one direction, keep it fixed (at high angle)
        // - apply a fast yaw rotation
        // - look at the roll and pitch angles: they should stay pretty much the same as when not yawing

        // calculate our current yaw error
        float yaw_error = wrap_pi(attitude_setpoint.yaw_body - yaw);

        // compute the vector obtained by rotating a z unit vector by the rotation
        // given by the roll and pitch commands of the user
        Vector3f zB = {0.0f, 0.0f, 1.0f};
        Dcmf R_sp_roll_pitch = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, 0.0f);
        Vector3f z_roll_pitch_sp = R_sp_roll_pitch * zB;

        // transform the vector into a new frame which is rotated around the z axis
        // by the current yaw error. this vector defines the desired tilt when we look
        // into the direction of the desired heading
        Dcmf R_yaw_correction = Eulerf(0.0f, 0.0f, -yaw_error);
        z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;

        // use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
        // R_tilt is computed from_euler; only true if cos(roll) not equal zero
        // -> valid if roll is not +-pi/2;
        attitude_setpoint.roll_body = -asinf(z_roll_pitch_sp(1));
        attitude_setpoint.pitch_body = atan2f(z_roll_pitch_sp(0), z_roll_pitch_sp(2));
    }

    /* copy quaternion setpoint to attitude setpoint topic */
    Quatf q_sp = Eulerf(attitude_setpoint.roll_body, attitude_setpoint.pitch_body, attitude_setpoint.yaw_body);
    q_sp.copyTo(attitude_setpoint.q_d);

    attitude_setpoint.thrust_body[2] = -throttle_curve(_manual_control_sp.z);
    attitude_setpoint.timestamp = hrt_absolute_time();

    _vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector
 */
void
GrinAttitudeControl::control_attitude()
{
    _v_att_sp_sub.update(&_v_att_sp);
    _rates_sp = _attitude_control.update(Quatf(_v_att.q), Quatf(_v_att_sp.q_d), _v_att_sp.yaw_sp_move_rate);
}

void
GrinAttitudeControl::publish_rates_setpoint()
{
    vehicle_rates_setpoint_s v_rates_sp{};
    // Mathaus
    Fx = _rates_sp(1);
    Fy = _rates_sp(0);
    TN = _rates_sp(2);

    v_rates_sp.roll = _rates_sp(0); // <<< provavel zerar aqui :mathaus
    v_rates_sp.pitch = _rates_sp(1); // <<< provavel zerar aqui :mathaus
    v_rates_sp.yaw = _rates_sp(2);
    v_rates_sp.thrust_body[0] = _v_att_sp.thrust_body[0];
    v_rates_sp.thrust_body[1] = _v_att_sp.thrust_body[1];
    v_rates_sp.thrust_body[2] = _v_att_sp.thrust_body[2];
    v_rates_sp.timestamp = hrt_absolute_time();

    _v_rates_sp_pub.publish(v_rates_sp);
}

void
GrinAttitudeControl::Run()
{
    if (should_exit()) {
        _vehicle_attitude_sub.unregisterCallback();
        exit_and_cleanup();
        return;
    }

    perf_begin(_loop_perf);

    // Check if parameters have changed
    if (_params_sub.updated()) {
        // clear update
        parameter_update_s param_update;
        _params_sub.copy(&param_update);

        updateParams();
        parameters_updated();
    }

    // run controller on attitude updates
    const uint8_t prev_quat_reset_counter = _v_att.quat_reset_counter;

    if (_vehicle_attitude_sub.update(&_v_att)) {

        // Check for a heading reset
        if (prev_quat_reset_counter != _v_att.quat_reset_counter) {
            // we only extract the heading change from the delta quaternion
            _man_yaw_sp += Eulerf(Quatf(_v_att.delta_q_reset)).psi();
        }

        const hrt_abstime now = hrt_absolute_time();

        // Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
        const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);
        _last_run = now;

        /* check for updates in other topics */
        _manual_control_sp_sub.update(&_manual_control_sp);
        _v_control_mode_sub.update(&_v_control_mode);
        _vehicle_land_detected_sub.update(&_vehicle_land_detected);
        _vehicle_status_sub.update(&_vehicle_status);

        /* Check if we are in rattitude mode and the pilot is above the threshold on pitch
        * or roll (yaw can rotate 360 in normal att control). If both are true don't
        * even bother running the attitude controllers */
        if (_v_control_mode.flag_control_rattitude_enabled) {
            _v_control_mode.flag_control_attitude_enabled =
                fabsf(_manual_control_sp.y) <= _param_mc_ratt_th.get() &&
                fabsf(_manual_control_sp.x) <= _param_mc_ratt_th.get();
        }

        bool attitude_setpoint_generated = false;

        const bool is_hovering = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.in_transition_mode;

        // vehicle is a tailsitter in transition mode
        const bool is_tailsitter_transition = _vehicle_status.in_transition_mode && _is_tailsitter;

        bool run_att_ctrl = _v_control_mode.flag_control_attitude_enabled && (is_hovering || is_tailsitter_transition);

        if (run_att_ctrl) {
            // Generate the attitude setpoint from stick inputs if we are in Manual/Stabilized mode
            if (_v_control_mode.flag_control_manual_enabled &&
                !_v_control_mode.flag_control_altitude_enabled &&
                !_v_control_mode.flag_control_velocity_enabled &&
                !_v_control_mode.flag_control_position_enabled) {

                generate_attitude_setpoint(dt, _reset_yaw_sp);
                attitude_setpoint_generated = true;
            }

            control_attitude();

            if (_v_control_mode.flag_control_yawrate_override_enabled) {
                /* Yaw rate override enabled, overwrite the yaw setpoint */
                _v_rates_sp_sub.update(&_v_rates_sp);
                const auto yawrate_reference = _v_rates_sp.yaw;
                _rates_sp(2) = yawrate_reference;
            }

            publish_rates_setpoint();
        }

        // reset yaw setpoint during transitions, tailsitter.cpp generates
        // attitude setpoint for the transition
        _reset_yaw_sp = (!attitude_setpoint_generated && !_v_control_mode.flag_control_rattitude_enabled) ||
                _vehicle_land_detected.landed ||
                (_vehicle_status.is_vtol && _vehicle_status.in_transition_mode);

    }

    perf_end(_loop_perf);
}

int GrinAttitudeControl::task_spawn(int argc, char *argv[])
{
    bool vtol = false;

    if (argc > 1) {
        if (strcmp(argv[1], "vtol") == 0) {
            vtol = true;
        }
    }

    GrinAttitudeControl *instance = new GrinAttitudeControl(vtol);

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init()) {
            return PX4_OK;
        }

    } else {
        PX4_ERR("alloc failed");
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

int GrinAttitudeControl::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int GrinAttitudeControl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
This implements the multicopter attitude controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.

The controller has a P loop for angular error

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("mc_att_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int mc_att_control_main(int argc, char *argv[])
{
    return GrinAttitudeControl::main(argc, argv);
}


//#include "grin_att_control.hpp"


//#define ACTUATOR_PUBLISH_PERIOD_MS 4


//**
// * GRIN attitude control app start / stop handling function
// *
// * @ingroup apps
// */
//extern "C" __EXPORT int grin_att_control_main(int argc, char *argv[]);


//GRINAttitudeControl::GRINAttitudeControl():
//	ModuleParams(nullptr),
//	/* performance counters */
//	_loop_perf(perf_alloc(PC_ELAPSED, "gnda_dt"))
//{
//}

//GRINAttitudeControl::~GRINAttitudeControl()
//{
//	perf_free(_loop_perf);
//}

//void GRINAttitudeControl::parameters_update(bool force)
//{
//	// check for parameter updates
//	if (_parameter_update_sub.updated() || force) {
//		// clear update
//		parameter_update_s pupdate;
//		_parameter_update_sub.copy(&pupdate);

//		// update parameters from storage
//		updateParams();
//	}
//}

//void GRINAttitudeControl::vehicle_control_mode_poll()
//{
//	bool updated = false;
//	orb_check(_vcontrol_mode_sub, &updated);

//	if (updated) {
//		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
//	}
//}

//void GRINAttitudeControl::manual_control_setpoint_poll()
//{
//	bool updated = false;
//	orb_check(_manual_control_sub, &updated);

//	if (updated) {
//		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual);
//	}
//}

//void GRINAttitudeControl::vehicle_attitude_setpoint_poll()
//{
//	bool updated = false;
//	orb_check(_vehicle_attitude_sp_sub, &updated);

//	if (updated) {
//		orb_copy(ORB_ID(vehicle_attitude_setpoint), _vehicle_attitude_sp_sub, &_vehicle_attitude_sp);
//	}
//}


//void GRINAttitudeControl::constrain_actuator_commands(float roll_u, float pitch_u, float yaw_u, float thrust_u)
//{
//	if (PX4_ISFINITE(roll_u)) {
//		roll_u = math::constrain(roll_u, -1.0f, 1.0f);
//		_actuators.control[actuator_controls_s::INDEX_ROLL] = roll_u;

//	} else {
//		_actuators.control[actuator_controls_s::INDEX_ROLL] = 0.0f;

//		if (loop_counter % 10 == 0) {
//			PX4_INFO("roll_u %.4f", (double)roll_u);
//		}
//	}

//	if (PX4_ISFINITE(pitch_u)) {
//		pitch_u = math::constrain(pitch_u, -1.0f, 1.0f);
//		_actuators.control[actuator_controls_s::INDEX_PITCH] = pitch_u;

//	} else {
//		_actuators.control[actuator_controls_s::INDEX_PITCH] = 0.0f;

//		if (loop_counter % 10 == 0) {
//			PX4_INFO("pitch_u %.4f", (
//double)pitch_u);
//		}
//	}

//	if (PX4_ISFINITE(yaw_u)) {
//		yaw_u = math::constrain(yaw_u, -1.0f, 1.0f);
//		_actuators.control[actuator_controls_s::INDEX_YAW] = yaw_u;

//	} else {
//		_actuators.control[actuator_controls_s::INDEX_YAW] = 0.0f;

//		if (loop_counter % 10 == 0) {
//			PX4_INFO("yaw_u %.4f", (double)yaw_u);
//		}
//	}

//	if (PX4_ISFINITE(thrust_u)) {
//		thrust_u = math::constrain(thrust_u, -1.0f, 1.0f);
//		_actuators.control[actuator_controls_s::INDEX_THROTTLE] = thrust_u;

//	} else {
//		_actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;

//		if (loop_counter % 10 == 0) {
//			PX4_INFO("thrust_u %.4f", (double)thrust_u);
//		}
//	}
//}


//void GRINAttitudeControl::control_attitude_geo(const vehicle_attitude_s &att, const vehicle_attitude_setpoint_s &att_sp)
//{
//	/** Geometric Controller
//	 *
//	 * based on
//	 * D. Mellinger, V. Kumar, "Minimum Snap Trajectory Generation and Control for Quadrotors", IEEE ICRA 2011, pp. 2520-2525.
//	 * D. A. Duecker, A. Hackbarth, T. Johannink, E. Kreuzer, and E. Solowjow, “Micro Underwater Vehicle Hydrobatics: A SubmergedFuruta Pendulum,” IEEE ICRA 2018, pp. 7498–7503.
//	 */


//	Eulerf euler_angles(matrix::Quatf(att.q));

//	float roll_u;
//	float pitch_u;
//	float yaw_u;
//	float thrust_u;

//	float roll_body = _vehicle_attitude_sp.roll_body;
//	float pitch_body = _vehicle_attitude_sp.pitch_body;
//	float yaw_body = _vehicle_attitude_sp.yaw_body;

//	/* get attitude setpoint rotational matrix */
//	Dcmf _rot_des = Eulerf(roll_body, pitch_body, yaw_body);

//	/* get current rotation matrix from control state quaternions */
//	Quatf q_att(att.q[0], att.q[1], att.q[2], att.q[3]);
//	Matrix3f _rot_att = q_att.to_dcm();

//	Vector3f e_R_vec;
//	Vector3f torques;
//	Vector3f omega;

//	/* Compute matrix: attitude error */
//	Matrix3f e_R = (_rot_des.transpose() * _rot_att - _rot_att.transpose() * _rot_des) * 0.5;

//	/* vee-map the error to get a vector instead of matrix e_R */
//	e_R_vec(0) = e_R(2, 1);  /**< Roll  */
//	e_R_vec(1) = e_R(0, 2);  /**< Pitch */
//	e_R_vec(2) = e_R(1, 0);  /**< Yaw   */

//	omega(0) = _angular_velocity.xyz[0] - 0.0f;
//	omega(1) = _angular_velocity.xyz[1] - 0.0f;
//	omega(2) = _angular_velocity.xyz[2] - 0.0f;

//	/**< P-Control */
//	torques(0) = - e_R_vec(0) * _param_roll_p.get();	/**< Roll  */
//	torques(1) = - e_R_vec(1) * _param_pitch_p.get();	/**< Pitch */
//	torques(2) = - e_R_vec(2) * _param_yaw_p.get();		/**< Yaw   */

//	/**< PD-Control */
//	torques(0) = torques(0) - omega(0) * _param_roll_d.get();  /**< Roll  */
//	torques(1) = torques(1) - omega(1) * _param_pitch_d.get(); /**< Pitch */
//	torques(2) = torques(2) - omega(2) * _param_yaw_d.get();   /**< Yaw   */

//	roll_u = torques(0);
//	pitch_u = torques(1);
//	yaw_u = torques(2);

//	//Quatf q_att(_att.q[0], _att.q[1], _att.q[2], _att.q[3]);
//	//Matrix3f _rot_att = q_att.to_dcm();
//	/** Vector3f current_velocity_boat;

//	current_velocity_boat(0) = _local_pos.vx;
//	current_velocity_boat(1) = _local_pos.vy;
//	current_velocity_boat(2) = _local_pos.vz;

//	current_velocity_boat = q_att.to_dcm() * current_velocity_boat;
//	*/

//	// take thrust as
//	thrust_u = _param_direct_thrust.get();

//	constrain_actuator_commands(roll_u, pitch_u, yaw_u, thrust_u);
//	/* Geometric Controller END*/
//}


//void GRINAttitudeControl::run()
//{
//	_vehicle_attitude_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
//	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
//	_angular_velocity_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));
//	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
//	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));

//	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

//	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));


//	/* rate limit control mode updates to 5Hz */
//	orb_set_interval(_vcontrol_mode_sub, 200);

//	parameters_update(true);

//	/* wakeup source(s) */
//	px4_pollfd_struct_t fds[3];

//	/* Setup of loop */
//	fds[0].fd = _vehicle_attitude_sub;
//	fds[0].events = POLLIN;
//	fds[1].fd = _manual_control_sub;
//	fds[1].events = POLLIN;
//	fds[2].fd = _sensor_combined_sub;
//	fds[2].events = POLLIN;


//	while (!should_exit()) {
//		/* wait for up to 500ms for data */
//		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

//		/* this is undesirable but not much we can do - might want to flag unhappy status */
//		if (pret < 0) {
//			warn("poll error %d, %d", pret, errno);
//			continue;
//		}

//		/* check vehicle control mode for changes to publication state */
//		vehicle_control_mode_poll();
//		vehicle_attitude_setpoint_poll();

//		/* update parameters from storage */
//		parameters_update();

//		/* only run controller if attitude changed */
//		if (fds[0].revents & POLLIN) {
//			static uint64_t last_run = 0;
//			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
//			last_run = hrt_absolute_time();

//			/* guard against too large deltaT's */
//			if (deltaT > 1.0f || fabsf(deltaT) < 0.00001f || !PX4_ISFINITE(deltaT)) {
//				deltaT = 0.01f;
//			}

//			/* load local copies */
//			orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_vehicle_attitude);
//			orb_copy(ORB_ID(vehicle_angular_velocity), _angular_velocity_sub, &_angular_velocity);
//			orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);

//			vehicle_attitude_setpoint_poll();
//			vehicle_control_mode_poll();
//			manual_control_setpoint_poll();


//			/* Run geometric attitude controllers if NOT manual mode*/
//			if (!_vcontrol_mode.flag_control_manual_enabled
//			    && _vcontrol_mode.flag_control_attitude_enabled
//			    && _vcontrol_mode.flag_control_rates_enabled) {

//				int input_mode = _param_input_mode.get();

//				// if (input_mode == 0) // process incoming vehicles setpoint data --> nothing to do
//				if (input_mode == 1) { // process manual data
//					_vehicle_attitude_sp.roll_body = _param_direct_roll.get();
//					_vehicle_attitude_sp.pitch_body = _param_direct_pitch.get();
//					_vehicle_attitude_sp.yaw_body = _param_direct_yaw.get();
//					_vehicle_attitude_sp.thrust_body[0] = _param_direct_thrust.get();
//				}

//				/* Geometric Control*/
//				control_attitude_geo(_vehicle_attitude, _vehicle_attitude_sp);
//			}
//		}

//		loop_counter++;
//		perf_end(_loop_perf);

//		/* Manual Control mode (e.g. gamepad,...) - raw feedthrough no assistance */
//		if (fds[1].revents & POLLIN) {
//			// This should be copied even if not in manual mode. Otherwise, the poll(...) call will keep
//			// returning immediately and this loop will eat up resources.
//			orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual);

//			if (_vcontrol_mode.flag_control_manual_enabled && !_vcontrol_mode.flag_control_rates_enabled) {
//				/* manual/direct control */
//				constrain_actuator_commands(_manual.y, -_manual.x, _manual.r, _manual.z);
//			}

//		}

//		if (fds[2].revents & POLLIN) {

//			orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);

//			_actuators.timestamp = hrt_absolute_time();

//			/* Only publish if any of the proper modes are enabled */
//			if (_vcontrol_mode.flag_control_manual_enabled ||
//			    _vcontrol_mode.flag_control_attitude_enabled) {
//				/* publish the actuator controls */
//				_actuator_controls_pub.publish(_actuators);

//			}
//		}
//	}

//	orb_unsubscribe(_vcontrol_mode_sub);
//	orb_unsubscribe(_manual_control_sub);
//	orb_unsubscribe(_vehicle_attitude_sub);
//	orb_unsubscribe(_local_pos_sub);
//	orb_unsubscribe(_sensor_combined_sub);

//	warnx("exiting.\n");
//}

//int GRINAttitudeControl::task_spawn(int argc, char *argv[])
//{
//	/* start the task */
//	_task_id = px4_task_spawn_cmd("GRIN_att_ctrl",
//				      SCHED_DEFAULT,
//				      SCHED_PRIORITY_ATTITUDE_CONTROL,
//				      1700,  // maybe switch to 1500
//				      (px4_main_t)&GRINAttitudeControl::run_trampoline,
//				      nullptr);

//	if (_task_id < 0) {
//		warn("task start failed");
//		return -errno;
//	}

//	return OK;
//}

//GRINAttitudeControl *GRINAttitudeControl::instantiate(int argc, char *argv[])
//{

//	if (argc > 0) {
//		PX4_WARN("Command 'start' takes no arguments.");
//		return nullptr;
//	}

//	GRINAttitudeControl *instance = new GRINAttitudeControl();

//	if (instance == nullptr) {
//		PX4_ERR("Failed to instantiate GRINAttitudeControl object");
//	}

//	return instance;
//}

//int GRINAttitudeControl::custom_command(int argc, char *argv[])
//{
//	return print_usage("unknown command");
//}


//int GRINAttitudeControl::print_usage(const char *reason)
//{
//	if (reason) {
//		PX4_WARN("%s\n", reason);
//	}

//	PRINT_MODULE_DESCRIPTION(
//		R"DESCR_STR(
//### Description
//Controls the attitude of an unmanned underwater vehicle (GRIN).

//Publishes `actuator_controls_0` messages at a constant 250Hz.

//### Implementation
//Currently, this implementation supports only a few modes:

// * Full manual: Roll, pitch, yaw, and throttle controls are passed directly through to the actuators
// * Auto mission: The GRIN runs missions

//### Examples
//CLI usage example:
//$ GRIN_att_control start
//$ GRIN_att_control status
//$ GRIN_att_control stop

//)DESCR_STR");

//	PRINT_MODULE_USAGE_NAME("GRIN_att_control", "controller");
//	PRINT_MODULE_USAGE_COMMAND("start")
//	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

//	return 0;
//}

//int grin_att_control_main(int argc, char *argv[])
//{
//	return GRINAttitudeControl::main(argc, argv);
//}
