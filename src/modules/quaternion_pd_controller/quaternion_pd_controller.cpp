/****************************************************************************
 *
 *   Quaternion PD controller module (quaternion_pd_controller)
 *
 ****************************************************************************/

#include "quaternion_pd_controller.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>

using math::constrain;
using namespace time_literals;

quaternion_pd_controller::quaternion_pd_controller() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

quaternion_pd_controller::~quaternion_pd_controller()
{
}

bool quaternion_pd_controller::init()
{
	// Run whenever vehicle_angular_velocity updates
	if (!_angular_velocity_sub.registerCallback()) {
		PX4_ERR("angular velocity callback registration failed");
		return false;
	}

	return true;
}

void quaternion_pd_controller::parameters_updated() {

	_Kv_r = _param_bsa_roll_kv.get();
	_Kv_p = _param_bsa_pitch_kv.get();
	_Kv_y = _param_bsa_yaw_kv.get();

	_Ka_r = _param_bsa_roll_ka.get();
	_Ka_p = _param_bsa_pitch_ka.get();
	_Ka_y = _param_bsa_yaw_ka.get();

	_Ixx = _param_bsa_ixx.get();
	_Iyy = _param_bsa_iyy.get();
	_Izz = _param_bsa_izz.get();
}

void quaternion_pd_controller::resetYawInit(uint64_t now)
{
    _yaw_initialized = false;
    _yaw_sp = 0.f;

    // reset timing gates
    _first_run = now;   // if you use this for "t_from_start"

    // if you also want to restart dt cleanly:
    // _last_run = now;

    // any other per-arm/disarm init you need
}

void quaternion_pd_controller::Run()
{
    if (should_exit()) {
		_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

    vehicle_angular_velocity_s angular_velocity{};

	// Run only when we have a new gyro sample
	if (!_angular_velocity_sub.update(&angular_velocity)) {
		return;
	}

	const uint64_t now = angular_velocity.timestamp_sample;

	// Compute dt from gyro sample timestamps (in seconds)
	if (_last_run == 0) {
		_last_run = now;
		_first_run = now;
		return; // wait one cycle to get a valid dt
	}

	float dt = (now - _last_run) * 1e-6f;
	_last_run = now;

	// Guard against unreasonable dt values
	dt = constrain(dt, 0.0005f, 0.02f);

	if (_vehicle_status_sub.update(&_vehicle_status)) {
		const bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

		if (armed != _armed_prev) {
			resetYawInit(now);
			_armed_prev = armed;
		}
	}

	// Current body rates and acceleration
	_rates_body(0) = angular_velocity.xyz[0];
	_rates_body(1) = angular_velocity.xyz[1];
	_rates_body(2) = angular_velocity.xyz[2];

    // Local position & velocity (NED)
	if (_local_position_sub.updated()) {
		_local_position_sub.copy(&_local_position);

		_vel_est_ned(0) = _local_position.vx; // [m/s] North
		_vel_est_ned(1) = _local_position.vy; // [m/s] East
		_vel_est_ned(2) = _local_position.vz; // [m/s] Down

		_vel_est_body = _R_att.transpose() * _vel_est_ned; // rotate NED velocity to body frame
	}

	// Attitude (quaternion)
	if (_attitude_sub.updated()) {
		_attitude_sub.copy(&_attitude);

		// Get the Quaternion from the attitude message
		_q_att = matrix::Quatf(_attitude.q[0],
			       _attitude.q[1],
			       _attitude.q[2],
			       _attitude.q[3]);
		// Get the DCM from the Quaternion
		_R_att = matrix::Dcmf(_q_att);
	}

	if (_attitude_setpoint_sub.updated()) {
		_attitude_setpoint_sub.copy(&_attitude_sp);

		const matrix::Quatf q_rp(_attitude_sp.q_d[0],
								_attitude_sp.q_d[1],
								_attitude_sp.q_d[2],
								_attitude_sp.q_d[3]);

		matrix::Eulerf rpy(q_rp);

		if(!_yaw_initialized) {
			float t_from_start = (now - _first_run)*1e-6;
			float yaw_abs = fabs(rpy(2));
			if(yaw_abs > 0.f && t_from_start > 3.f) {
				_yaw_initialized = true;
			}

			_q_att_sp = matrix::Quatf(matrix::Eulerf(rpy(0), rpy(1), rpy(2)));
			_q_att_sp.normalize();
			_yaw_sp = rpy(2);
		} else {
			_q_att_sp = matrix::Quatf(matrix::Eulerf(rpy(0), rpy(1), _yaw_sp));
			_q_att_sp.normalize();
		}

		if(_constrain_yaw) {
			_q_att_sp = matrix::Quatf(matrix::Eulerf(rpy(0), rpy(1), rpy(2)));
			_q_att_sp.normalize();
		}

		_thrust_sp(2) = _attitude_sp.thrust_body[2];

		// DEBUG
		// matrix::Eulerf rpy2(_q_att_sp);

		// PX4_INFO("att_sp RPY [rad]: roll=%.3f pitch=%.3f yaw=%.3f",
		// 			(double)rpy2(0),
		// 			(double)rpy2(1),
		// 			(double)rpy2(2));
	}

	// Manual control input
	if (_manual_control_sub.updated()) {
		_manual_control_sub.copy(&_manual_control);
		updateYawRateSp();
		integrateYawSp(dt);
	}

    calcRollTorque();
    calcPitchTorque();
    calcYawTorque();

	_torque_sp *= _torque_scale;

    // Publish torque setpoint
	vehicle_torque_setpoint_s torque_msg{};
	torque_msg.timestamp_sample = angular_velocity.timestamp_sample;
	torque_msg.timestamp = hrt_absolute_time();

	torque_msg.xyz[0] = PX4_ISFINITE(_torque_sp(0)) ? _torque_sp(0) : 0.f;
	torque_msg.xyz[1] = PX4_ISFINITE(_torque_sp(1)) ? _torque_sp(1) : 0.f;
	torque_msg.xyz[2] = PX4_ISFINITE(_torque_sp(2)) ? _torque_sp(2) : 0.f;

	_torque_sp_pub.publish(torque_msg);

	// Publish thrust setpoint
	vehicle_thrust_setpoint_s thrust_msg{};
	thrust_msg.timestamp_sample = angular_velocity.timestamp_sample;
	thrust_msg.timestamp = hrt_absolute_time();
    
    thrust_msg.xyz[0] = PX4_ISFINITE(_thrust_sp(0)) ? _thrust_sp(0) : 0.f;
	thrust_msg.xyz[1] = PX4_ISFINITE(_thrust_sp(1)) ? _thrust_sp(1) : 0.f;
	thrust_msg.xyz[2] = PX4_ISFINITE(_thrust_sp(2)) ? _thrust_sp(2) : 0.f;

	_thrust_sp_pub.publish(thrust_msg);
}

void quaternion_pd_controller::calcRollTorque() {
	// Roll channel P and D gains (k1, k2)
	const float k1 = _Kv_r;
	const float k2 = _Ka_r;

	const float alpha_d = 0.f;   
	const float V_b2    = _vel_est_body(1);    // body-velocity y component (V_b2)

	// Current attitude quaternion q = [q0, qv1, qv2, qv3]
	const float q0  = _q_att(0);
	const float qv1 = _q_att(1);
	const float qv2 = _q_att(2);
	const float qv3 = _q_att(3);

	// Commanded quaternion q_c = [q_c0, q_cv1, q_cv2, q_cv3]
	const float qc0  = _q_att_sp(0);
	const float qcv1 = _q_att_sp(1);
	const float qcv2 = _q_att_sp(2);
	const float qcv3 = _q_att_sp(3);

	// Body rates omega = [omega1, omega2, omega3]
	const float omega1 = _rates_body(0);
	const float omega2 = _rates_body(1);
	const float omega3 = _rates_body(2);

	// Calculate intermediate values
	t2 = qc0*qc0;
	t3 = qcv1*qcv1;
	t4 = qcv2*qcv2;
	t5 = qcv3*qcv3;
	t6 = t2+t3+t4+t5;
	t7 = 1.0/t6;
	_torque_sp(0) = k1.*(q0*qcv1*t7-qc0*qv1*t7-qcv2*qv3*t7+qcv3*qv2*t7)-k2*omega1;
}

void quaternion_pd_controller::calcPitchTorque() {
	// Pitch channel P and D gains (k1, k2)
	const float k1 = _Kv_p;
	const float k2 = _Ka_p;

	const float alpha_d = 0.f; 
	const float V_b1    = _vel_est_body(0);   // body-velocity x component (V_b1)

	// Current attitude quaternion q = [q0, qv1, qv2, qv3]
	const float q0  = _q_att(0);
	const float qv1 = _q_att(1);
	const float qv2 = _q_att(2);
	const float qv3 = _q_att(3);

	// Commanded quaternion q_c = [q_c0, q_cv1, q_cv2, q_cv3]
	const float qc0  = _q_att_sp(0);
	const float qcv1 = _q_att_sp(1);
	const float qcv2 = _q_att_sp(2);
	const float qcv3 = _q_att_sp(3);

	// Body rates omega = [omega1, omega2, omega3]
	const float omega1 = _rates_body(0);
	const float omega2 = _rates_body(1);
	const float omega3 = _rates_body(2);

	const float t2 = qc0*qc0;
	const float t3 = qcv1*qcv1;
	const float t4 = qcv2*qcv2;
	const float t5 = qcv3*qcv3;

	const float t6 = t2+t3+t4+t5;
	const float t7 = 1.0/t6;

	_torque_sp(0) = k1.*(q0*qcv2*t7-qc0*qv2*t7+qcv1*qv3*t7-qcv3*qv1*t7)-k2*omega2;
}

void quaternion_pd_controller::calcYawTorque() {
	// Yaw channel P and D gains (k1, k2)
	const float k1 = _Kv_y;
	const float k2 = _Ka_y;

	// Current attitude quaternion q = [q0, qv1, qv2, qv3]
	const float q0  = _q_att(0);
	const float qv1 = _q_att(1);
	const float qv2 = _q_att(2);
	const float qv3 = _q_att(3);

	// Commanded quaternion q_c = [q_c0, q_cv1, q_cv2, q_cv3]
	const float qc0  = _q_att_sp(0);
	const float qcv1 = _q_att_sp(1);
	const float qcv2 = _q_att_sp(2);
	const float qcv3 = _q_att_sp(3);

	// Body rates omega = [omega1, omega2, omega3]
	const float omega1 = _rates_body(0);
	const float omega2 = _rates_body(1);
	const float omega3 = _rates_body(2);

	// Calculate intermediate values
	t2 = q_c0*q_c0;
	t3 = q_cv1*q_cv1;
	t4 = q_cv2*q_cv2;
	t5 = q_cv3*q_cv3;
	t6 = t2+t3+t4+t5;
	t7 = 1.0/t6;
	_torque_sp(2) = k1.*(q_0*q_cv3*t7-q_c0*q_v3*t7-q_cv1*q_v2*t7+q_cv2*q_v1*t7)-k2*omega3;
}

void quaternion_pd_controller::updateYawRateSp() {
	_yaw_rate_sp = _yaw_rate_scale * _manual_control.yaw;
}

void quaternion_pd_controller::integrateYawSp(float& dt) {
    _yaw_sp += _yaw_rate_sp * dt;
}

/** ModuleBase interface **/

int quaternion_pd_controller::task_spawn(int argc, char *argv[])
{
	quaternion_pd_controller *instance = new quaternion_pd_controller();

	if (!instance) {
		PX4_ERR("backstepping attitude failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init()) {
		return PX4_OK;
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int quaternion_pd_controller::custom_command(int argc, char *argv[])
{
	// No custom commands yet
	return print_usage("unknown command");
}

int quaternion_pd_controller::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Vertical vehicle quaternion PD controller.

- Subscribes: manual_control_input, vehicle local position, vehicle_attitude, vehicle_angular_velocity, vehicle_control_mode
- Publishes: vehicle_torque_setpoint, vehicle_thrust_setpoint
- Uses fixed gains for controller.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("quaternion_pd_controller", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int quaternion_pd_controller_main(int argc, char *argv[])
{
	return quaternion_pd_controller::main(argc, argv);
}
