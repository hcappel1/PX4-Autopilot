/****************************************************************************
 * CdusAllocator.cpp
 ****************************************************************************/

#include "CdusAllocator.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

CdusAllocator::CdusAllocator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	init_effectiveness_matrix();

	if (!_torque_sp_sub.registerCallback()) {
		PX4_WARN("Callback registration failed");
	}
	#ifndef ENABLE_LOCKSTEP_SCHEDULER
		// ScheduleDelayed(50_ms);   // backup periodic run
	#endif

	ScheduleNow();
}

// void CdusAllocator::init_effectiveness_matrix()
// {
// 	float x_pos = 0.131; //x offset in meters for motors
// 	float y_pos = 0.219; //y offset in meters for motors

// 	Vector3f pos[NUM_MOTORS];
// 	pos[0] = Vector3f(+x_pos, +y_pos, 0.0);
// 	pos[1] = Vector3f(-x_pos, -y_pos, 0.0);
// 	pos[2] = Vector3f(+x_pos, -y_pos, 0.0);
// 	pos[3] = Vector3f(-x_pos, +y_pos, 0.0);

// 	//Thrust axis
// 	const Vector3f axis(0.f, 0.f, -1.f);

// 	//Thrust coeffs
// 	float Ct = 6.5;
// 	// float Ct = 1.0;

// 	//Moment ratios
// 	const Vector4f Km(0.05, 0.05, -0.05, -0.05);

// 	for(int i=0; i < NUM_MOTORS; ++i) {
// 		Vector3f thrust = Ct * axis;
// 		Vector3f moment = Ct * pos[i].cross(axis) - Ct * Km(i) * axis;
// 		for (size_t j=0; j < 3; j++) {
// 			_B(j,i) = moment(j);
// 			_B(3,i) = thrust(2);
// 		}
// 	}

// 	matrix::geninv(_B,_B_pinv);
// 	normalize_allocation_matrix();

// 	// _B_pinv(0,0) = -0.175623f; _B_pinv(0,1) =  0.293600f; _B_pinv(0,2) =  0.769231f; _B_pinv(0,3) = -1.0f;
// 	// _B_pinv(1,0) =  0.175623f; _B_pinv(1,1) = -0.293600f; _B_pinv(1,2) =  0.769231f; _B_pinv(1,3) = -1.0f;
// 	// _B_pinv(2,0) =  0.175623f; _B_pinv(2,1) =  0.293600f; _B_pinv(2,2) = -0.769231f; _B_pinv(2,3) = -1.0f;
// 	// _B_pinv(3,0) = -0.175623f; _B_pinv(3,1) = -0.293600f; _B_pinv(3,2) = -0.769231f; _B_pinv(3,3) = -1.0f;

// 	// THIS IS THE ONE THAT WORKS THE BEST!
// 	// _B_pinv(0,0) = -0.4377f; _B_pinv(0,1) =  0.7071f; _B_pinv(0,2) =  0.9091f; _B_pinv(0,3) = -1.0f;
// 	// _B_pinv(1,0) =  0.4377f; _B_pinv(1,1) = -0.7071f; _B_pinv(1,2) =  1.0f; _B_pinv(1,3) = -1.0f;
// 	// _B_pinv(2,0) =  0.4377f; _B_pinv(2,1) =  0.7071f; _B_pinv(2,2) = -0.9091f; _B_pinv(2,3) = -1.0f;
// 	// _B_pinv(3,0) = -0.4377f; _B_pinv(3,1) = -0.7071f; _B_pinv(3,2) = -1.0f; _B_pinv(3,3) = -1.0f;


// }

void CdusAllocator::init_effectiveness_matrix_duct()
{
	// Create initial allocation matrix for the ducted vehicle 
	matrix::Matrix<float, 4, 4> allocation_matrix_ducted;
	Matrix4f = allocation_matrix_ducted = {
		{ 0.01037f,  0.00967f, 0.0f, 0.0f}, // Thrust 
		{ 0.0f, 0.0f, -0.00038f,  0.00105f}, // Roll
		{ 0.0f, 0.0f, 0.00072f,  0.00040f}, // Pitch 
		{ 0.00030f,  -0.00030f, 0.0f,  0.0f} // Yaw
	};
}

{
	_hover_thrust = hover_thrust;
}

// void CdusAllocator::normalize_allocation_matrix() {
// 	int num_non_zero_roll_torque = 0;
// 	int num_non_zero_pitch_torque = 0;

// 	for (int i = 0; i < NUM_MOTORS; i++) {

// 		if (fabsf(_B_pinv(i, 0)) > 1e-3f) {
// 			++num_non_zero_roll_torque;
// 		}

// 		if (fabsf(_B_pinv(i, 1)) > 1e-3f) {
// 			++num_non_zero_pitch_torque;
// 		}
// 	}

// 	float roll_norm_scale = 1.f;

// 	if (num_non_zero_roll_torque > 0) {
// 		roll_norm_scale = sqrtf(_B_pinv.col(0).norm_squared() / (num_non_zero_roll_torque / 2.f));
// 	}

// 	float pitch_norm_scale = 1.f;

// 	if (num_non_zero_pitch_torque > 0) {
// 		pitch_norm_scale = sqrtf(_B_pinv.col(1).norm_squared() / (num_non_zero_pitch_torque / 2.f));
// 	}

// 	_control_allocation_scale(0) = fmaxf(roll_norm_scale, pitch_norm_scale);
// 	_control_allocation_scale(1) = _control_allocation_scale(0);

// 	// Scale yaw separately
// 	_control_allocation_scale(2) = _B_pinv.col(2).max();

// 	_control_allocation_scale(3) = 1.f;

// 	int num_non_zero_thrust = 0;
// 	float norm_sum = 0.f;

// 	for (int i = 0; i < NUM_MOTORS; i++) {
// 		float norm = fabsf(_B_pinv(i, 3));
// 		norm_sum += norm;

// 		if (norm > FLT_EPSILON) {
// 			++num_non_zero_thrust;
// 		}
// 	}

// 	if (num_non_zero_thrust > 0) {
// 		_control_allocation_scale(3) = norm_sum / num_non_zero_thrust;

// 	} else {
// 		_control_allocation_scale(3) = _control_allocation_scale(3);
// 	}

// 	if (_control_allocation_scale(0) > FLT_EPSILON) {
// 		_B_pinv.col(0) /= _control_allocation_scale(0);
// 		_B_pinv.col(1) /= _control_allocation_scale(1);
// 	}

// 	if (_control_allocation_scale(2) > FLT_EPSILON) {
// 		_B_pinv.col(2) /= _control_allocation_scale(2);
// 	}

// 	if (_control_allocation_scale(3) > FLT_EPSILON) {
// 		_B_pinv.col(3) /= _control_allocation_scale(3);
// 	}

// 	//Set all the thrust values to -1.0
// 	_B_pinv.col(3) = -1.0;

// 	// Set all the small elements to 0 to avoid issues
// 	// in the control allocation algorithms
// 	for (int i = 0; i < NUM_MOTORS; i++) {
// 		for (int j = 0; j < 4; j++) {
// 			if (fabsf(_B_pinv(i, j)) < 1e-3f) {
// 				_B_pinv(i, j) = 0.f;
// 			}
// 		}
// 	}
// }

// void CdusAllocator::Run()
// {
// 	if (should_exit()) {
// 		_torque_sp_sub.unregisterCallback();
// 		exit_and_cleanup();
// 		return;
// 	}

// 	vehicle_status_s vehicle_status{};
// 	if (_vehicle_status_sub.update(&vehicle_status)) {
// 		_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
// 	}

// 	vehicle_control_mode_s vcm{};
// 	if (_vehicle_control_mode_sub.update(&vcm)) {
// 		_rate_control_enabled = vcm.flag_control_rates_enabled;
// 	}

// 	// Disarmed or not in rate control => publish zero motors
// 	if (!_armed || !_rate_control_enabled) {
// 		actuator_motors_s out{};
// 		out.timestamp = hrt_absolute_time();
// 		for (int i = 0; i < NUM_MOTORS; i++) { out.control[i] = 0.f; }
// 		_actuator_motors_pub.publish(out);
// 		return;
// 	}

// 	vehicle_torque_setpoint_s torque_sp{};
// 	if (!_torque_sp_sub.update(&torque_sp)) {
// 		return;
// 	}

// 	vehicle_thrust_setpoint_s thrust_sp{};
// 	_thrust_sp_sub.copy(&thrust_sp);

// 	// Build desired vector
// 	Vector<float, 4> desired{};
// 	desired(0) = torque_sp.xyz[0];
// 	desired(1) = torque_sp.xyz[1];
// 	desired(2) = torque_sp.xyz[2];
// 	desired(3) = thrust_sp.xyz[2];

// 	Vector4f actuator_trim(0.0, 0.0, 0.0, 0.0);
// 	Vector4f control_trim(0.0, 0.0, 0.0, 0.0);

// 	// Solve using pseudo-inverse
// 	Vector<float, NUM_MOTORS> u = actuator_trim + _B_pinv * (desired - control_trim);

// 	actuator_motors_s out{};
// 	out.timestamp = hrt_absolute_time();

// 	for (int i = 0; i < NUM_MOTORS; i++) {
// 		float cmd = u(i);
// 		if (!PX4_ISFINITE(cmd)) cmd = 0.f;
// 		if (cmd < 0.f) cmd = 0.f;
// 		if (cmd > 1.f) cmd = 1.f;
// 		out.control[i] = cmd;
// 	}

// 	// PX4_INFO("Motor cmds: %.3f %.3f %.3f %.3f %.3f",
//     //      (double)out.control[0],
//     //      (double)out.control[1],
//     //      (double)out.control[2],
//     //      (double)out.control[3],
// 	//  (double)_hover_thrust
// 	// );

// 	_actuator_motors_pub.publish(out);
// }

void CdusAllocator::Run_ducted()
{
	if (should_exit()) {
		_torque_sp_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	vehicle_status_s vehicle_status{};
	if (_vehicle_status_sub.update(&vehicle_status)) {
		_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	}

	vehicle_control_mode_s vcm{};
	if (_vehicle_control_mode_sub.update(&vcm)) {
		_rate_control_enabled = vcm.flag_control_rates_enabled;
	}

	// Disarmed or not in rate control => publish zero motors (FAILSAFE CONDITION)
	if (!_armed || !_rate_control_enabled) {
		actuator_motors_s out{};
		out.timestamp = hrt_absolute_time();
		for (int i = 0; i < NUM_MOTORS; i++) { out.control[i] = 0.f; }
		_actuator_motors_pub.publish(out);
		return;
	}
	
	// For the case where the vehicle is not disarmed and is in rate control 
	// Update the torque and thrust setpoints
	vehicle_torque_setpoint_s torque_sp{};
	if (!_torque_sp_sub.update(&torque_sp)) {
		return;
	}

	vehicle_thrust_setpoint_s thrust_sp{};
	_thrust_sp_sub.copy(&thrust_sp);

	// Build desired vector
	Vector<float, 4> desired{};
	desired(0) = torque_sp.xyz[0]; // Roll Command 
	desired(1) = torque_sp.xyz[1]; // Pitch Command 
	desired(2) = torque_sp.xyz[2]; // Yaw Command 
	desired(3) = thrust_sp.xyz[2]; // Thrust Command 

	// Take Psuedo-Inverse of the allocation matrix to get the mixer matrix
	matrix::Matrix<float, 4, 4> mixer_matrix;
	matrix::geninv(allocation_matrix_ducted,mixer_matrix);
	
	Vector4f Raw_PWM; // Empty vector to hold raw PWM values
    for (int i = 0; i < 4; ++i) {
      Raw_PWM(i) = mixer_matrix(i,0)*desired(0)
          + mixer_matrix(i,1)*desired(1)
          + mixer_matrix(i,2)*desired(2)
          + mixer_matrix(i,3)*desired(3);
    }

	// Now, normalize delta PWM values 
    Vector4f delta_pwm(Raw_PWM(0), Raw_PWM(1), Raw_PWM(2), Raw_PWM(3));
    Vector4f delta_pwm_norm; 

    // Calculate delta PWM norm for each motor (PWM0 -> UR || PWM1 -> LR || PWM2 -> S1 || PWM3 -> S2)
    delta_pwm_norm(0) = clip((delta_pwm(0)/800.f), -1.f, 1.f); // Motor 1 
    delta_pwm_norm(1) = clip((delta_pwm(1)/800.f) , -1.f, 1.f); // Motor 2
    delta_pwm_norm(2) = clip((delta_pwm(2)/400.f) , -1.f, 1.f); // Servo 1  
    delta_pwm_norm(3) = clip((delta_pwm(3)/400.f), -1.f, 1.f); // Servo 2

    // Now add delta_pwm_norm with pwm_trim values and output as acutator_sp and         
    // clamp to [0,1] (single clamp is enough)

    // For mapping in QGC, _pwm_commands(0) --> Channel 1 (Motor 1 -- Upper Rotor) MAPS TO -----> delta_pwm_norm[0] 
    // For mapping in QGC, _pwm_commands(1) --> Channel 2 (Motor 2 -- Lower Rotor) MAPS TO -----> delta_pwm_norm[1] 
    // For mapping in QGC, _pwm_commands(2) --> Channel 3 (Motor 3 -- Servo 1) MAPS TO -----> delta_pwm_norm[2] 
    // For mapping in QGC, _pwm_commands(3) --> Channel 4 (Motor 4 -- Servo 2) MAPS TO -----> delta_pwm_norm[3] 

	Vector4f actuator_trim(0.6f, 0.6f, 0.5f, 0.5f); // Set PWM Trim Values (Normalized 0 to 1) [Upper Rotor, Lower Rotor, Servo 1, Servo 2]

    Vector4f _pwm_commands; 
    // Assign PWM Commands 
    _pwm_commands(0) = delta_pwm_norm(0) + actuator_trim(0); // Upper Rotor
    _pwm_commands(1) = delta_pwm_norm(1) + actuator_trim(1); // Lower Rotor
    _pwm_commands(2) = delta_pwm_norm(2) + actuator_trim(2); // Servo 1
    _pwm_commands(3) = delta_pwm_norm(3) + actuator_trim(3); // Servo 2 

	actuator_motors_s out{};
	out.timestamp = hrt_absolute_time();

	// PX4 Check for infinite comamnds and saturate all values below 0 to 0 and all values above 1 to 1; If not send pwm_commands through
	for (int i = 0; i < NUM_MOTORS; i++) {
		float cmd = _pwm_commands(i);
		if (!PX4_ISFINITE(cmd)) cmd = 0.f;
		if (cmd < 0.f) cmd = 0.f;
		if (cmd > 1.f) cmd = 1.f;
		out.control[i] = cmd;
	}	

	// PX4_INFO("Motor cmds: %.3f %.3f %.3f %.3f %.3f",
    //      (double)out.control[0],
    //      (double)out.control[1],
    //      (double)out.control[2],
    //      (double)out.control[3],
	//  (double)_hover_thrust
	// );

	_actuator_motors_pub.publish(out);
}

int CdusAllocator::task_spawn(int argc, char *argv[])
{
	CdusAllocator *instance = new CdusAllocator();
	if (!instance) {
		PX4_ERR("Allocation failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	return PX4_OK;
}

CdusAllocator *CdusAllocator::instantiate(int argc, char *argv[])
{
	return new CdusAllocator();
}

int CdusAllocator::custom_command(int argc, char *argv[])
{
	return print_usage("Unknown command");
}

int CdusAllocator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR(
CdusAllocator — simple quadcopter control allocator.

Hard-coded geometry + pseudo-inverse mapping:
    torque/thrust → actuator_motors

Drop-in replacement for ControlAllocator for quad-only experiments.
)DESCR"
	);

	PRINT_MODULE_USAGE_NAME("cdus_allocator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");

	return 0;
}

extern "C" __EXPORT int cdus_allocator_main(int argc, char *argv[])
{
	return CdusAllocator::main(argc, argv);
}
