#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/parameter_update.h>


#include <matrix/matrix/math.hpp>

using matrix::Vector3f;
using namespace time_literals;

/**
 * Very simple quadcopter rate controller.
 *
 * - PID on body rates (roll, pitch, yaw)
 * - Takes rate + body thrust setpoints via vehicle_rates_setpoint
 * - Outputs vehicle_torque_setpoint + vehicle_thrust_setpoint
 */
class qpd_control final :
	public ModuleBase<qpd_control>,
	public ModuleParams,
	public px4::WorkItem
{
public:
	qpd_control();
	~qpd_control() override;

	/** Initialize subscriptions and schedule */
	bool init();

	/** WorkItem entry point (runs on gyro updates) */
	void Run() override;

	/** ModuleBase interface */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

private:

    uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::SubscriptionCallbackWorkItem _angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// uORB publications
	uORB::Publication<vehicle_torque_setpoint_s> _torque_sp_pub{ORB_ID(vehicle_torque_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s> _thrust_sp_pub{ORB_ID(vehicle_thrust_setpoint)};

    // Methods
    void calcRollTorque();
    void calcPitchTorque();
    void calcYawTorque();
	void parameters_updated();
	void updateYawRateSp();
	void integrateYawSp(float& dt);
	void resetYawInit(uint64_t now);

	// latest control mode
	vehicle_control_mode_s _control_mode{};

	// Cached topic data
	manual_control_setpoint_s   _manual_control{};
	vehicle_local_position_s    _local_position{};
	vehicle_attitude_s          _attitude{};
    vehicle_attitude_setpoint_s _attitude_sp{};
	vehicle_angular_velocity_s  _angular_velocity{};
	vehicle_status_s 			_vehicle_status{};

	// Controller I/O 
	matrix::Vector3f _vel_est_ned{0.f, 0.f, 0.f};
	matrix::Vector3f _vel_est_body{0.f, 0.f, 0.f};
	matrix::Quatf    _q_att{1.f, 0.f, 0.f, 0.f};
    matrix::Quatf    _q_att_sp{1.f, 0.f, 0.f, 0.f};
	matrix::Vector3f _rates_body{0.f, 0.f, 0.f};
	matrix::Dcmf     _R_att{matrix::eye<float, 3>()};

	// Outputs
	matrix::Vector3f _torque_sp{0.f, 0.f, 0.f};  // [L, M, N] (N*m)
	matrix::Vector3f _thrust_sp{0.f, 0.f, 0.f};  // typically thrust in N or normalized

	uint64_t _last_run{0};                  // last timestamp_sample [µs]
	uint64_t _first_run{0};

	// Yaw adjustment params
	bool _constrain_yaw{false};
	bool _yaw_initialized{false};
	bool _armed_prev{false};   // last seen arming state
	float _yaw_sp{0.f};
	float _yaw_rate_sp{0.f};
	float _yaw_rate_scale{10.f};
	int iter{0};

    // Physical parameters
	float _mass{0.07f};
	float _Ixx{0.02f};
	float _Iyy{0.02f};
	float _Izz{0.02f};

	// Quaternion PD gains 
	float _Cd{0.0f};
	float _Kv_r{1.0f};
	float _Kv_p{1.0f};
	float _Kv_y{1.0f};
	float _Ka_r{0.2f};
	float _Ka_p{0.2f};
	float _Ka_y{0.2f};
	float _torque_scale{1.0};


	// Temporary fix, define params for the quaternion PD controller. These should be tuned for the specific vehicle and ideally exposed as parameters.
	// Quaternion PD gains 
	float qpd_Roll_kP{1.0f};
	float qpd_Pitch_kP{1.0f};
	float qpd_Yaw_kP{1.0f};
	float qpd_Roll_kD{0.2f};
	float qpd_Pitch_kD{0.2f};
	float qpd_Yaw_kD{0.2f};


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BSA_ROLL_KV>) _param_bsa_roll_kv,
		(ParamFloat<px4::params::BSA_ROLL_KA>) _param_bsa_roll_ka,

		(ParamFloat<px4::params::BSA_PITCH_KV>) _param_bsa_pitch_kv,
		(ParamFloat<px4::params::BSA_PITCH_KA>) _param_bsa_pitch_ka,

		(ParamFloat<px4::params::BSA_YAW_KV>) _param_bsa_yaw_kv,
		(ParamFloat<px4::params::BSA_YAW_KA>) _param_bsa_yaw_ka,

		(ParamFloat<px4::params::BSA_IXX>) _param_bsa_ixx,
		(ParamFloat<px4::params::BSA_IYY>) _param_bsa_iyy,
		(ParamFloat<px4::params::BSA_IZZ>) _param_bsa_izz
	)
	
};
