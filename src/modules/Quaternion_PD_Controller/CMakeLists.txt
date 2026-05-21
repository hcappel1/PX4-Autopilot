px4_add_module(
	MODULE modules__quaternion_pd_controller
	MAIN quaternion_PD_Controller
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		quaternion_PD_Controller.cpp
		quaternion_PD_Controller.hpp
	DEPENDS
		circuit_breaker
		mathlib
		px4_work_queue
	)
