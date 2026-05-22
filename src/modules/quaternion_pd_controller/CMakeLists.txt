px4_add_module(
	MODULE modules__quaternion_pd_controller
	MAIN quaternion_pd_controller
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		quaternion_pd_controller.cpp
		quaternion_pd_controller.hpp
	DEPENDS
		circuit_breaker
		mathlib
		px4_work_queue
	)
