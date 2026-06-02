px4_add_module(
	MODULE modules__qpd_control
	MAIN qpd_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		qpd_control.cpp
		qpd_control.hpp
	DEPENDS
		circuit_breaker
		mathlib
		px4_work_queue
	)
