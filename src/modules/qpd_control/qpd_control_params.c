/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file qpd_control_params.c
 *
 * Parameters for Quaternion PD Attitude Controllerg
 */

/**
 * Quaternion Attitude Controller Roll Proportional Gain
 *
 * Tune +/- 5 starting at 10
 * 0 disables the filter
 *
 * @min 0
 * @max 100
 * @unit Hz
 * @decimal 3
 * @group QPD Controller
 */

 PARAM_DEFINE_FLOAT(QPD_ROLL_KP, 1.0f);

/**
 * Quaternion Attitude Controller Pitch Proportional Gain
 *
 * Tune +/- 5 starting at 10
 * 0 disables the filter
 *
 * @min 0
 * @max 100
 * @unit Hz
 * @decimal 3
 * @group QPD Controller
 */

 PARAM_DEFINE_FLOAT(QPD_PITCH_KP, 1.0f);

/**
 * Quaternion Attitude Controller Yaw Proportional Gain
 *
 * Tune +/- 5 starting at 10
 * 0 disables the filter
 *
 * @min 0
 * @max 100
 * @unit Hz
 * @decimal 3
 * @group QPD Controller
 */

 PARAM_DEFINE_FLOAT(QPD_YAW_KP, 0.2f);

/**
 * Quaternion Attitude Controller Roll Derivative Gain
 *
 * Tune +/- 5 starting at 10
 * 0 disables the filter
 *
 * @min 0
 * @max 100
 * @unit Hz
 * @decimal 3
 * @group QPD Controller
 */

 PARAM_DEFINE_FLOAT(QPD_ROLL_KV, 0.2f);

/**
 * Quaternion Attitude Controller Pitch Derivative Gain
 *
 * Tune +/- 5 starting at 10
 * 0 disables the filter
 *
 * @min 0
 * @max 100
 * @unit Hz
 * @decimal 3
 * @group QPD Controller
 */

 PARAM_DEFINE_FLOAT(QPD_PITCH_KV, 0.2f);

/**
 * Quaternion Attitude Controller Yaw Derivative Gain
 *
 * Tune +/- 5 starting at 10
 * 0 disables the filter
 *
 * @min 0
 * @max 100
 * @unit Hz
 * @decimal 3
 * @group QPD Controller
 */

 PARAM_DEFINE_FLOAT(QPD_YAW_KV, 0.01f);

 /**
 * Quaternion Attitude Controller Yaw Scale Parameter
 *
 * Tune +/- 5 starting at 10
 * 0 disables the filter
 *
 * @min 0
 * @max 10
 * @unit Hz
 * @decimal 3
 * @group QPD Controller
 */

 PARAM_DEFINE_FLOAT(QPD_YAW_RATE_SCL, 0.01f);

  /**
 * Yaw Constrain Parameter for the Quaternion PD Attitude Controller.
 *
 * Enbale param to constrain the yaw setpoint to the inital yaw when the vehicle is armed.
 *
 * @boolean
 * @group QPD Controller
 */
 PARAM_DEFINE_INT32(QPD_YAW_CONST, 0);



