/***************************************************************************
 *
 *   Copyright (c) 2016 PX4 DeveloMPent Team. All rights reserved.
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
 * INCIDENTAL, SECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file followme.cpp
 *
 * Helper class to track and follow a given position
 *
 * @author Jimmy Johnson <catch22@fastmail.net>
 */

#pragma once

#include "navigator_mode.h"
#include "mission_block.h"

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

#include <px4_module_params.h>
#include <uORB/topics/follow_target.h>



class FollowTarget : public MissionBlock, public ModuleParams
{

public:
	FollowTarget(Navigator *navigator);
	~FollowTarget() = default;

	void on_inactive() override;
	void on_activation() override;
	void on_active() override;

private:

    static constexpr int TARGET_TIMEOUT_MS = 2000;
    static constexpr int TARGET_ACCEPTANCE_RADIUS_M = 8;  //从机对目标位置的跟踪范围,距离小于此值时,认为飞机进圈,会切换跟踪模式
	static constexpr int INTERPOLATION_PNTS = 20;
    static constexpr float FF_K = .5F;
    static constexpr float OFFSET_M = 8.0f;

	enum FollowTargetState {
		TRACK_POSITION,
		TRACK_VELOCITY,
		SET_WAIT_FOR_TARGET_POSITION,
		WAIT_FOR_TARGET_POSITION
	};

	enum {
		FOLLOW_FROM_RIGHT,
		FOLLOW_FROM_BEHIND,
		FOLLOW_FROM_FRONT,
		FOLLOW_FROM_LEFT
	};

    static constexpr float _follow_position_matricies[4][9] = {      //从机相对主机位置的矩阵集合
		{ 1.0F, -1.0F, 0.0F,  1.0F,  1.0F, 0.0F, 0.0F, 0.0F, 1.0F}, // follow right
		{-1.0F,  0.0F, 0.0F,  0.0F, -1.0F, 0.0F, 0.0F, 0.0F, 1.0F}, // follow behind
		{ 1.0F,  0.0F, 0.0F,  0.0F,  1.0F, 0.0F, 0.0F, 0.0F, 1.0F}, // follow front
		{ 1.0F,  1.0F, 0.0F, -1.0F,  1.0F, 0.0F, 0.0F, 0.0F, 1.0F}  // follow left side
	};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::NAV_MIN_FT_HT>)	_param_min_alt,
		(ParamFloat<px4::params::NAV_FT_DST>) _param_tracking_dist,
		(ParamInt<px4::params::NAV_FT_FS>) _param_tracking_side,
		(ParamFloat<px4::params::NAV_FT_RS>) _param_tracking_resp
	)

	int _follow_target_position{FOLLOW_FROM_BEHIND};

	int _follow_target_sub{-1};
    int _vehicle_status_sub{-1};

      int		_vehicle_gps_position_sub{-1};


	float _step_time_in_ms{0.0f};
    float _follow_offset{OFFSET_M};  //从机跟踪目标到主机的距离

    uint64_t _target_updates{0};
	uint64_t _last_update_time{0};



	matrix::Vector3f _current_vel;
	matrix::Vector3f _step_vel;


    matrix::Vector3f _target_position_offset; //从机目标位置到主机位置的位置向量

    matrix::Vector3f _filteredMP_position_delta3D;


    matrix::Vector2f offset_PA_ned;
    matrix::Vector2f offset_PB_ned;




    follow_target_s MP_position_filter{};
    follow_target_s MP_position_prev{}; //主机的上一个有效坐标位置
    follow_target_s  _send_follow_target{};



    follow_target_s PA_position_sp{};
    follow_target_s PB_position_sp{};


    matrix::Vector3f MP_position_delta3D{};   //主机在dt时间中的位移向量
    matrix::Vector3f MP_velocity_average3D{};

    matrix::Vector2f L_MPtoSP{};

    matrix::Vector2f MP_speed_ned {};
    matrix::Vector2f L_MPtoSP_ned{};

follow_target_s  SP_position_sp{};

float cos_MP_yaw{};
float sin_MP_yaw{};

matrix::Vector2f L_ned{0.0f,0.0f};



matrix::Vector3f _target_distance{}; //从机实际位置到主机的位置向量


    float yaw_rate_sp{0.0f};
    float _responsiveness{0.0f};


    uint8_t sys_id{0};
    uint8_t comp_id{0};
    bool status_valid{false};
    uint8_t nav_status{0};

	// Mavlink defined motion reporting capabilities
	enum {
		POS = 0,
		VEL = 1,
		ACCEL = 2,
		ATT_RATES = 3
	};




    matrix::Dcmf _rot_matrix;  //表示从机在主机周围的相对位置矩阵

	void track_target_position();
	void track_target_velocity();
    bool target_velocity_valid();
    bool target_position_valid();
	void reset_target_validity();
	void update_position_sp(bool velocity_valid, bool position_valid, float yaw_rate);
    void update_ABposition_sp();
    matrix::Vector2f bodytoNED(matrix::Vector2f L_body,matrix::Vector2f speed_ned, float yaw);
    void updateMP_position();
	void update_target_velocity();

    void status_poll();



	/**
	 * Set follow_target item
	 */
    void set_follow_target_item(struct mission_item_s *item, float min_clearance, follow_target_s &target,
                                float yaw);

    void set_follow_target_itemAB(struct mission_item_s *item, float min_clearance,float yaw);


};
