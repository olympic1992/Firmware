/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file followme.cpp
 *
 * Helper class to track and follow a given position
 *
 * @author Jimmy Johnson <catch22@fastmail.net>
 */

#include "follow_target.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/follow_target.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/math/Limits.hpp>

#include "navigator.h"

using matrix::wrap_pi;

constexpr float FollowTarget::_follow_position_matricies[4][9];

FollowTarget::FollowTarget(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
	_current_vel.zero();
	_step_vel.zero();
	_est_target_vel.zero();
	_target_distance.zero();
	_target_position_offset.zero();
	_target_position_delta.zero();
    offset_PA.zero();
    offset_PB.zero();
    offset_PA_ned.zero();

}

void FollowTarget::on_inactive()
{
	reset_target_validity();
//    PX4_INFO("执行 FollowTarget::on_inactive()");
}

void FollowTarget::on_activation()
{
	_follow_offset = _param_tracking_dist.get() < 1.0F ? 1.0F : _param_tracking_dist.get();
    _follow_offset = 8.0f;

	_responsiveness = math::constrain((float) _param_tracking_resp.get(), .1F, 1.0F);
    _responsiveness = 0.0f;

    float L_distantPA(50.0f);//2.0f * _navigator->get_acceptance_radius());  //A点到从机目标位置的距离,默认应大于2倍认为到点的距离
    float L_distantPB(50.0f);  //B点到从机目标位置的距离,这个距离暂定20m

    offset_PA = {L_distantPA,0.0f};             //PA方向为正
    offset_PB = {-1.0f * L_distantPB , 0.0f};   //PB方向为负


//	_follow_target_position = _param_tracking_side.get();
    _follow_target_position = FOLLOW_FROM_LEFT; //自定义让飞机跟在左边

	if ((_follow_target_position > FOLLOW_FROM_LEFT) || (_follow_target_position < FOLLOW_FROM_RIGHT)) {
		_follow_target_position = FOLLOW_FROM_BEHIND;
	}



	_rot_matrix = (_follow_position_matricies[_follow_target_position]);

	if (_follow_target_sub < 0) {
		_follow_target_sub = orb_subscribe(ORB_ID(follow_target));
	}
}

void FollowTarget::on_active()
{
    //自定义输出的频率控制语句
    static bool INFO_enable{false};
    static hrt_abstime last_info_time{0};
    if(hrt_elapsed_time(&last_info_time)  * 1e-6f >= 2.0f) {
        INFO_enable = true;
        last_info_time = hrt_absolute_time();
    }



    struct map_projection_reference_s target_ref;
    uint64_t current_time = hrt_absolute_time();
    bool updated = false;


    orb_check(_follow_target_sub, &updated);

    if (updated) {  //如果获得了目标更新

        _target_updates++;
        //                PX4_INFO("_target_updates递增数值 =  %.0f " ,1.0 * _target_updates);


        // save last known motion topic

        _previous_target_motion = _current_target_motion;

        orb_copy(ORB_ID(follow_target), _follow_target_sub, &target_motion);


        //这里对接收到的位置进行初步计算

        if (_current_target_motion.timestamp == 0) {
            _current_target_motion = target_motion;
        }

        _current_target_motion.timestamp = target_motion.timestamp;
        //下面的_responsiveness用来对本次接收的目标位置进行滤波.
        _current_target_motion.lat = (_current_target_motion.lat * (double)_responsiveness) + target_motion.lat * (double)(
                    1 - _responsiveness);
        _current_target_motion.lon = (_current_target_motion.lon * (double)_responsiveness) + target_motion.lon * (double)(
                    1 - _responsiveness);
        //        _current_target_motion.alt = _current_target_motion.alt * _responsiveness + target_motion.alt * (1 - _responsiveness); //附加一个对高度的滤波
        _current_target_motion.alt = target_motion.alt-10.0f;

        _current_target_motion.vx = target_motion.vx;  //
        _current_target_motion.vy = target_motion.vy;
        _current_target_motion.vz = target_motion.vz;

        _current_target_motion.yaw_body = target_motion.yaw_body;


    }
    //如果一段时间没有获得目标更新
    else if (((current_time - _current_target_motion.timestamp) / 1000) > TARGET_TIMEOUT_MS && target_velocity_valid()) {
        PX4_INFO("一段时间没有获得目标更新 _target_updates %d \n",_target_updates);
        reset_target_validity();
    }


    //    PX4_INFO("_send_ftarget.alt :\t%8.4f \t%d lat:\t%8.6f vx:\t%8.4f "
    //                                     ,double(_send_follow_target.alt),_send_follow_target.timestamp,_send_follow_target.lat,double(_send_follow_target.vx));





    if (target_position_valid() && updated) {
        _follow_target_state = TRACK_POSITION;
        if(INFO_enable) PX4_INFO("追踪位置: _follow_target_state = %d ",_follow_target_state);

        matrix::Vector2f  V_P1{_current_target_motion.vx,_current_target_motion.vy};    //收到的主机的速度.单位 m/s

        /*这部分将从机相对位置(速度轴系)转换到从机相对位置(地轴系)*/ //主机被转换坐标轴的方向,向速度方向为x正,速度方向右边为y正,正北方向为地轴系x正,正东方向为地轴系y正
        float cos_yaw_ned = V_P1(0) / V_P1.length();
        float sin_yaw_ned = V_P1(1) / V_P1.length();

        if(INFO_enable) PX4_INFO("地速方向与机头指向的偏差角度: %4.2f deg",double(math::degrees(atan2f(cos_yaw_ned,sin_yaw_ned)) - math::degrees(_current_target_motion.yaw_body))) ;

        if(V_P1.length()<5.0f){   //当地速很小时,飞机有可能遭遇大逆风,或者主机没起飞,此时使用飞机机头指向

            //注意调试:这部分需要在飞行时确认飞机的地速方向与机头指向偏差不大,务必注意根据试验情况确定
            cos_yaw_ned = cos(double(_current_target_motion.yaw_body));
            sin_yaw_ned = sin(double(_current_target_motion.yaw_body));
        }

        offset_PA_ned = {offset_PA(0) * cos_yaw_ned + offset_PA(1) * sin_yaw_ned,
                         offset_PA(1) * cos_yaw_ned - offset_PA(0) * sin_yaw_ned};
        matrix::Vector2f offset_PB_ned {offset_PB(0) * cos_yaw_ned - offset_PB(1) * sin_yaw_ned,
                    offset_PB(1) * cos_yaw_ned + offset_PB(0) * sin_yaw_ned};

        PB_with_offset = PA_with_offset = _current_target_motion;

        map_projection_init(&target_ref,  _current_target_motion.lat, _current_target_motion.lon);
        //计算A点坐标,A点位置为主机地速方向前方一定距离
        map_projection_reproject(&target_ref, offset_PA_ned(0), offset_PA_ned(1),&PA_with_offset.lat, &PA_with_offset.lon);
        //计算B点坐标,B点位置为从机地速方向后方一定距离
        map_projection_reproject(&target_ref, offset_PB_ned(0), offset_PB_ned(1),&PB_with_offset.lat, &PB_with_offset.lon);

        set_follow_target_item(&_mission_item, _param_min_alt.get(), PA_with_offset);
        update_ABposition_sp();



        if(INFO_enable) PX4_INFO("PA_with_offset.alt :\t%8.4f \t%d lat:\t%8.6f vx:\t%8.4f ",double(PA_with_offset.alt),PA_with_offset.timestamp,PA_with_offset.lat,double(PA_with_offset.vx));
        //            if(INFO_enable) PX4_INFO("_current_targe.alt :\t%8.4f \t%d lat:\t%8.6f vx:\t%8.4f ",double(_current_target_motion.alt),_current_target_motion.timestamp,_current_target_motion.lat,double(_current_target_motion.vx));
        if(INFO_enable) PX4_INFO("PB_with_offset.alt :\t%8.4f \t%d lat:\t%8.6f vx:\t%8.4f ",double(PB_with_offset.alt),PB_with_offset.timestamp,PB_with_offset.lat,double(PB_with_offset.vx));
        if(INFO_enable) PX4_INFO("offset_PA(0)= %.0f   offset_PB(0)= %.0f \n",double(offset_PA(0)),double(offset_PB(0)));
        if(INFO_enable) PX4_INFO("target_motion.lat %8.7f    _current_target_motion.lat %8.7f ",  target_motion.lat,_current_target_motion.lat);



    } else { //飞机目标位置无效时,设置飞机当前位置作为目标位置,飞机会在此处长时间盘旋

        if(_follow_target_state == SET_WAIT_FOR_TARGET_POSITION) {
            PX4_INFO("设置等待追踪位置: _follow_target_state = %d ",_follow_target_state);

            follow_target_s target = {};
            // for now set the target at the minimum height above the uav
            target.lat = _navigator->get_global_position()->lat;
            target.lon = _navigator->get_global_position()->lon;
            target.alt = 0.0F;

            set_follow_target_item(&_mission_item, _param_min_alt.get(), target);
            update_position_sp(false, false, _yaw_rate);
            _follow_target_state = WAIT_FOR_TARGET_POSITION;
        }
        if (_follow_target_state == WAIT_FOR_TARGET_POSITION) {
            PX4_INFO("等待追踪位置: _follow_target_state = %d ",_follow_target_state);
            if (is_mission_item_reached() && target_position_valid()) {
                _follow_target_state = TRACK_POSITION;
            }
        }
    }
    if(updated) INFO_enable = false;

}

void FollowTarget::update_position_sp(bool use_velocity, bool use_position, float yaw_rate)
{
	// convert mission item to current setpoint

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// activate line following in pos control if position is valid

    pos_sp_triplet->previous.valid = use_position;
	pos_sp_triplet->previous = pos_sp_triplet->current;
	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET;
	pos_sp_triplet->current.position_valid = use_position;
	pos_sp_triplet->current.velocity_valid = use_velocity;
	pos_sp_triplet->current.vx = _current_vel(0);
	pos_sp_triplet->current.vy = _current_vel(1);
	pos_sp_triplet->next.valid = false;
	pos_sp_triplet->current.yawspeed_valid = PX4_ISFINITE(yaw_rate);
	pos_sp_triplet->current.yawspeed = yaw_rate;
	_navigator->set_position_setpoint_triplet_updated();
}


void FollowTarget::update_ABposition_sp()
{
    // convert mission item to current setpoint

    struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

    // activate line following in pos control if position is valid

    pos_sp_triplet->previous.valid = true ;
    pos_sp_triplet->previous.lat = PB_with_offset.lat;
    pos_sp_triplet->previous.lon = PB_with_offset.lon;
    pos_sp_triplet->previous.alt = PB_with_offset.alt;

    mission_apply_limitation(_mission_item);
    mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
    pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET;
    pos_sp_triplet->current.position_valid = true;
    pos_sp_triplet->current.velocity_valid = true;
    pos_sp_triplet->current.vx = PA_with_offset.vx;
    pos_sp_triplet->current.vy = PA_with_offset.vy;
    pos_sp_triplet->current.vz = PA_with_offset.vz;
    pos_sp_triplet->current.yaw = PA_with_offset.yaw_body;
    pos_sp_triplet->current.x = offset_PA_ned(0);   //利用这个变量来传递A点相对于从机目标位置的坐标
    pos_sp_triplet->current.y = offset_PA_ned(1);   //利用这个变量来传递A点相对于从机目标位置的坐标


    pos_sp_triplet->current.timestamp = target_motion.timestamp; //传递主机时间
    pos_sp_triplet->next.valid = false;

    _navigator->set_position_setpoint_triplet_updated();
}



void FollowTarget::reset_target_validity()
{
	_yaw_rate = NAN;
	_previous_target_motion = {};
	_current_target_motion = {};
	_target_updates = 0;
	_current_vel.zero();
	_step_vel.zero();
	_est_target_vel.zero();
	_target_distance.zero();
	_target_position_offset.zero();
	reset_mission_item_reached();
	_follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
}

bool FollowTarget::target_velocity_valid()
{
	// need at least 2 continuous data points for velocity estimate
	return (_target_updates >= 2);
}

bool FollowTarget::target_position_valid()
{
	// need at least 1 continuous data points for position estimate
	return (_target_updates >= 1);
}

void
FollowTarget::set_follow_target_item(struct mission_item_s *item, float min_clearance, follow_target_s &target)
{
    if (_navigator->get_land_detected()->landed) {
		/* landed, don't takeoff, but switch to IDLE mode */
		item->nav_cmd = NAV_CMD_IDLE;

	} else {


		item->nav_cmd = NAV_CMD_DO_FOLLOW_REPOSITION;

		/* use current target position */
		item->lat = target.lat;
        item->lon = target.lon;

        //高度这里怎么传递还需要与主机联调决定
        //这里把追踪模式的最低高度改成40米,从机理论上会在40米高度以上飞行
        item->altitude = math::max(target.alt,_navigator->get_home_position()->alt + math::max(40.0f,min_clearance));

        //         PX4_INFO("DO_FOLLOW_REPOSITION target.alt = %.1f",double(target.alt));

//        if (min_clearance > 40.0f) {  //这里把追踪模式的最低高度改成40米,从机理论上会在40米高度飞行
//			item->altitude += min_clearance;

//		} else {
//            item->altitude += 40.0f; // if min clearance is bad set it to 8.0 meters (well above the average height of a person)
//		}
	}

	item->altitude_is_relative = false;
//    item->yaw = yaw;
	item->loiter_radius = _navigator->get_loiter_radius();
	item->acceptance_radius = _navigator->get_acceptance_radius();
	item->time_inside = 0.0f;
	item->autocontinue = false;
	item->origin = ORIGIN_ONBOARD;
}
