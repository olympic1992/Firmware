/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 DeveloMPent Team. All rights reserved.
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

#define mainplaneID 1

using matrix::wrap_pi;

constexpr float FollowTarget::_follow_position_matricies[4][9];

FollowTarget::FollowTarget(Navigator *navigator) :
    MissionBlock(navigator),
    ModuleParams(navigator)
{
    _current_vel.zero();
    _step_vel.zero();
    _target_position_offset.zero();
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
    _responsiveness = 0.25f;

      _vehicle_gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));


    //	_follow_target_position = _param_tracking_side.get();
    _follow_target_position = FOLLOW_FROM_LEFT; //自定义让飞机跟在左边

    if ((_follow_target_position > FOLLOW_FROM_LEFT) || (_follow_target_position < FOLLOW_FROM_RIGHT)) {
        _follow_target_position = FOLLOW_FROM_BEHIND;
    }



    _rot_matrix = (_follow_position_matricies[_follow_target_position]);

    if (_follow_target_sub < 0) {
        _follow_target_sub = orb_subscribe(ORB_ID(follow_target));
    }

    if (_vehicle_status_sub < 0) {
        _vehicle_status_sub             = orb_subscribe(ORB_ID(vehicle_status));
    }

}

void FollowTarget::on_active()
{
    bool info_enable = false;

    //控制输出频率
    static uint64_t prevsendtime = 0;
    uint64_t dt_sendtime = hrt_elapsed_time(&prevsendtime);
    prevsendtime = hrt_absolute_time();
    float sendHZ = float(1.0 /(double(dt_sendtime) * 1e-6));
    static uint64_t previnfotime{0};
    if((hrt_elapsed_time(&previnfotime) * 1e-6) > 10.0){
        previnfotime = prevsendtime;
        PX4_INFO("FollowTarget运行频率: %3.1fHz",double(sendHZ));
        info_enable = true;
    }

    //当前如果是主机,不继续执行
    status_poll(); //主要为了更新sysid
    if(sys_id == mainplaneID){   //当飞机是主机的时候,不执行下面的程序
        if(info_enable) PX4_INFO(" 飞机是主机  不执行下面的程序");
        return;
    }


    //开始正常程序
    static uint64_t last_updated_time = hrt_absolute_time();
    uint64_t current_time = hrt_absolute_time();









    bool follow_target_updated = false;
    static follow_target_s MP_position{};
    orb_check(_follow_target_sub, &follow_target_updated);
    if (follow_target_updated) {        //如果获得了目标更新
        orb_copy(ORB_ID(follow_target), _follow_target_sub, &MP_position);
        if(info_enable) PX4_INFO("获得目标更新 MP_position.timestamp: %.0f",double(MP_position.timestamp));




        last_updated_time = hrt_absolute_time();
        _target_updates++;

        static struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

        if (_navigator->get_land_detected()->landed) {

            if(info_enable)  PX4_INFO(" land_detected()->landed更新");
            /* landed, don't takeoff, but switch to IDLE mode */
            _mission_item.nav_cmd = NAV_CMD_IDLE;
            pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;
        } else {
            _mission_item.nav_cmd = NAV_CMD_DO_FOLLOW_REPOSITION;
            pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET;
        }

        pos_sp_triplet->previous.valid = true ;
        pos_sp_triplet->current.valid = true ;


        pos_sp_triplet->current.home_alt = _navigator->get_home_position()->alt;


        pos_sp_triplet->current.alt += 40.0f; // if min clearance is bad set it to 40.0 meters (well above the average height of a person)




        _navigator->set_position_setpoint_triplet_updated();


    } else if (((current_time - last_updated_time) / 1000) > TARGET_TIMEOUT_MS) {  //如果一段时间没有获得目标更新
        if(info_enable) mavlink_log_info(_navigator->get_mavlink_log_pub(),"#没有获得主机发来的follow_target主题");

        if(_target_updates >=1){
            reset_target_validity();

            mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "#%d号目标更新超时,设置在当前位置",sys_id);
            // Climb to the minimum altitude
            // and wait until a position is received

            follow_target_s target = {};

            // for now set the target at the minimum height above the uav

            target.lat = _navigator->get_global_position()->lat;
            target.lon = _navigator->get_global_position()->lon;
            target.alt = 0.0F;

            set_follow_target_item(&_mission_item, _param_min_alt.get(), target, NAN);

            update_position_sp(false, false, NAN);

        }
    }




    //待办:在这里考虑增加一个返航程序,当自动执行任务时,如果长时间没有收到主机的信号,飞机在当前位置盘旋较长时间后就可以返航

    //待办:还可以考虑增加一个程序,当2号从机确认长时间没有获得主机信息时,2号机可以自动成为主机,代替1号机,考虑以下实现方法


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


}



void FollowTarget::reset_target_validity()
{
    yaw_rate_sp = NAN;
    MP_position_prev = {};
    MP_position_filter = {};
    _target_updates = 0;
    _current_vel.zero();
    _step_vel.zero();
    _target_position_offset.zero();
    reset_mission_item_reached();
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
FollowTarget::status_poll()
{
    bool _updated;
    static vehicle_status_s  status{};
    orb_check(_vehicle_status_sub, &_updated);
    if (_updated) {
        orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &status);  //从飞行器状态中获得sysid

        sys_id = status.system_id;
        comp_id = status.component_id;
        status_valid = (status.timestamp > 0);
        nav_status = status.nav_state;

    }
}



void
FollowTarget::set_follow_target_itemAB(struct mission_item_s *item, float min_clearance, float yaw)
{

    if (_navigator->get_land_detected()->landed) {
        /* landed, don't takeoff, but switch to IDLE mode */
        item->nav_cmd = NAV_CMD_IDLE;

        PX4_INFO("输出_navigator->get_land_detected()->landed");

    } else {

          PX4_INFO("输出NAV_CMD_DO_FOLLOW_REPOSITION");

        item->nav_cmd = NAV_CMD_DO_FOLLOW_REPOSITION;

        /* use current target position */
//        item->lat = target.lat;
//        item->lon = target.lon;
//        item->altitude = _navigator->get_home_position()->alt;

//        if (min_clearance > 40.0f) {
//            item->altitude += min_clearance;

//        } else {
//            item->altitude += 40.0f; // if min clearance is bad set it to 40.0 meters (well above the average height of a person)
//        }
    }

    item->altitude_is_relative = false;
//    item->yaw = yaw;
    item->loiter_radius = _navigator->get_loiter_radius();
    item->acceptance_radius = _navigator->get_acceptance_radius();
    item->time_inside = 0.0f;
    item->autocontinue = false;
    item->origin = ORIGIN_ONBOARD;


}



void
FollowTarget::set_follow_target_item(struct mission_item_s *item, float min_clearance, follow_target_s &target,
                     float yaw)
{
    if (_navigator->get_land_detected()->landed) {
        /* landed, don't takeoff, but switch to IDLE mode */
        item->nav_cmd = NAV_CMD_IDLE;

    } else {

        item->nav_cmd = NAV_CMD_DO_FOLLOW_REPOSITION;

        /* use current target position */
        item->lat = target.lat;
        item->lon = target.lon;
        item->altitude = _navigator->get_home_position()->alt;

        if (min_clearance > 40.0f) {
            item->altitude += min_clearance;

        } else {
            item->altitude += 40.0f; // if min clearance is bad set it to 40.0 meters (well above the average height of a person)
        }
    }

    item->altitude_is_relative = false;
    item->yaw = yaw;
    item->loiter_radius = _navigator->get_loiter_radius();
    item->acceptance_radius = _navigator->get_acceptance_radius();
    item->time_inside = 0.0f;
    item->autocontinue = false;
    item->origin = ORIGIN_ONBOARD;
}
