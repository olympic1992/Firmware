/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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

#include "FormationControl.hpp"

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int formation_control_main(int argc, char *argv[]);

FormationControl::FormationControl():
    _idparam_initialized(false),
  _param_system_id(PARAM_INVALID),
  _param_component_id(PARAM_INVALID)
{

}

FormationControl::~FormationControl()
{

}




void
FormationControl::manual_control_setpoint_poll()
{
    /* check if there is a new setpoint */
    bool _updated;
    orb_check(_manual_control_setpoint_sub, &_updated);

    if (_updated) {
        orb_copy(ORB_ID(manual_control_setpoint), _manual_control_setpoint_sub, &_manual_sp);
    }
}

//测试数据发送程序,一直发送假的GPS位置点和地速
void
FormationControl::test_data_program(bool enable_test)
{

    if(enable_test == false){ //程序执行的开关
        return;
    }

    run_test = enable_test;


}


void
FormationControl::formationrec_sp_poll()
{
    /* check if there is a new setpoint */
    bool formationrec_updated;
    orb_check(_formationrec_sub, &formationrec_updated);

    if (formationrec_updated) {
        orb_copy(ORB_ID(formationrec), _formationrec_sub, &P1_received);
//printf("收到主机编队信息.....P1_received.lat  :  %.7f \n", (double)P1_received.lat);
//printf("P1_received.alt :\t%8.4f  \n",double(P1_received.alt));

    }






}



void
FormationControl::status_poll()
{
    bool _updated;
    static vehicle_status_s            status{};
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
FormationControl::attitude_poll()
{
    bool _updated;
    orb_check(_att_sub, &_updated);
    if (_updated) {
          orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_mainuav_att);  //从飞行器状态中获得sysid
    }
}









void
FormationControl::formationx_sp_publish()
{

    if (P1_send_pub != nullptr) {
        /* publish the attitude rates setpoint */
        orb_publish(ORB_ID(formationx), P1_send_pub, &P1_send);
//            PX4_INFO("推送formationx_sp_publish");

    } else {
        /* advertise the attitude rates setpoint */
        P1_send_pub = orb_advertise(ORB_ID(formationx), &P1_send);
    }
}



//从机向本机uorb总线发送follow_target的目标位置和速度
void
FormationControl::send_follow_target_publish()
{

    if (_send_follow_target_pub != nullptr) {
        /* publish the attitude rates setpoint */
        orb_publish(ORB_ID(follow_target), _send_follow_target_pub, &_send_follow_target);
//        PX4_INFO("当前是从机,推送ORB_ID(follow_target)");

    } else {
        /* advertise the attitude rates setpoint */
        _send_follow_target_pub = orb_advertise(ORB_ID(follow_target), &_send_follow_target);
    }
}




//void
//FormationControl::hold_offboard_status()
//{
//    _offboard_control_mode.ignore_position = true;
//    _offboard_control_mode.ignore_velocity = true;
//    _offboard_control_mode.ignore_acceleration_force = true;

//    _offboard_control_mode.timestamp = hrt_absolute_time();


//    if (_offboard_control_mode_pub == nullptr) {
//        _offboard_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &_offboard_control_mode);
//    } else {
//        orb_publish(ORB_ID(offboard_control_mode), _offboard_control_mode_pub, &_offboard_control_mode);
////         PX4_INFO("推送offboard_control_mode_publish");
//    }
//}

//void
//FormationControl::enable_offboard_mode(bool offboard_sw)
//{
//    orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &status);
//    _command.command            = vehicle_command_s::VEHICLE_CMD_NAV_GUIDED_ENABLE;//设置模式命令id
//    _command.param1             = offboard_sw;
//    _command.target_system      = status.system_id;
//    _command.target_component   = status.component_id;

//    if (_vehicle_command_pub != nullptr) {
//        orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &_command);//发布这个命令
//    } else {
//        _vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &_command);
//    }

//}

//用aux1开关控制是否进入 followme模式,使用时需要将某个通道映射到aux1上.
bool
FormationControl::check_aux_follow_sw()
{
  return true;  //调试语句,注意删除
    manual_control_setpoint_poll();

//    printf("_manual_sp.aux1 : %.2f \n",(double)_manual_sp.aux1);
    if (_manual_sp.aux1 <= 1.2f && _manual_sp.aux1 >= -1.2f) {
        if (_manual_sp.aux1 >= 0.5f){
            return true;
        } else {
            return false;
        }
    } else
        return false;
}




void
FormationControl::enable_follow_target_mode(bool follow_target_enabled)
{
    if (follow_target_enabled) {
        //        printf("启用followme模式 %d \n",follow_target_enabled);
        //        status_poll();  //获取飞机状态

        if(nav_status != vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET){

            _command.command            = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;//设置模式命令id
            _command.param1             = 1.0f;
            _command.param2             = 4.0f;
            _command.param3             = 8.0f;
            _command.target_system      = sys_id;
            _command.target_component   = comp_id;

            if (_vehicle_command_pub != nullptr) {
                orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &_command);//发布这个命令
            } else {
                _vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &_command);
            }
        }
    }
}



//void
//FormationControl::heartbeat_poll()
//{
//    /* check if there is a new setpoint */
//    bool heartbeat_updated;
//    orb_check(_heartbeat_sub, &heartbeat_updated);

//    if (heartbeat_updated) {
//        orb_copy(ORB_ID(heartbeat), _heartbeat_sub, &_heartbeat);
//    }
//}


//从参数设置中获得sysid
void FormationControl::param_update_system()
{
    _param_system_id = param_find("MAV_SYS_ID");
    _param_component_id = param_find("MAV_COMP_ID");

    /* update system and component id */
    int32_t system_id;
    param_get(_param_system_id, &system_id);

    int32_t component_id;
    param_get(_param_component_id, &component_id);

    if (system_id > 0 && system_id < 255) {
        mav_sysid = system_id;
    }

    if (component_id > 0 && component_id < 255) {
        mav_compid = component_id;
    }

}




//void
//FormationControl::enable_offboard_mode()
//{
////	mavlink_set_attitude_target_t set_attitude_target;
////	mavlink_msg_set_attitude_target_decode(msg, &set_attitude_target);

////	bool values_finite =
////		PX4_ISFINITE(set_attitude_target.q[0]) &&
////		PX4_ISFINITE(set_attitude_target.q[1]) &&
////		PX4_ISFINITE(set_attitude_target.q[2]) &&
////		PX4_ISFINITE(set_attitude_target.q[3]) &&
////		PX4_ISFINITE(set_attitude_target.thrust) &&
////		PX4_ISFINITE(set_attitude_target.body_roll_rate) &&
////		PX4_ISFINITE(set_attitude_target.body_pitch_rate) &&
////		PX4_ISFINITE(set_attitude_target.body_yaw_rate);

//    /* Only accept messages which are intended for this system */
////	if ((mavlink_system.sysid == set_attitude_target.target_system ||
////	     set_attitude_target.target_system == 0) &&
////	    (mavlink_system.compid == set_attitude_target.target_component ||
////	     set_attitude_target.target_component == 0) &&
////	    values_finite) {

////		/* set correct ignore flags for thrust field: copy from mavlink message */
////		_offboard_control_mode.ignore_thrust = (bool)(set_attitude_target.type_mask & (1 << 6));

////		/*
////		 * The tricky part in parsing this message is that the offboard sender *can* set attitude and thrust
////		 * using different messages. Eg.: First send set_attitude_target containing the attitude and ignore
////		 * bits set for everything else and then send set_attitude_target containing the thrust and ignore bits
////		 * set for everything else.
////		 */

//        /*
//         * if attitude or body rate have been used (not ignored) previously and this message only sends
//         * throttle and has the ignore bits set for attitude and rates don't change the flags for attitude and
//         * body rates to keep the controllers running
//         */
////		bool ignore_bodyrate_msg = (bool)(set_attitude_target.type_mask & 0x7);
////		bool ignore_attitude_msg = (bool)(set_attitude_target.type_mask & (1 << 7));

////		if (ignore_bodyrate_msg && ignore_attitude_msg && !_offboard_control_mode.ignore_thrust) {
////			/* Message want's us to ignore everything except thrust: only ignore if previously ignored */
////			_offboard_control_mode.ignore_bodyrate = ignore_bodyrate_msg && _offboard_control_mode.ignore_bodyrate;
////			_offboard_control_mode.ignore_attitude = ignore_attitude_msg && _offboard_control_mode.ignore_attitude;

////		} else {
////			_offboard_control_mode.ignore_bodyrate = ignore_bodyrate_msg;
////			_offboard_control_mode.ignore_attitude = ignore_attitude_msg;
////		}

//        _offboard_control_mode.ignore_attitude = true;
//        _offboard_control_mode.ignore_position = true;
//        _offboard_control_mode.ignore_velocity = true;
//        _offboard_control_mode.ignore_acceleration_force = true;

//        _offboard_control_mode.timestamp = hrt_absolute_time();

//        if (_offboard_control_mode_pub == nullptr) {
//            _offboard_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &_offboard_control_mode);

//        } else {
//            orb_publish(ORB_ID(offboard_control_mode), _offboard_control_mode_pub, &_offboard_control_mode);
//            PX4_INFO("推送set_offboard_publish");
//        }

////		/* If we are in offboard control mode and offboard control loop through is enabled
////		 * also publish the setpoint topic which is read by the controller */
////		if (_mavlink->get_forward_externalsp()) {
////			bool updated;
////			orb_check(_control_mode_sub, &updated);

////			if (updated) {
////				orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
////			}

////			if (_control_mode.flag_control_offboard_enabled) {

////				/* Publish attitude setpoint if attitude and thrust ignore bits are not set */
////				if (!(_offboard_control_mode.ignore_attitude)) {
////					vehicle_attitude_setpoint_s att_sp = {};
////					att_sp.timestamp = hrt_absolute_time();

////					if (!ignore_attitude_msg) { // only copy att sp if message contained new data
////						matrix::Quatf q(set_attitude_target.q);
////						q.copyTo(att_sp.q_d);
////						att_sp.q_d_valid = true;

////						matrix::Eulerf euler{q};
////						att_sp.roll_body = euler.phi();
////						att_sp.pitch_body = euler.theta();
////						att_sp.yaw_body = euler.psi();
////						att_sp.yaw_sp_move_rate = 0.0f;
////					}

////					if (!_offboard_control_mode.ignore_thrust) { // dont't overwrite thrust if it's invalid
////						att_sp.thrust = set_attitude_target.thrust;
////					}

////					if (_att_sp_pub == nullptr) {
////						_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

////					} else {
////						orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &att_sp);
////					}
////				}

////				/* Publish attitude rate setpoint if bodyrate and thrust ignore bits are not set */
////				///XXX add support for ignoring individual axes
////				if (!(_offboard_control_mode.ignore_bodyrate)) {
////					vehicle_rates_setpoint_s rates_sp = {};
////					rates_sp.timestamp = hrt_absolute_time();

////					if (!ignore_bodyrate_msg) { // only copy att rates sp if message contained new data
////						rates_sp.roll = set_attitude_target.body_roll_rate;
////						rates_sp.pitch = set_attitude_target.body_pitch_rate;
////						rates_sp.yaw = set_attitude_target.body_yaw_rate;
////					}

////					if (!_offboard_control_mode.ignore_thrust) { // dont't overwrite thrust if it's invalid
////						rates_sp.thrust = set_attitude_target.thrust;
////					}

////					if (_rates_sp_pub == nullptr) {
////						_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);

////					} else {
////						orb_publish(ORB_ID(vehicle_rates_setpoint), _rates_sp_pub, &rates_sp);
////					}
////				}
////			}

////		}
////	}
//}




void FormationControl::run()
{
    sleep(8); //这是为了在启动程序之前正常启动飞控,让飞控正常进入followme模式,如果不延时有可能进不了模式
    PX4_INFO("Formation Control Run!");

    //    int fw_pos_ctrl_status_sub_fd = orb_subscribe(ORB_ID(fw_pos_ctrl_status));
    //提取主机数据
    //    int vehicle_attitude_setpoint_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));

    //提取从机接收到的数据

    _att_sub                        = orb_subscribe(ORB_ID(vehicle_attitude));
    _formationrec_sub               = orb_subscribe(ORB_ID(formationrec));


    //    _set_offboard_sub               = orb_subscribe(ORB_ID(vehicle_command));
    _vehicle_status_sub             = orb_subscribe(ORB_ID(vehicle_status));
    _vehicle_global_position_sub    = orb_subscribe(ORB_ID(vehicle_global_position));
    _follow_target_sub              = orb_subscribe(ORB_ID(follow_target));
    _manual_control_setpoint_sub    = orb_subscribe(ORB_ID(manual_control_setpoint));
    _params_sub                     = orb_subscribe(ORB_ID(parameter_update));


    status_poll();  //在飞机循环之前,预先获得飞机状态
    formationrec_sp_poll();

    /* 1.公告attitude主题 */
    //    struct vehicle_attitude_setpoint_s att;
    //    memset(&att, 0, sizeof(att));

    /* 可以在此等待多个主题 */

    /* wakeup source */
    px4_pollfd_struct_t fds[2];

    /* Setup of loop */
    fds[0].fd = _vehicle_global_position_sub;
    fds[0].events = POLLIN;
    fds[1].fd = _formationrec_sub;
    fds[1].events = POLLIN;

    int error_counter = 0;

    orb_set_interval(fds[0].fd, 50);//限制更新频率为20 Hz

    PX4_INFO(" ");
    PX4_INFO("formation_control 执行!! ");

    PX4_INFO("formation_control should_exit?  %d",should_exit());




    while (!should_exit()) {








        status_poll();  //获取飞机状态


        //根据飞机ID判断当前飞机的属性和位置,然后执行对应操作
        //ID 1 是主机,执行位置/导航姿态/速度/速度设置点发送等
        //ID 其他值是从机,从机接收主机的数据,根据编队队形计算相应的控制指令,然后进入offboard模式执行



        if (sys_id <= 4 && sys_id >= 1){ //飞机编号正常

            if (sys_id == 1) {
                //            printf("当前飞机是主机 \n");
                //说明当前飞机是主机,执行主机相应操作
                int poll_sp = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100); /* 等待100ms获取数据 */


                /* 处理结果*/
                if (poll_sp == 0) {  //未得到数据
                    PX4_ERR("主机:在0.1秒内没有获得uorb vehicle_global_position 数据! ");
                } else if (poll_sp < 0) {  //严重错误
                    if (error_counter < 10 || error_counter % 50 == 0) {
                        /* use a counter to prevent flooding (and slowing us down) */
                        PX4_ERR("ERROR return value from poll(): %d", poll_sp);
                    }
                    error_counter++;
                } else {    //抓到数据
                    if ((fds[0].revents & POLLIN) != 0) {



                        //注意:飞机必须启用attitude控制模式及以上的模式,否则飞机内部ORB_ID(vehicle_attitude_setpoint)不会更新,本程序也不会运行

                        //                    struct vehicle_attitude_setpoint_s _mainuav_att;
                        //                    orb_copy(ORB_ID(vehicle_attitude_setpoint), vehicle_attitude_setpoint_sub_fd, &_mainuav_att);


                        struct vehicle_global_position_s P1_global_pos;
                        orb_copy(ORB_ID(vehicle_global_position), _vehicle_global_position_sub, &P1_global_pos);


                        //                   struct fw_pos_ctrl_status_s _mainuav_navatt;
                        //                   orb_copy(ORB_ID(fw_pos_ctrl_status), fw_pos_ctrl_status_sub_fd, &_mainuav_navatt);

                        //                主机姿态和主机速度是主要数据源,要以最高速率更新;主机位置跟着发送.
                        //                数据打包发送到uorb主题

                        //整理主机的实际位置/速度/偏航角,作为从机目标位置的依据
                        P1_send.lat = P1_global_pos.lat;
                        P1_send.lon = P1_global_pos.lon;
                        P1_send.alt = P1_global_pos.alt;  //获得主机海拔高度

                        P1_send.vx = P1_global_pos.vel_n;
                        P1_send.vy = P1_global_pos.vel_e;
                        P1_send.vz = P1_global_pos.vel_d;

                        P1_send.yaw_body = P1_global_pos.yaw;   //主机的偏航角也要发给从机,当主机地速很小时使用主机机头指向.

                        P1_send.timestamp = P1_global_pos.timestamp;  //传递主机数据的时间戳

                        //                    P1_send.timestamp = (uint64_t)123456;  //测试数据
                        printf("当前飞机是主机 P1_send.alt = %.1f  \n",(double)P1_send.alt);

                        //                    printf("当前飞机是主机 P1_send.lat = %.7f  \n",P1_send.lat);

                        //                   printf("当前飞机是主机 hrt_absolute_time() = %d  \n",hrt_absolute_time());
                        //                   printf("当前飞机是主机 P1_send.timestamp %d  \n",P1_send.timestamp);

                        formationx_sp_publish();  //将主机数据推送到主机的uorb总线上,然后由mavlink发送出去


                    }
                }
            } else if (check_aux_follow_sw()) { //当前是从机

                //说明当前飞机是从机,并且遥控器上的编队开关启用了,此时要根据从机编号做相应操作
                //不管是哪个从机,首先都要获取主机的数据
                int poll_rec = px4_poll(&fds[1], (sizeof(fds) / sizeof(fds[1])), 300); /* 等待1000ms获取数据 */

                poll_rec =1; //调试语句,注意删除

                /* 处理结果*/
                if (poll_rec == 0) {  //未得到数据
                    PX4_ERR("从机: 在0.3秒内没有获得 uorb formationrec 数据 ");
                } else if (poll_rec < 0) {  //严重错误
                    if (error_counter < 10 || error_counter % 20000 == 0) {
                        /* use a counter to prevent flooding (and slowing us down) */
                        PX4_ERR("从机: ERROR return value from poll(): %d  次数=%d", poll_rec ,error_counter);
                    }
                    error_counter++;
                } else {    //抓到数据
                    if (
                            true || //调试语句,注意删除
                            fds[1].revents & POLLIN) {

                        sleep(1);  //调试语句,注意删除

                        {//调试语句,注意删除
                            struct vehicle_attitude_s Px_att;
                            orb_copy(ORB_ID(vehicle_attitude), _att_sub, &Px_att);

                            matrix::Dcmf R = matrix::Quatf(Px_att.q);
                            matrix::Eulerf euler_angles(R);

                            PX4_INFO("(euler_angles.psi()) = %8.4f ",double(math::degrees(euler_angles.psi())));
                        }

                        struct map_projection_reference_s target_ref;
                        static formationrec_s  P1_with_offsetL{}; //根据从机相对主机的距离,计算出从机的目标位置
                        enable_follow_target_mode(check_aux_follow_sw()); //这一段预留:使从机进入follow_target模式  //试验时,程序和遥控器共同实现
                        formationrec_sp_poll();

                        //                        orb_copy(ORB_ID(formationrec), _formationrec_sub, &P1_received);
                        //                        PX4_INFO("P1_received.alt :\t%8.4f \t%d lat:\t%8.6f vx:\t%8.4f ",double(P1_received.alt),P1_received.timestamp,P1_received.lat,double(P1_received.vx));


                        /*这一段预留:增加编队变换的控制程序,要想办法写的简洁易懂*/
                        //输入,主机指令(或人工控制指令)
                        //根据当前队形编号和目标队形编号,分时间输出不同的编队队形编号,要包括过渡队形编号.
                        //输出,编队队形编号_form_shape_current

                        _form_shape_current = FORMATION_1;





                        /*这部分进行编队队形计算,计算出从机在编队中的相对位置*/
                        //输入:编队队形编号
                        //输出:从机在某一时刻相对于主机的距离向量 offset_L{L_along,L_cross}
                        float L_distant(10.0f);  //编队飞机之间的距离
                        Vector2f offset_L{L_distant,L_distant}; //从机相对主机的偏移距离向量,主机地轴航向作为x轴正方向,主机右侧是y正方向

                        switch (_form_shape_current) {
                        case FORMATION_1 :

                            offset_L = {-1.0f * L_distant * (sys_id-1) ,0.0f};

                            break;
                        case FORMATION_rhombus4 :

                            offset_L(0) = FORMATION_rhombus4_axis[sys_id][0] * L_distant;
                            offset_L(1) = FORMATION_rhombus4_axis[sys_id][1] * L_distant;

                            break;

                        }

                        /*这部分将从机相对位置(速度轴系)转换到从机相对位置(地轴系)*/ //主机被转换坐标轴的方向,向速度方向为x正,速度方向右边为y正,正北方向为地轴系x正,正东方向为地轴系y正
                        Vector2f  V1_trans{P1_received.vx,P1_received.vy};    //收到的主机的速度.单位 m/s
                        float cos_yaw_ned = V1_trans(0) / V1_trans.length();
                        float sin_yaw_ned = V1_trans(1) / V1_trans.length();
                        Vector2f offset_L_ned {offset_L(0) * cos_yaw_ned - offset_L(1) * sin_yaw_ned,
                                    offset_L(1) * cos_yaw_ned + offset_L(0) * sin_yaw_ned};


//                        PX4_INFO("offset_L(0) = %.1f ",double(offset_L(0)));



                        /*
                         * 这部分利用从机相对位置(地轴系)和预测的主机坐标,计算从机的目标坐标
                         * 注意:这里不考虑时间的流逝,时间修正放在整个流程的最后一步处理 */

                        map_projection_init(&target_ref,  P1_received.lat, P1_received.lon);
                        map_projection_reproject(&target_ref, offset_L_ned(0), offset_L_ned(1),&P1_with_offsetL.lat, &P1_with_offsetL.lon);


                        /*这部分将处理后的从机目标位置\速度发送给跟踪控制程序*/


                        _send_follow_target.timestamp=P1_received.timestamp;
                        //printf("_send_follow_target.timestamp :\t %.1f \n",1.0 * _send_follow_target.timestamp);


                        //从机将获得的位置数据传递到follow数据

                        _send_follow_target.lat=P1_with_offsetL.lat;
                        _send_follow_target.lon=P1_with_offsetL.lon;
                        _send_follow_target.alt=P1_received.alt; //从机要高度减小保安全


                        //假设速度在短时间内不会有明显变化 直接赋值
                        _send_follow_target.vx=P1_received.vx;
                        _send_follow_target.vy=P1_received.vy;
                        _send_follow_target.vz=P1_received.vz;

                        _send_follow_target.yaw_body=P1_received.yaw_body;



                        //                        PX4_INFO("_send_ftarget.alt :\t%8.4f \t%d lat:\t%8.6f vx:\t%8.4f "
                        //                                 ,double(_send_follow_target.alt),_send_follow_target.timestamp,_send_follow_target.lat,double(_send_follow_target.vx));


                        send_follow_target_publish();



                    } else{
                        PX4_INFO("fds[1].revents & POLLIN 异常");
                        sleep(1);
                    }
                }
            } else {
                PX4_INFO("从机未进入followme模式");
                sleep(1);

            }


        } else {//注意这里要报错
            if(status_valid)  //当程序poll到的status数据有效时,才根据这个数据执行程序,避免初始值对程序造成不必要影响
                PX4_ERR("sys_id = %d 从机status.system_id编号错误,不能继续执行,程序目前最多支持4机编队,请修改status.system_id为1~4之间的值!",sys_id);
        }
    }
    PX4_INFO("exiting");
}




FormationControl *FormationControl::instantiate(int argc, char *argv[])
{
        return new FormationControl();
}

int FormationControl::task_spawn(int argc, char *argv[])
{
        _task_id = px4_task_spawn_cmd("formation_control",
				      SCHED_DEFAULT,
                      SCHED_PRIORITY_DEFAULT,
				      1500,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

int FormationControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FormationControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
formation_control is the ........

)DESCR_STR");

	PRINT_MODULE_USAGE_COMMAND("start");

        PRINT_MODULE_USAGE_NAME("formation_control", "controller");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int FormationControl::print_status()
{
	PX4_INFO("Running");

	// perf?

	return 0;
}

int formation_control_main(int argc, char *argv[])
{
        return FormationControl::main(argc, argv);
}
