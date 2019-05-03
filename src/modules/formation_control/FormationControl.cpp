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




void
FormationControl::formationx_sp_poll()
{
    /* check if there is a new setpoint */
    bool formationrec_updated;
    orb_check(_formationrec_sub, &formationrec_updated);

    if (formationrec_updated) {
        orb_copy(ORB_ID(formationrec), _formationrec_sub, &_formationrec);
//printf("收到主机编队信息....._formationrec.lat  :  %.7f \n", (double)_formationrec.lat);
//printf("_formationrec.alt :\t%8.4f  \n",double(_formationrec.alt));

    }






}


void
FormationControl::status_poll()
{
    bool _updated;
    orb_check(_vehicle_status_sub, &_updated);
    if (_updated) {
          orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &status);  //从飞行器状态中获得sysid
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

    if (_mainuav_sp_pub != nullptr) {
        /* publish the attitude rates setpoint */
        orb_publish(ORB_ID(formationx), _mainuav_sp_pub, &_mainuav_sp);
//            PX4_INFO("推送formationx_sp_publish");

    } else {
        /* advertise the attitude rates setpoint */
        _mainuav_sp_pub = orb_advertise(ORB_ID(formationx), &_mainuav_sp);
    }
}



//从机向本机uorb总线发送follow_target的目标位置和速度
void
FormationControl::send_follow_target_publish()
{

    //这一段预留:要增加对从机相对于主机位置的控制





    if (_send_follow_target_pub != nullptr) {
        /* publish the attitude rates setpoint */
        orb_publish(ORB_ID(follow_target), _send_follow_target_pub, &_send_follow_target);
//printf("当前是从机,推送followme sp \n");

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
        status_poll();  //获取飞机状态
        _command.command            = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;//设置模式命令id
        _command.param1             = 1.0f;
        _command.param2             = 4.0f;
        _command.param3             = 8.0f;
        _command.target_system      = status.system_id;
        _command.target_component   = status.component_id;

        if (_vehicle_command_pub != nullptr) {
            orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &_command);//发布这个命令
        } else {
            _vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &_command);
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


    while (!should_exit()) {

//        manual_control_setpoint_poll();  //获取遥控器指令
        status_poll();  //获取飞机状态

//        /* 当参数改变时更新参数 */
//        bool params_updated = false;
//        orb_check(_params_sub, &params_updated);

//        if (params_updated) {
//            /* read from param to clear updated flag */
//            parameter_update_s update;
//            orb_copy(ORB_ID(parameter_update), _params_sub, &update);

//            /* update parameters from storage */
//            param_update_system();
//        }

        //根据飞机ID判断当前飞机的属性和位置,然后执行对应操作
        //ID 1 是主机,执行位置/导航姿态/速度/速度设置点发送等
        //ID 其他值是从机,从机接收主机的数据,根据编队队形计算相应的控制指令,然后进入offboard模式执行


        if (status.system_id == 1) {
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


                    attitude_poll();



                    struct vehicle_global_position_s _mainuav_pos;
                    orb_copy(ORB_ID(vehicle_global_position), _vehicle_global_position_sub, &_mainuav_pos);


                    //                   struct fw_pos_ctrl_status_s _mainuav_navatt;
                    //                   orb_copy(ORB_ID(fw_pos_ctrl_status), fw_pos_ctrl_status_sub_fd, &_mainuav_navatt);

                    //                主机姿态和主机速度是主要数据源,要以最高速率更新;主机位置跟着发送.
                    //                数据打包发送到uorb主题

                    //整理主机的实际位置,作为从机目标位置的依据
                    _mainuav_sp.lat = _mainuav_pos.lat;
                    _mainuav_sp.lon = _mainuav_pos.lon;
                    _mainuav_sp.alt = _mainuav_pos.alt;
                    _mainuav_sp.timestamp = hrt_absolute_time();  //测试数据

//                    _mainuav_sp.timestamp = (uint64_t)123456;  //测试数据
printf("当前飞机是主机 _mainuav_sp.alt = %.1f  \n",(double)_mainuav_sp.alt);
                    //整理主机的姿态设置点,作为从机的姿态设置点
//                    _mainuav_sp.roll_body  = _mainuav_att.rollspeed;
//                    _mainuav_sp.pitch_body = _mainuav_att.pitchspeed;
//                    _mainuav_sp.yaw_body   = _mainuav_att.yawspeed;

//                    printf("当前飞机是主机 _mainuav_sp.lat = %.7f  \n",_mainuav_sp.lat);

//                   printf("当前飞机是主机 hrt_absolute_time() = %d  \n",hrt_absolute_time());
//                   printf("当前飞机是主机 _mainuav_sp.timestamp %d  \n",_mainuav_sp.timestamp);

                      formationx_sp_publish();  //将主机数据推送到主机的uorb总线上,然后由mavlink发送出去


                }
            }
        } else if (check_aux_follow_sw()) {
//             printf("当前是从机 \n");

//            formationx_sp_poll();


//            sleep(0.5f);

            //说明当前飞机是从机,并且遥控器上的编队开关启用了,此时要根据从机编号做相应操作
            //不管是哪个从机,首先都要获取主机的数据
            int poll_rec = px4_poll(&fds[1], (sizeof(fds) / sizeof(fds[1])), 300); /* 等待1000ms获取数据 */

//            orb_copy(ORB_ID(formationx), _formationrec_sub, &_formationrec);
//            printf("_formationrec.alt :\t%8.4f \n",double(_formationrec.alt));

            /* 处理结果*/
            if (poll_rec == 0) {  //未得到数据
                PX4_ERR("从机: 在0.3秒内没有获得 uorb formationrec 数据 ");
            } else if (poll_rec < 0) {  //严重错误
                if (error_counter < 10 || error_counter % 50 == 0) {
                    /* use a counter to prevent flooding (and slowing us down) */
                    PX4_ERR("ERROR return value from poll(): %d", poll_rec);
                }
                error_counter++;
            } else {    //抓到数据
                if (fds[1].revents & POLLIN) {
                    //这一段预留:使从机进入follow_target模式  //试验时,程序和遥控器共同实现
                    enable_follow_target_mode(check_aux_follow_sw());





                    //将主机实际位置作为从机跟踪的目标位置,后续要加偏移值


                    formationx_sp_poll();


                    _send_follow_target.timestamp=_formationrec.timestamp;
//printf("_send_follow_target.timestamp :\t %.1f \n",1.0 * _send_follow_target.timestamp);


                    //从机将获得的位置数据传递到follow数据
                    _send_follow_target.alt=_formationrec.alt-8.0f; //从机高度减小保安全
                    _send_follow_target.lat=_formationrec.lat;
                    _send_follow_target.lon=_formationrec.lon;


                    //下面这部分暂时没用
                    _send_follow_target.vx=_formationrec.vx;
                    _send_follow_target.vy=_formationrec.vy;
                    _send_follow_target.vz=_formationrec.vz;




                    //这一段预留:增加编队变换的控制程序,要想办法写的简洁易懂
                    //下面这段预留:根据计算的各种位置向offboard模式的从机发送指令
                    switch (mav_sysid) {
                    case 2 :   //2号机的程序
                        //设定2号机跟踪的位置是主机右侧10米

                        //            mavlink_log_critical(&_mavlink_log_pub, "test = %.1f/n", double(_mainuav_sp.alt));

                        //                        formationx_sp_poll();

                        //                mavlink_log_critical(&_mavlink_log_pub, "test = %9.4f/n", double(_formationrec.alt_rec));
                        //                        PX4_INFO("receiver :\t%8.4f",double(_formationrec.alt));


                        break;
                    case 3 :   //3号机的程序

                        break;

                    case 4 :   //4号机的程序

                        break;

                    }

                    send_follow_target_publish();


                }
            }
        } else {
            printf("从机未进入followme模式 \n");
            sleep(1.0f);
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
