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

FormationControl::FormationControl()
{

}

FormationControl::~FormationControl()
{

}








void
FormationControl::formationx_sp_poll()
{
    /* check if there is a new setpoint */
    bool formationrec_updated;
    orb_check(_formationrec_sub, &formationrec_updated);

    if (formationrec_updated) {
        orb_copy(ORB_ID(formationrec), _formationrec_sub, &_formationrec);
    }
}





void
FormationControl::formationx_sp_publish()
{

    if (_mainuavformcmdorb_pub != nullptr) {
        /* publish the attitude rates setpoint */
        orb_publish(ORB_ID(formationx), _mainuavformcmdorb_pub, &_mainuavformcmdorb);
            PX4_INFO("推送formationx_sp_publish");

    } else {
        /* advertise the attitude rates setpoint */
        _mainuavformcmdorb_pub = orb_advertise(ORB_ID(formationx), &_mainuavformcmdorb);
    }
}



void FormationControl::run()
{
    PX4_INFO("Formation Control Run!");

    /* 订阅 主题*/
    int vehicle_attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
//    int vehicle_attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
    orb_set_interval(vehicle_attitude_sub_fd, 1000);//限制更新频率为1 Hz

    /* 1.公告attitude主题 */
    struct vehicle_attitude_s att;
    memset(&att, 0, sizeof(att));


    /* 可以在此等待多个主题 */

    /* wakeup source */
    px4_pollfd_struct_t fds[1];

    /* Setup of loop */
    fds[0].fd = vehicle_attitude_sub_fd;
    fds[0].events = POLLIN;


    int error_counter = 0;

    _formationrec_sub = orb_subscribe(ORB_ID(formationrec));


    while (!should_exit()) {


        /* 等待1000ms获取数据 */
        int poll_ret = px4_poll(fds, 1, 1000);

        /* 处理结果*/
        if (poll_ret == 0)  //未得到数据
        {
            PX4_ERR("Got no data within 1 second");
        }
        else if (poll_ret < 0)  //严重错误
        {
            if (error_counter < 10 || error_counter % 50 == 0)
            {
                /* use a counter to prevent flooding (and slowing us down) */
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }
            error_counter++;
        }
        else    //抓到数据
        {
            if (fds[0].revents & POLLIN)
            {

                struct vehicle_attitude_s _mainuav_att;
                orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub_fd, &_mainuav_att);

//                主机姿态和主机速度是主要数据源,要以最高速率更新;主机位置跟着发送.
//                数据打包发送到uorb主题



                _mainuavformcmdorb.lat = 0.0;//_mainuav_att.lat;
                _mainuavformcmdorb.lon = 0.0;//_mainuav_att.lon;
                _mainuavformcmdorb.alt = 0.0;//_mainuav_att.alt;



                _mainuavformcmdorb.alt = hrt_absolute_time();  //测试数据

                formationx_sp_publish();

           //            mavlink_log_critical(&_mavlink_log_pub, "test = %.1f/n", double(_mainuavformcmdorb.alt));
                sleep(0.5);

                formationx_sp_poll();

//                mavlink_log_critical(&_mavlink_log_pub, "test = %9.4f/n", double(_formationrec.alt_rec));
                 PX4_INFO("receiver :\t%8.4f",double(_formationrec.alt));




            }

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
