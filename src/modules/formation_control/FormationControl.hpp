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

#include <px4_module.h>
//#include <drivers/drv_hrt.h>


#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/formationrec.h>
#include <uORB/topics/formationx.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/fw_pos_ctrl_status.h>
//#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/follow_target.h>



#include <vtol_att_control/vtol_type.h>
#include <systemlib/mavlink_log.h>

#include <uORB/topics/sensor_combined.h>

//#include "mavlink_orb_subscription.h"



using uORB::Subscription;

class FormationControl final : public ModuleBase<FormationControl>
{
public:
        FormationControl();
        ~FormationControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
        static FormationControl *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

    int     _att_sub{-1};
    int     _formationrec_sub{-1};
    int		_params_sub{-1};			/**< notification of parameter updates */
    int		_set_offboard_sub{-1};
    int		_vehicle_status_sub{-1};
    int	 _vehicle_global_position_sub{-1};
    int	 _follow_target_sub{-1};

    int     _manual_control_setpoint_sub{-1};



    int mav_sysid{-1};
    int mav_compid{-1};

    orb_advert_t    _mainuav_sp_pub{nullptr};			/**<  */
    orb_advert_t	_mavlink_log_pub{nullptr};
//    orb_advert_t    _offboard_control_mode_pub{nullptr};
    orb_advert_t    _vehicle_command_pub{nullptr};

    orb_advert_t    _send_follow_target_pub{nullptr};

    orb_id_t _mainuav_sp_id{nullptr};

    formationx_s                _mainuav_sp{}; /*自定义的编队控制结构体 */
    formationrec_s              _formationrec{}; /*自定义的编队控制结构体 */
    vehicle_command_s           _set_offboard{}; /*自定义的编队控制结构体 */
//    offboard_control_mode_s     _offboard_control_mode{};
    vehicle_command_s           _command{};
    vehicle_status_s            status{};
    follow_target_s             _send_follow_target{}; /*自定义的编队控制结构体 */
    manual_control_setpoint_s   _manual_sp{};

    vehicle_attitude_s   _mainuav_att{};

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_nonfinite_input_perf;		/**< performance counter for non finite input */
	perf_counter_t	_nonfinite_output_perf;		/**< performance counter for non finite output */

    bool			_idparam_initialized;
    param_t			_param_system_id;
    param_t			_param_component_id;

    void        formationx_sp_poll();
    void        status_poll();

    void        attitude_poll();
    void        formationx_sp_publish();
    void        hold_offboard_status();
    void        enable_offboard_mode(bool offboard_sw);
    void        param_update_system();
    void        send_follow_target_publish();
    void        enable_follow_target_mode(bool follow_target_enabled);
    void        manual_control_setpoint_poll();

    bool        check_aux_follow_sw();

};
