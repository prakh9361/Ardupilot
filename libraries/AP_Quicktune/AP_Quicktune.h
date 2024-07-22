#pragma once


#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/AP_HAL_Boards.h>
#ifndef QUICKTUNE_ENABLED
 #define QUICKTUNE_ENABLED 1 //APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#endif

#if QUICKTUNE_ENABLED

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AP_AHRS/AP_AHRS.h>
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS.h>

#define UPDATE_RATE_HZ 40
#define UPDATE_PERIOD_MS (1000U/UPDATE_RATE_HZ)
#define STAGE_DELAY 4000
#define PILOT_INPUT_DELAY 4000
#define YAW_FLTE_MAX 2.0
#define FLTD_MUL 0.5
#define FLTT_MUL 0.5
#define DEFAULT_SMAX 50.0

#define OPTIONS_TWO_POSITION (1<<0)

class AP_Quicktune {
public:
    AP_Quicktune()
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    // Empty destructor to suppress compiler warning
    virtual ~AP_Quicktune() {}

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Quicktune);

    // parameter block
    static const struct AP_Param::GroupInfo var_info[];

    void update();
    void update_switch_pos(const  RC_Channel::AuxSwitchPos ch_flag);

// private:

    // parameters
    AP_Float enable;
    AP_Int8 axes_enabled;
    AP_Float double_time;
    AP_Float gain_margin;
    AP_Float osc_smax;
    AP_Float yaw_p_max;
    AP_Float yaw_d_max;
    AP_Float rp_pi_ratio;
    AP_Float y_pi_ratio;
    AP_Int8 auto_filter;
    AP_Float auto_save;
    AP_Float max_reduce;
    AP_Int16 options;

    //Low, Mid and High must be in the same positions as they are in RC_Channel::AuxSwitchPos
    enum class qt_switch_pos : uint8_t {
        LOW,
        MID,
        HIGH,
        NONE,
    };


    enum class axis_names : uint8_t {
        RLL,
        PIT,
        YAW,
        DONE,
        END,
    };

    enum class param_s : uint8_t {
        RLL_P,
        RLL_I,
        RLL_D,
        RLL_SMAX,
        RLL_FLTT,
        RLL_FLTD,
        RLL_FLTE,
        RLL_FF,
        PIT_P,
        PIT_I,
        PIT_D,
        PIT_SMAX,
        PIT_FLTT,
        PIT_FLTD,
        PIT_FLTE,
        PIT_FF,
        YAW_P,
        YAW_I,
        YAW_D,
        YAW_SMAX,
        YAW_FLTT,
        YAW_FLTD,
        YAW_FLTE,
        YAW_FF,
        END,
    };

    //Also the gains
    enum class stages : uint8_t {
        D,
        P,
        DONE,
        I,
        FF,
        SMAX,
        FLTT,
        FLTD,
        FLTE,
        END,
    };

    uint32_t last_stage_change;
    uint32_t last_gain_report = AP_HAL::millis();
    uint32_t last_pilot_input = AP_HAL::millis();
    uint32_t last_warning = AP_HAL::millis();
    uint32_t tune_done_time;

    stages current_stage = stages::D;
    param_s slew_parm = param_s::END;
    float slew_target;
    uint8_t slew_steps;
    float slew_delta;
    uint32_t last_update;
    qt_switch_pos sw_pos; //Switch pos to be set by aux func.

    uint32_t axes_done;
    uint32_t filters_done;

    //Bitmask of changed parameters
    uint32_t param_changed;

    //Saved values of the parameter
    float param_saved[uint8_t(param_s::END)];

    bool need_restore;

    void reset_axes_done();
    void setup_filters(axis_names axis);
    bool have_pilot_input();
    axis_names get_current_axis();
    float get_slew_rate(axis_names axis);
    void advance_stage(axis_names axis);
    void adjust_gain(param_s param, float value);
    void adjust_gain_limited(param_s param, float value);
    float get_gain_mul();
    void restore_all_params();
    void save_all_params();
    bool item_in_bitmask(uint8_t item, uint32_t bitmask);
    param_s get_pname(axis_names axis, stages stage);
    AP_Float *get_param_pointer(param_s param);
    float get_param_value(param_s param);
    void set_param_value(param_s param, float value);
    void set_and_save_param_value(param_s param, float value);
    float gain_limit(param_s param);
    axis_names get_axis(param_s param);
    float limit_gain(param_s param, float value);
    const char* get_param_name(param_s param);
    void set_bitmask(bool value, uint32_t &bitmask, uint8_t position);
    stages get_stage(param_s param);
    const char* get_axis_name(axis_names axis);
    void write_quik(float SRate, float Gain, param_s param);

    AC_AttitudeControl &attitude_control = *AC_AttitudeControl::get_singleton();
};

#endif  // QUICKTUNE_ENABLED
