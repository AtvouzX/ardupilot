#include "AP_Gripper.h"

#if AP_GRIPPER_ENABLED

#include "AP_Gripper_Servo.h"
#include "AP_Gripper_EPM.h"
#include "AP_Gripper_BTS7960.h"

extern const AP_HAL::HAL& hal;

#define GRIPPER_GRAB_PWM_DEFAULT        1900
#define GRIPPER_RELEASE_PWM_DEFAULT     1100
// EPM PWM definitions
#define GRIPPER_NEUTRAL_PWM_DEFAULT     1500
#define GRIPPER_REGRAB_DEFAULT          0           // default EPM re-grab interval (in seconds) to ensure cargo is securely held
#define GRIPPER_AUTOCLOSE_DEFAULT       0.0         // default automatic close time (in seconds)

const AP_Param::GroupInfo AP_Gripper::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Gripper Enable/Disable
    // @Description: Gripper enable/disable
    // @User: Standard
    // @Values: 0:Disabled, 1:Enabled
    AP_GROUPINFO_FLAGS("ENABLE", 0, AP_Gripper, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: TYPE
    // @DisplayName: Gripper Type
    // @Description: Gripper enable/disable
    // @User: Standard
    // @Values: 0:None,1:Servo,2:EPM,3:BTS7960
    AP_GROUPINFO("TYPE", 1, AP_Gripper, config.type, 0),

    // @Param: GRAB
    // @DisplayName: Gripper Grab PWM
    // @Description: PWM value in microseconds sent to Gripper to initiate grabbing the cargo
    // @User: Advanced
    // @Range: 1000 2000
    // @Units: PWM
    AP_GROUPINFO("GRAB",    2, AP_Gripper, config.grab_pwm, GRIPPER_GRAB_PWM_DEFAULT),

    // @Param: RELEASE
    // @DisplayName: Gripper Release PWM
    // @Description: PWM value in microseconds sent to Gripper to release the cargo
    // @User: Advanced
    // @Range: 1000 2000
    // @Units: PWM
    AP_GROUPINFO("RELEASE", 3, AP_Gripper, config.release_pwm, GRIPPER_RELEASE_PWM_DEFAULT),

    // @Param: NEUTRAL
    // @DisplayName: Neutral PWM
    // @Description: PWM value in microseconds sent to grabber when not grabbing or releasing
    // @User: Advanced
    // @Range: 1000 2000
    // @Units: PWM
    AP_GROUPINFO("NEUTRAL", 4, AP_Gripper, config.neutral_pwm, GRIPPER_NEUTRAL_PWM_DEFAULT),

    // @Param: REGRAB
    // @DisplayName: EPM Gripper Regrab interval
    // @Description: Time in seconds that EPM gripper will regrab the cargo to ensure grip has not weakened; 0 to disable
    // @User: Advanced
    // @Range: 0 255
    // @Units: s
    AP_GROUPINFO("REGRAB",  5, AP_Gripper, config.regrab_interval, GRIPPER_REGRAB_DEFAULT),

    // @Param: CAN_ID
    // @DisplayName: EPM UAVCAN Hardpoint ID
    // @Description: Refer to https://docs.zubax.com/opengrab_epm_v3#UAVCAN_interface
    // @User: Standard
    // @Range: 0 255
    AP_GROUPINFO("CAN_ID", 6, AP_Gripper, config.uavcan_hardpoint_id, 0),

    // @Param: AUTOCLOSE
    // @DisplayName: Gripper Autoclose time
    // @Description: Time in seconds that gripper close the gripper after opening; 0 to disable
    // @User: Advanced
    // @Range: 0.25 255
    // @Units: s
    AP_GROUPINFO("AUTOCLOSE",  7, AP_Gripper, config.autoclose_time, GRIPPER_AUTOCLOSE_DEFAULT),

    // @Param: DC_PWM_FREQ
    // @DisplayName: DC Gripper PWM Frequency
    // @Description: PWM frequency in Hz used on the BTS7960 DC gripper outputs. The pulse width in microseconds is interpreted as a duty cycle of the resulting period, so set SERVOx_MIN to 0 and SERVOx_MAX to the period in microseconds (1000 at 1000 Hz) on the claw channels
    // @User: Advanced
    // @Range: 100 5000
    // @Units: Hz
    // @RebootRequired: True
    AP_GROUPINFO("DC_PWM_FREQ", 8, AP_Gripper, config.dc_pwm_freq, 1000),

    // @Param: DC_PWM_MAX
    // @DisplayName: DC Gripper Maximum Duty Cycle
    // @Description: Maximum PWM duty cycle in percent applied to the BTS7960 while the claw is moving
    // @User: Advanced
    // @Range: 10 100
    // @Units: %
    AP_GROUPINFO("DC_PWM_MAX", 9, AP_Gripper, config.dc_pwm_max, 100),

    // @Param: DC_RAMP_MS
    // @DisplayName: DC Gripper Soft Start Ramp
    // @Description: Time in milliseconds over which the duty cycle ramps from zero to DC_PWM_MAX after a move starts; 0 for an immediate full duty cycle
    // @User: Advanced
    // @Range: 0 2000
    // @Units: ms
    AP_GROUPINFO("DC_RAMP_MS", 10, AP_Gripper, config.dc_ramp_ms, 300),

    // @Param: DC_STALL_MA
    // @DisplayName: DC Gripper Stall Current
    // @Description: Motor current in milliamps above which the claw is considered stalled (end of travel or object grasped), as measured by the INA219. Set to 0 to disable current-based detection entirely; the INA219 is then not required and only the movement timeout applies
    // @User: Advanced
    // @Range: 0 20000
    // @Units: mA
    AP_GROUPINFO("DC_STALL_MA", 11, AP_Gripper, config.dc_stall_ma, 2000),

    // @Param: DC_STALL_MS
    // @DisplayName: DC Gripper Stall Time
    // @Description: Time in milliseconds the motor current must stay continuously above DC_STALL_MA before a stall is declared
    // @User: Advanced
    // @Range: 20 2000
    // @Units: ms
    AP_GROUPINFO("DC_STALL_MS", 12, AP_Gripper, config.dc_stall_ms, 200),

    // @Param: DC_BLANK_MS
    // @DisplayName: DC Gripper Current Blanking Time
    // @Description: Time in milliseconds after a move starts (or reverses) during which the motor current is ignored, to mask the start-up inrush current
    // @User: Advanced
    // @Range: 0 2000
    // @Units: ms
    AP_GROUPINFO("DC_BLANK_MS", 13, AP_Gripper, config.dc_blank_ms, 300),

    // @Param: DC_OVRC_MA
    // @DisplayName: DC Gripper Overcurrent Limit
    // @Description: Hard motor current limit in milliamps; exceeding it (after the blanking time) stops the claw immediately without waiting for DC_STALL_MS. Also used as the INA219 full-scale design current. The INA219 can only measure up to 320 mV across the shunt, so the maximum measurable current is 320 mV / DC_SHUNT (3200 mA with the stock 100 mOhm shunt); keep this limit below that, or fit a 0.05 Ohm shunt for a 6.4 A range and raise this limit via the parameter without rebuilding. Set to 0 to disable
    // @User: Advanced
    // @Range: 0 30000
    // @Units: mA
    AP_GROUPINFO("DC_OVRC_MA", 14, AP_Gripper, config.dc_ovrc_ma, 3000),

    // @Param: DC_TOUT_S
    // @DisplayName: DC Gripper Movement Timeout
    // @Description: Maximum duration in seconds of a single open or close movement. A stall detected in the last 20 percent of this time is treated as the end of travel; a movement that reaches the timeout without a stall is stopped and flagged as a fault
    // @User: Advanced
    // @Range: 1 60
    // @Units: s
    AP_GROUPINFO("DC_TOUT_S", 15, AP_Gripper, config.dc_tout_s, 10),

    // @Param: DC_HOLD_PCT
    // @DisplayName: DC Gripper Holding Duty Cycle
    // @Description: Duty cycle in percent kept applied in the closing direction after an object has been grasped, to hold the grip. Set to 0 to simply stop (and brake, if DC_BRAKE is 1) after a grasp
    // @User: Advanced
    // @Range: 0 100
    // @Units: %
    AP_GROUPINFO("DC_HOLD_PCT", 16, AP_Gripper, config.dc_hold_pct, 0),

    // @Param: DC_BRAKE
    // @DisplayName: DC Gripper Brake On Stop
    // @Description: Behaviour of the BTS7960 when the claw stops: 0 to coast (driver disabled), 1 to brake (driver enabled with both direction inputs at zero)
    // @User: Advanced
    // @Values: 0:Coast,1:Brake
    AP_GROUPINFO("DC_BRAKE", 17, AP_Gripper, config.dc_brake, 1),

    // @Param: DC_I2C_BUS
    // @DisplayName: DC Gripper INA219 I2C Bus
    // @Description: I2C bus number the INA219 claw current sensor is connected to (the external I2C port on most boards)
    // @User: Advanced
    // @Range: 0 3
    // @RebootRequired: True
    AP_GROUPINFO("DC_I2C_BUS", 18, AP_Gripper, config.dc_i2c_bus, 1),

    // @Param: DC_I2C_ADDR
    // @DisplayName: DC Gripper INA219 I2C Address
    // @Description: I2C address of the INA219 claw current sensor, 64 (0x40) by default, as set by the A0/A1 pins of the sensor
    // @User: Advanced
    // @Range: 64 79
    // @RebootRequired: True
    AP_GROUPINFO("DC_I2C_ADDR", 19, AP_Gripper, config.dc_i2c_addr, 64),

    // @Param: DC_SHUNT
    // @DisplayName: DC Gripper INA219 Shunt Resistor
    // @Description: Value in milliohms of the INA219 shunt resistor (100 milliohms on most INA219 breakout boards)
    // @User: Advanced
    // @Range: 1 1000
    // @RebootRequired: True
    AP_GROUPINFO("DC_SHUNT", 20, AP_Gripper, config.dc_shunt, 100),

    // @Param: DC_CURR
    // @DisplayName: DC Gripper Motor Current
    // @Description: Live claw motor current in milliamps as measured by the INA219; updated by the BTS7960 backend while the claw is moving and intended for stall threshold calibration. Read-only, do not set
    // @User: Advanced
    // @Units: mA
    AP_GROUPINFO("DC_CURR", 21, AP_Gripper, config.dc_curr, 0),

    AP_GROUPEND
};

AP_Gripper::AP_Gripper()
{
    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many grippers");
#endif
        return;
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

/*
 * Get the AP_Gripper singleton
 */
AP_Gripper *AP_Gripper::_singleton = nullptr;
AP_Gripper *AP_Gripper::get_singleton()
{
    return _singleton;
}

void AP_Gripper::init()
{
    // return immediately if not enabled
    if (!_enabled.get()) {
        return;
    }

    switch(config.type.get()) {
    case 0:
        break;
#if AP_GRIPPER_SERVO_ENABLED
    case 1:
        backend = NEW_NOTHROW AP_Gripper_Servo(config);
        break;
#endif
#if AP_GRIPPER_EPM_ENABLED
    case 2:
        backend = NEW_NOTHROW AP_Gripper_EPM(config);
        break;
#endif
#if AP_GRIPPER_BTS7960_ENABLED
    case 3:
        backend = NEW_NOTHROW AP_Gripper_BTS7960(config);
        break;
#endif
    default:
        break;
    }
    if (backend != nullptr) {
        backend->init();
    }
}

// update - should be called at at least 10hz
#define PASS_TO_BACKEND(function_name) \
    void AP_Gripper::function_name()   \
    {                                  \
        if (!enabled()) {              \
            return;                    \
        }                              \
        if (backend != nullptr) {      \
            backend->function_name();  \
        }                              \
    }

PASS_TO_BACKEND(grab)
PASS_TO_BACKEND(release)
PASS_TO_BACKEND(stop)
PASS_TO_BACKEND(update)

#undef PASS_TO_BACKEND


#define PASS_TO_BACKEND(function_name)        \
    bool AP_Gripper::function_name() const    \
    {                                         \
        if (!enabled()) {                     \
            return false;                     \
        }                                     \
        if (backend != nullptr) {             \
            return backend->function_name();  \
        }                                     \
        return false;                         \
    }

PASS_TO_BACKEND(valid)
PASS_TO_BACKEND(released)
PASS_TO_BACKEND(grabbed)

#undef PASS_TO_BACKEND

namespace AP {

AP_Gripper &gripper()
{
    return *AP_Gripper::get_singleton();
}

};

#endif  // AP_GRIPPER_ENABLED
