#include "AP_Gripper_BTS7960.h"

#if AP_GRIPPER_BTS7960_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

uint16_t AP_Gripper_BTS7960::period_us() const
{
    // use the frequency the outputs actually run at; the HAL may clamp
    // the requested frequency (ChibiOS clamps normal PWM groups to 400Hz)
    if (_pwm_period_us != 0) {
        return _pwm_period_us;
    }
    const int16_t freq = MIN(constrain_int16(config.dc_pwm_freq, 100, 5000), int16_t(400));
    return uint16_t(1000000 / freq);
}

void AP_Gripper_BTS7960::write_outputs(uint16_t fwd_pwm, uint16_t rev_pwm, bool enable)
{
    if (!_have_channels) {
        return;
    }
    SRV_Channels::set_output_pwm_chan(_chan_fwd, fwd_pwm);
    SRV_Channels::set_output_pwm_chan(_chan_rev, rev_pwm);
    if (_chan_en != UINT8_MAX) {
        SRV_Channels::set_output_pwm_chan(_chan_en, enable ? period_us() : 0);
    }
}

void AP_Gripper_BTS7960::write_drive(bool closing, float duty_pct)
{
    const uint16_t pwm = uint16_t(period_us() * constrain_float(duty_pct, 0.0f, 100.0f) / 100.0f);
    if (closing) {
        write_outputs(pwm, 0, true);
    } else {
        write_outputs(0, pwm, true);
    }
}

void AP_Gripper_BTS7960::stop_motor()
{
    // both direction inputs at zero; with the driver kept enabled
    // (DC_BRAKE = 1) the BTS7960 brakes, with it disabled it coasts
    write_outputs(0, 0, config.dc_brake != 0);
}

void AP_Gripper_BTS7960::enter_fault(const char *reason)
{
    stop_motor();
    _state = ClawState::FAULT;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Claw: fault - %s", reason);
}

bool AP_Gripper_BTS7960::read_register(uint8_t reg, uint16_t &value) const
{
    uint8_t b[2];
    if (!_dev->read_registers(reg, b, sizeof(b))) {
        return false;
    }
    value = (uint16_t(b[0]) << 8) | b[1];
    return true;
}

bool AP_Gripper_BTS7960::write_register(uint8_t reg, uint16_t value) const
{
    const uint8_t b[3] { reg, uint8_t(value >> 8), uint8_t(value & 0xff) };
    return _dev->transfer(b, sizeof(b), nullptr, 0);
}

bool AP_Gripper_BTS7960::ina219_init()
{
    _dev = hal.i2c_mgr->get_device_ptr(config.dc_i2c_bus, config.dc_i2c_addr, 100000, false, 20);
    if (_dev == nullptr) {
        return false;
    }

    // full-scale current: the overcurrent limit, clamped to what the
    // INA219 shunt voltage range (max 320 mV) can measure with this shunt
    const float shunt_ohm = (config.dc_shunt > 0 ? config.dc_shunt : 100) * 0.001f;
    float fs_ma = config.dc_ovrc_ma > 0 ? float(config.dc_ovrc_ma) : 5000.0f;
    float vshunt_mv = fs_ma * shunt_ohm;    // mA * ohm = mV
    if (vshunt_mv > 320.0f) {
        fs_ma = 320.0f / shunt_ohm;
        vshunt_mv = 320.0f;
    }
    _current_lsb_ma = fs_ma / 32768.0f;

    // smallest PGA range covering the full-scale shunt voltage
    uint16_t pga = 3;
    const float pga_ranges_mv[4] = { 40.0f, 80.0f, 160.0f, 320.0f };
    for (uint8_t i = 0; i < 4; i++) {
        if (vshunt_mv <= pga_ranges_mv[i]) {
            pga = i;
            break;
        }
    }

    // Calibration register: Cal = 0.04096 / (Current_LSB * Rshunt)
    const uint16_t calib = uint16_t(0.04096f / ((_current_lsb_ma * 0.001f) * shunt_ohm));

    // Configuration register: 32 V bus range, selected PGA range,
    // 12-bit shunt and bus ADC, continuous shunt+bus conversion
    const uint16_t conf = (1U << 13) | (pga << 11) | (8U << 7) | (8U << 3) | 0x7;

    WITH_SEMAPHORE(_dev->get_semaphore());
    if (!write_register(REG_CONFIG, conf) || !write_register(REG_CALIB, calib)) {
        return false;
    }
    uint16_t dummy;
    if (!read_register(REG_CONFIG, dummy)) {
        return false;
    }

    _dev->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_Gripper_BTS7960::timer, void));
    return true;
}

void AP_Gripper_BTS7960::timer()
{
    if (_dev == nullptr) {
        return;
    }
    uint16_t raw;
    if (!read_register(REG_CURRENT, raw)) {
        _sample_failures++;
        return;
    }
    _sample_failures = 0;
    _current_ma = labs(int32_t(int16_t(raw) * _current_lsb_ma));
    _last_sample_ms = AP_HAL::millis();

    if (config.dc_stall_ma > 0 && _current_ma > config.dc_stall_ma) {
        if (_stall_since_ms == 0) {
            _stall_since_ms = _last_sample_ms;
        }
    } else {
        _stall_since_ms = 0;
    }
}

void AP_Gripper_BTS7960::init_gripper()
{
    _have_channels = SRV_Channels::find_channel(SRV_Channel::k_gripper_dc_fwd, _chan_fwd) &&
                     SRV_Channels::find_channel(SRV_Channel::k_gripper_dc_rev, _chan_rev);
    const bool have_en = SRV_Channels::find_channel(SRV_Channel::k_gripper_dc_en, _chan_en);
    if (!_have_channels) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Claw: GripperDcFwd/Rev outputs not assigned");
    } else {
        uint32_t mask = (1U << _chan_fwd) | (1U << _chan_rev);
        if (have_en) {
            mask |= (1U << _chan_en);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Claw: no enable output, tie R_EN/L_EN high");
        }
        // ChibiOS normal PWM groups top out at 400Hz, so request no more
        // than that. This keeps the request and the rate the HAL applies
        // in agreement, including on IOMCU (MAIN) outputs where the
        // reported rate is the requested one.
        const uint16_t req_freq = MIN(uint16_t(constrain_int16(config.dc_pwm_freq, 100, 5000)), uint16_t(400));
        hal.rcout->set_freq(mask, req_freq);

        // derive the PWM period from the rate the output actually ended
        // up at. The pulse widths below are duty cycles of this period.
        uint16_t freq_hz = hal.rcout->get_freq(_chan_fwd);
        if (freq_hz == 0 || freq_hz > req_freq) {
            freq_hz = req_freq;
        }
        _pwm_period_us = uint16_t(1000000U / freq_hz);

        write_outputs(0, 0, false);
    }

    _sensor_ok = ina219_init();
    if (!_sensor_ok && config.dc_stall_ma > 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Claw: INA219 not found");
    }
    _state = ClawState::IDLE;
}

bool AP_Gripper_BTS7960::start_move(ClawState new_state)
{
    if (!_have_channels) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Claw: outputs not assigned");
        return false;
    }
    if (_state == ClawState::FAULT) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Claw: fault active, stop to reset");
        return false;
    }
    if (config.dc_stall_ma > 0 && !_sensor_ok) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Claw: INA219 not found");
        return false;
    }

    _state = new_state;
    _move_start_ms = AP_HAL::millis();
    _last_grab_or_release = _move_start_ms;
    if (new_state == ClawState::CLOSING) {
        config.state = AP_Gripper::STATE_GRABBING;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Claw: closing");
        LOGGER_WRITE_EVENT(LogEvent::GRIPPER_GRAB);
    } else {
        config.state = AP_Gripper::STATE_RELEASING;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Claw: opening");
        LOGGER_WRITE_EVENT(LogEvent::GRIPPER_RELEASE);
    }
    return true;
}

void AP_Gripper_BTS7960::grab()
{
    if (_state == ClawState::CLOSING || _state == ClawState::HOLDING) {
        return;
    }
    if (grabbed()) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Claw: already closed");
        return;
    }
    start_move(ClawState::CLOSING);
}

void AP_Gripper_BTS7960::release()
{
    if (_state == ClawState::OPENING) {
        return;
    }
    if (released()) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Claw: already open");
        return;
    }
    start_move(ClawState::OPENING);
}

void AP_Gripper_BTS7960::stop()
{
    if (_state == ClawState::IDLE) {
        return;
    }
    stop_motor();
    _state = ClawState::IDLE;
    config.state = AP_Gripper::STATE_NEUTRAL;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Claw: stopped");
}

bool AP_Gripper_BTS7960::grabbed() const
{
    return config.state == AP_Gripper::STATE_GRABBED;
}

bool AP_Gripper_BTS7960::released() const
{
    return config.state == AP_Gripper::STATE_RELEASED;
}

bool AP_Gripper_BTS7960::valid() const
{
    return _have_channels && (config.dc_stall_ma <= 0 || _sensor_ok);
}

void AP_Gripper_BTS7960::conclude_close_stall(uint32_t now_ms)
{
    const uint32_t elapsed_ms = now_ms - _move_start_ms;
    const uint32_t tout_ms = uint32_t(config.dc_tout_s) * 1000;
    config.state = AP_Gripper::STATE_GRABBED;
    _last_grab_or_release = now_ms;

    if (elapsed_ms * 5 < tout_ms * 4) {
        // stalled well before the end of travel: an object was grasped
        if (config.dc_hold_pct > 0) {
            _state = ClawState::HOLDING;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Claw: object grasped, holding");
        } else {
            stop_motor();
            _state = ClawState::IDLE;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Claw: object grasped");
        }
    } else {
        stop_motor();
        _state = ClawState::IDLE;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Claw: fully closed");
    }
}

void AP_Gripper_BTS7960::conclude_open_stall(uint32_t now_ms)
{
    const uint32_t elapsed_ms = now_ms - _move_start_ms;
    const uint32_t tout_ms = uint32_t(config.dc_tout_s) * 1000;

    if (elapsed_ms * 5 >= tout_ms * 4) {
        // stalled near the end of travel: fully open
        stop_motor();
        _state = ClawState::IDLE;
        config.state = AP_Gripper::STATE_RELEASED;
        _last_grab_or_release = now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Claw: fully open");
    } else {
        enter_fault("obstacle while opening");
    }
}

void AP_Gripper_BTS7960::update_gripper()
{
    const uint32_t now_ms = AP_HAL::millis();

    // publish the live motor current for stall threshold calibration
    if (_sensor_ok && _state != ClawState::IDLE && now_ms - _last_curr_param_ms >= 500) {
        config.dc_curr.set(_current_ma);
        _last_curr_param_ms = now_ms;
    }

    switch (_state) {
    case ClawState::IDLE:
    case ClawState::FAULT:
        return;
    case ClawState::HOLDING:
        // same guards as while moving: the current sensor must stay
        // healthy and the holding current must stay below the
        // overcurrent limit, otherwise release the grip and fault
        if (config.dc_stall_ma > 0 &&
            (!_sensor_ok || _sample_failures >= 5 || now_ms - _last_sample_ms > 500)) {
            enter_fault("INA219 read failed");
            return;
        }
        if (config.dc_ovrc_ma > 0 && _current_ma > config.dc_ovrc_ma) {
            enter_fault("overcurrent while holding");
            return;
        }
        write_drive(true, float(config.dc_hold_pct));
        return;
    case ClawState::CLOSING:
    case ClawState::OPENING:
        break;
    }

    const bool closing = (_state == ClawState::CLOSING);
    const uint32_t elapsed_ms = now_ms - _move_start_ms;
    const uint32_t tout_ms = uint32_t(config.dc_tout_s) * 1000;

    // the current sensor must stay healthy while moving
    if (config.dc_stall_ma > 0 &&
        (!_sensor_ok || _sample_failures >= 5 || now_ms - _last_sample_ms > 500)) {
        enter_fault("INA219 read failed");
        return;
    }

    if (elapsed_ms >= tout_ms) {
        enter_fault("movement timeout");
        return;
    }

    // soft-start ramp, then full configured duty cycle
    float ramp_frac = 1.0f;
    if (config.dc_ramp_ms > 0) {
        ramp_frac = constrain_float(float(elapsed_ms) / float(config.dc_ramp_ms), 0.0f, 1.0f);
    }
    write_drive(closing, float(config.dc_pwm_max) * ramp_frac);

    if (config.dc_stall_ma <= 0) {
        return;     // current-based detection disabled
    }
    if (elapsed_ms < uint32_t(config.dc_blank_ms)) {
        return;     // mask the start-up inrush current
    }

    // hard overcurrent limit: stop immediately
    if (config.dc_ovrc_ma > 0 && _current_ma > config.dc_ovrc_ma) {
        if (closing) {
            conclude_close_stall(now_ms);
        } else {
            enter_fault("overcurrent while opening");
        }
        return;
    }

    // sustained stall current
    if (_stall_since_ms != 0) {
        const uint32_t stall_from_ms = MAX(_stall_since_ms, _move_start_ms + uint32_t(config.dc_blank_ms));
        if (now_ms - stall_from_ms >= uint32_t(config.dc_stall_ms)) {
            if (closing) {
                conclude_close_stall(now_ms);
            } else {
                conclude_open_stall(now_ms);
            }
        }
    }
}

#endif  // AP_GRIPPER_BTS7960_ENABLED
