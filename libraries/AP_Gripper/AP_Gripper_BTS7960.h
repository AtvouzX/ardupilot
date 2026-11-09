/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <AP_Gripper/AP_Gripper_Backend.h>

#if AP_GRIPPER_BTS7960_ENABLED

#include <AP_HAL/I2CDevice.h>

/*
  Gripper backend for a DC motor claw driven by a BTS7960 H-bridge,
  with an INA219 I2C current sensor used for end-of-travel, grasp and
  obstacle (stall) detection.

  Three servo outputs are used, at the GRIP_DC_PWM_FREQ frequency so
  that the pulse width is a duty cycle of the PWM period:
    SERVOx_FUNCTION 190 (GripperDcFwd) -> BTS7960 RPWM (close)
    SERVOx_FUNCTION 191 (GripperDcRev) -> BTS7960 LPWM (open)
    SERVOx_FUNCTION 192 (GripperDcEn)  -> BTS7960 R_EN and L_EN
 */
class AP_Gripper_BTS7960 : public AP_Gripper_Backend {
public:

    AP_Gripper_BTS7960(struct AP_Gripper::Backend_Config &_config) :
        AP_Gripper_Backend(_config) { }

    // grab - close the claw
    void grab() override;

    // release - open the claw
    void release() override;

    // stop - stop the claw and clear any fault
    void stop() override;

    // grabbed - returns true if the claw is closed/grasping
    bool grabbed() const override;

    // released - returns true if the claw is fully open
    bool released() const override;

    // valid - returns true if the outputs (and current sensor, if
    // current detection is enabled) were found at init
    bool valid() const override;

protected:

    // type-specific initialisations:
    void init_gripper() override;

    // type-specific periodic updates:
    void update_gripper() override;

private:

    enum class ClawState : uint8_t {
        IDLE,
        CLOSING,
        OPENING,
        HOLDING,
        FAULT,
    };

    // INA219 register addresses
    static const uint8_t REG_CONFIG = 0x00;
    static const uint8_t REG_CURRENT = 0x04;
    static const uint8_t REG_CALIB = 0x05;

    // start a movement in the given state; returns false if refused
    bool start_move(ClawState new_state);

    // stop the motor, applying the configured brake/coast behaviour
    void stop_motor();

    // enter the fault state and stop the motor
    void enter_fault(const char *reason);

    // conclude a stall-detected closing movement (grasp or fully closed)
    void conclude_close_stall(uint32_t now_ms);

    // conclude a stall-detected opening movement (fully open or obstacle)
    void conclude_open_stall(uint32_t now_ms);

    // write raw pulse widths (in microseconds of the PWM period) to
    // the forward, reverse and enable outputs
    void write_outputs(uint16_t fwd_pwm, uint16_t rev_pwm, bool enable);

    // write the outputs for a movement at the given duty cycle percent
    void write_drive(bool closing, float duty_pct);

    // PWM period in microseconds for GRIP_DC_PWM_FREQ, or the rate the
    // HAL actually applied if it clamped the requested frequency
    uint16_t period_us() const;

    // period of the claw outputs, filled in at init from get_freq()
    uint16_t _pwm_period_us;

    // configure the INA219; returns false if the sensor does not answer
    bool ina219_init();

    bool read_register(uint8_t reg, uint16_t &value) const;
    bool write_register(uint8_t reg, uint16_t value) const;

    // periodic I2C callback: sample the INA219 current register into
    // the cache and note when the stall threshold was first exceeded
    void timer();

    // servo output channels, UINT8_MAX when not assigned
    uint8_t _chan_fwd = UINT8_MAX;
    uint8_t _chan_rev = UINT8_MAX;
    uint8_t _chan_en = UINT8_MAX;
    bool _have_channels;

    // INA219 state
    AP_HAL::I2CDevice *_dev;
    bool _sensor_ok;
    float _current_lsb_ma;          // milliamps per INA219 current-register count
    int32_t _current_ma;            // latest sampled current (mA, magnitude)
    uint32_t _last_sample_ms;       // time of the last successful sample
    uint32_t _sample_failures;      // consecutive failed samples
    uint32_t _stall_since_ms;       // first time the current exceeded the stall threshold (0 = below)

    // state machine state
    ClawState _state = ClawState::IDLE;
    uint32_t _move_start_ms;        // time the current movement started
    uint32_t _last_curr_param_ms;   // last GRIP_DC_CURR live parameter update
};

#endif  // AP_GRIPPER_BTS7960_ENABLED
