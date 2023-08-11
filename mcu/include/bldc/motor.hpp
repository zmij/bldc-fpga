#pragma once

#include <armpp/hal/handle_base.hpp>
#include <armpp/hal/registers.hpp>
#include <armpp/hal/uart_io.hpp>

namespace bldc {

namespace hal = armpp::hal;
using hal::enabled_t;

enum class hall_sector_t {
    s0   = 0,
    s1   = 1,
    s2   = 2,
    s3   = 3,
    s4   = 4,
    s5   = 5,
    none = 0b111,
};

enum rotation_direction_t {
    none  = 0,
    cw    = 0b01,
    ccw   = 0b11,
    brake = 0b10,
};

hal::uart::uart_handle&
operator<<(hal::uart::uart_handle& dev, rotation_direction_t dir)
{
    switch (dir) {
    case rotation_direction_t::none:
        dev << "---";
        break;
    case rotation_direction_t::cw:
        dev << " CW";
        break;
    case rotation_direction_t::ccw:
        dev << "CCW";
        break;
    case rotation_direction_t::brake:
        dev << "BRK";
        break;
    }
    return dev;
}

enum class driver_state_t : std::uint32_t {
    idle             = 0,
    startup          = 1,
    run              = 2,
    error            = 3,
    position_control = 4,
    gate_reset_start = 5,
    gate_reset_wait  = 6,
    gate_reset_done  = 7,
};

hal::uart::uart_handle&
operator<<(hal::uart::uart_handle& dev, driver_state_t val)
{
    switch (val) {
    case driver_state_t::idle:
        dev << "IDLE";
        break;
    case driver_state_t::startup:
        dev << "STRT";
        break;
    case driver_state_t::run:
        dev << " RUN";
        break;
    case driver_state_t::error:
        dev << " ERR";
        break;
    case driver_state_t::position_control:
        dev << " POS";
        break;
    case driver_state_t::gate_reset_start:
        dev << "RSTS";
        break;
    case driver_state_t::gate_reset_wait:
        dev << "RSTW";
        break;
    case driver_state_t::gate_reset_done:
        dev << "RSTD";
        break;
    default:
        // Invalid value, asterisks to stand out
        dev << "**" << hal::uart::width_out(0)
            << static_cast<std::underlying_type_t<driver_state_t>>(val) << "*";
        break;
    }
    return dev;
}

union status_registry {
    hal::raw_read_only_register_field<0, 3>                   hall_values;
    hal::read_only_register_field<hall_sector_t, 3, 3>        sector;
    hal::read_only_register_field<rotation_direction_t, 6, 2> detected_rotation;
    hal::raw_read_only_register_field<8, 6>                   phase_enable;
    hal::bool_read_only_register_field<14>                    hall_error;
    hal::bool_read_only_register_field<15>                    driver_fault;
    hal::bool_read_only_register_field<16>                    overcurrent_warning;
    hal::read_only_register_field<driver_state_t, 17, 3>      driver_state;
    hal::raw_read_only_register_field<20, 5>                  pole_pairs;
};
static_assert(sizeof(status_registry) == sizeof(hal::raw_register));

using enc_counter_register            = hal::raw_read_only_register_field<0, 32>;
using transitions_per_period_register = hal::raw_read_only_register_field<0, 32>;
using rpm_register                    = hal::raw_read_only_register_field<0, 32>;

union control_register {
    hal::bool_read_write_register_field<0>                     enable;
    hal::read_write_register_field<rotation_direction_t, 1, 2> dir;
    hal::bool_read_write_register_field<3>                     invert_phases;
    volatile hal::raw_register                                 raw;
};
static_assert(sizeof(control_register) == sizeof(hal::raw_register));

union pwm_control_register {
    hal::raw_read_write_register_field<0, 16> duty;
    hal::raw_read_only_register_field<16, 16> cycle;
};
static_assert(sizeof(pwm_control_register) == sizeof(hal::raw_register));

union pos_control_register {
    hal::read_write_register_field<enabled_t, 0, 1> enable;
};
static_assert(sizeof(pos_control_register) == sizeof(hal::raw_register));

using target_pos_register = hal::raw_read_write_register_field<0, 32>;

/**
 * @brief APB2 BLDC motor driver peripheral
 *
 * The purpose is to get the status of the motor and control it
 */
class bldc_motor {
public:
    static constexpr hal::address base_address   = 0x40002400;
    static constexpr std::size_t  register_count = 8;

    status_registry volatile const&
    status() const
    {
        return status_;
    }

    std::uint32_t
    hall_values() volatile const
    {
        return status_.hall_values;
    }

    bool
    hall_error() volatile const
    {
        return status_.hall_error;
    }

    bool
    driver_fault() volatile const
    {
        return status_.driver_fault;
    }

    bool
    overcurrent() volatile const
    {
        return status_.overcurrent_warning;
    }

    driver_state_t
    state() volatile const
    {
        return status_.driver_state;
    }

    std::uint32_t
    pole_pairs() volatile const
    {
        return status_.pole_pairs;
    }

    rotation_direction_t
    detected_rotation() volatile const
    {
        return status_.detected_rotation;
    }

    hall_sector_t
    sector() volatile const
    {
        return status_.sector;
    }

    std::uint32_t
    phase_enable() volatile const
    {
        return status_.phase_enable;
    }

    std::uint32_t
    enc_counter() volatile const
    {
        return enc_counter_;
    }

    std::uint32_t
    target_position() volatile const
    {
        return target_;
    }

    /**
     * @brief Number of hall transitions per RPM measurement period
     *
     * @return std::uint32_t
     */
    std::uint32_t
    transitions_per_period() volatile const
    {
        return transitions_per_period_;
    }

    std::uint32_t
    rpm() volatile const
    {
        return rpm_;
    }

    bool
    enabled() volatile const
    {
        return ctl_.enable;
    }

    void
    enable()
    {
        control_register new_val{.raw = ctl_.raw};
        new_val.enable = true;
        ctl_.raw       = new_val.raw;
    }

    void
    disable()
    {
        control_register new_val{.raw = ctl_.raw};
        new_val.enable = false;
        ctl_.raw       = new_val.raw;
    }

    rotation_direction_t
    direction() volatile const
    {
        return ctl_.dir;
    }

    bool
    phases_inverted() volatile const
    {
        return ctl_.invert_phases;
    }

    void
    set_invert_phases(bool val)
    {
        ctl_.invert_phases = val;
    }

    std::uint32_t
    pwm_cycle() const
    {
        return pwm_ctl_.cycle;
    }

    std::uint32_t
    pwm_duty() volatile const
    {
        return pwm_ctl_.duty;
    }

    void
    set_pwm_duty(std::uint32_t duty)
    {
        pwm_ctl_.duty = duty;
    }

    void
    run(rotation_direction_t dir)
    {
        set_control(dir, true);
    }

    void
    brake()
    {
        set_control(rotation_direction_t::brake, true);
    }

    void
    stop()
    {
        set_control(rotation_direction_t::none, false);
    }

    void
    go_to_position(std::uint32_t pos)
    {
        target_         = pos;
        pos_ctl_.enable = enabled_t::enabled;
        ctl_.enable     = true;
    }

    void
    step(rotation_direction_t dir, std::uint32_t steps)
    {
        auto pos = enc_counter();
        if (dir == rotation_direction_t::cw)
            pos += steps;
        else if (dir == rotation_direction_t::ccw)
            pos -= steps;
        go_to_position(pos);
    }

private:
    void
    set_control(rotation_direction_t dir, bool enabled)
    {
        pos_ctl_.enable = enabled_t::disabled;
        if (enabled)
            ctl_.enable = false;
        control_register new_val{.raw = ctl_.raw};
        new_val.dir    = dir;
        new_val.enable = enabled;
        ctl_.raw       = new_val.raw;
    }

private:
    status_registry                 status_;
    enc_counter_register            enc_counter_;
    transitions_per_period_register transitions_per_period_;
    rpm_register                    rpm_;
    control_register                ctl_;
    pwm_control_register            pwm_ctl_;
    pos_control_register            pos_ctl_;
    target_pos_register             target_;
};
static_assert(sizeof(bldc_motor) == sizeof(hal::raw_register) * bldc_motor::register_count);

using bldc_motor_handle = hal::handle_base<bldc_motor>;

}    // namespace bldc
