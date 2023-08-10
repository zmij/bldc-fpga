#pragma once

#include <armpp/hal/handle_base.hpp>
#include <armpp/hal/registers.hpp>
#include <armpp/hal/uart_io.hpp>

namespace bldc {

namespace hal = armpp::hal;

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
    gate_reset_start = 4,
    gate_reset_wait  = 5,
    gate_reset_done  = 6,
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
};
static_assert(sizeof(status_registry) == sizeof(hal::raw_register));

using enc_counter_register  = hal::raw_read_only_register_field<0, 32>;
using rot_duration_register = hal::raw_read_only_register_field<0, 32>;
using rpm_register          = hal::raw_read_only_register_field<0, 32>;

union control_register {
    hal::bool_read_write_register_field<0>                     enable;
    hal::read_write_register_field<rotation_direction_t, 1, 2> dir;
    hal::bool_read_write_register_field<3>                     invert_phases;
};
static_assert(sizeof(control_register) == sizeof(hal::raw_register));

union pwm_control_register {
    hal::raw_read_write_register_field<0, 16> duty;
    hal::raw_read_only_register_field<16, 16> cycle;
};
static_assert(sizeof(pwm_control_register) == sizeof(hal::raw_register));

/**
 * @brief APB2 BLDC motor driver peripheral
 *
 * The purpose is to get the status of the motor and control it
 */
class bldc_motor {
public:
    static constexpr hal::address base_address   = 0x40002400;
    static constexpr std::size_t  register_count = 6;

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
    rotation_duration() volatile const
    {
        return rot_duration_;
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
        ctl_.enable = true;
    }

    void
    disable()
    {
        ctl_.enable = false;
    }

    rotation_direction_t
    direction() volatile const
    {
        return ctl_.dir;
    }

    void
    set_direction(rotation_direction_t dir)
    {
        ctl_.dir = dir;
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

private:
    status_registry       status_;
    enc_counter_register  enc_counter_;
    rot_duration_register rot_duration_;
    rpm_register          rpm_;
    control_register      ctl_;
    pwm_control_register  pwm_ctl_;
};
static_assert(sizeof(bldc_motor) == sizeof(hal::raw_register) * bldc_motor::register_count);

using bldc_motor_handle = hal::handle_base<bldc_motor>;

}    // namespace bldc
