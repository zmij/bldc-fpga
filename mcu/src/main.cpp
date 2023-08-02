#include <armpp/fpm/fixed.hpp>
#include <armpp/hal/scb.hpp>
#include <armpp/hal/system.hpp>
#include <armpp/hal/systick.hpp>
#include <armpp/hal/timer.hpp>
#include <armpp/hal/uart.hpp>
#include <armpp/hal/uart_io.hpp>

#include <chrono>

using address      = armpp::hal::address;
using uart_handle  = armpp::hal::uart::uart_handle;
using timer_handle = armpp::hal::timer::timer_handle;

// TODO Move the addresses to vendor-specific headers
constexpr address apb1periph_base = 0x40000000;
constexpr address timer0_address  = apb1periph_base + 0x0000;
constexpr address uart0_address   = apb1periph_base + 0x4000;

constexpr address apb2_periph_base = apb1periph_base + 0x02000;

namespace bldc {
namespace ah = armpp::hal;

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
    none   = 0,
    cw     = 0b01,
    ccw    = 0b11,
    break_ = 0b10,
};

ah::uart::uart_handle&
operator<<(ah::uart::uart_handle& dev, rotation_direction_t dir)
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
    case rotation_direction_t::break_:
        dev << "BRK";
        break;
    }
    return dev;
}

union status_registry {
    ah::raw_read_only_register_field<0, 3>                   hall_values;
    ah::read_only_register_field<hall_sector_t, 3, 3>        sector;
    ah::read_only_register_field<rotation_direction_t, 6, 2> detected_rotation;
};
static_assert(sizeof(status_registry) == sizeof(ah::raw_register));

using enc_counter_register  = ah::raw_read_only_register_field<0, 32>;
using rot_duration_register = ah::raw_read_only_register_field<0, 32>;

/**
 * @brief APB2 BLDC motor driver peripheral
 *
 * The purpose is to get the status of the motor and control it
 */
class bldc_motor {
public:
    static constexpr address     base_address   = 0x40002400;
    static constexpr std::size_t register_count = 3;

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
    enc_counter() volatile const
    {
        return enc_counter_;
    }

    std::uint32_t
    rotation_duration() volatile const
    {
        return rot_duration_;
    }

private:
    status_registry volatile status_;
    enc_counter_register volatile enc_counter_;
    rot_duration_register volatile rot_duration_;
};
static_assert(sizeof(bldc_motor) == sizeof(ah::raw_register) * bldc_motor::register_count);

using bldc_motor_handle = ah::handle_base<bldc_motor>;

};    // namespace bldc

extern "C" int
main()
{
    using armpp::hal::uart::bin_out;
    using armpp::hal::uart::dec_out;
    using armpp::hal::uart::hex_out;
    using armpp::hal::uart::width_out;
    using sysclock = armpp::hal::system::clock;

    armpp::hal::scb::scb_handle scb_handle;

    auto const& clock           = armpp::hal::system::clock::instance();
    auto const  ticks_per_milli = clock.ticks_per_millisecond();

    uart_handle  uart0{uart0_address, {.enable{.tx = true}, .baud_rate = 230400}};
    timer_handle timer0{timer0_address, {.enable = false, .interrupt_enable = false}};

    bldc::bldc_motor_handle motor;

    uart0 << "Hello world\r\n"
          << "CPU ID: 0x" << hex_out << scb_handle->get_cpu_id().raw << "\r\n"
          << "System freq: " << dec_out
          << frequency_cast<armpp::frequency::megahertz>(clock.system_frequency()) << "\r\n"
          << "Ticks per milli: " << ticks_per_milli << "\r\n"
          << "Frequency period duration: "
          << clock.system_frequency().period_duration<armpp::chrono::nanoseconds>() << " "
          << clock.system_frequency().period_duration<armpp::chrono::picoseconds>() << "\r\n"
          << "Motor device address " << motor.operator->() << "\r\n";

    std::uint32_t hall_values = 0;

    while (1) {
        timer0.delay(ticks_per_milli * 10);
        if (motor->hall_values() != hall_values) {
            hall_values = motor->hall_values();
            uart0 << "t: " << width_out(10) << sysclock::now().time_since_epoch()
                  << " hall: " << width_out(3) << bin_out << hall_values << " sector: " << dec_out
                  << width_out(0) << motor->sector() << " dir: " << motor->detected_rotation()
                  << " cnt: " << width_out(10) << motor->enc_counter() << " rot: " << width_out(10)
                  << motor->rotation_duration() << "\r\n";
        }
    }
}

extern "C" void
hardware_fault()
{
    // Reconfigure UART0
    uart_handle uart0{uart0_address, {.enable{.tx = true}, .baud_rate = 9600}};
    uart0 << "*** Hardware fault ***\r\n";
    while (1) {}
}

extern "C" void
mem_manage_fault()
{
    uart_handle uart0{uart0_address, {.enable{.tx = true}, .baud_rate = 9600}};
    uart0 << "*** MPU fault ***\r\n";
    while (1) {}
}

extern "C" void
bus_fault()
{
    uart_handle uart0{uart0_address, {.enable{.tx = true}, .baud_rate = 9600}};
    uart0 << "*** Bus fault ***\r\n";
    while (1) {}
}

extern "C" void
usage_fault()
{
    uart_handle uart0{uart0_address, {.enable{.tx = true}, .baud_rate = 9600}};
    uart0 << "*** Usage fault ***\r\n";
    while (1) {}
}
