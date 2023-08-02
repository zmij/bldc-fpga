#include <armpp/fpm/fixed.hpp>
#include <armpp/hal/scb.hpp>
#include <armpp/hal/system.hpp>
#include <armpp/hal/systick.hpp>
#include <armpp/hal/timer.hpp>
#include <armpp/hal/uart.hpp>
#include <armpp/hal/uart_io.hpp>
#include <bldc/motor.hpp>

#include <chrono>

using address      = armpp::hal::address;
using uart_handle  = armpp::hal::uart::uart_handle;
using timer_handle = armpp::hal::timer::timer_handle;

// TODO Move the addresses to vendor-specific headers
constexpr address apb1periph_base = 0x40000000;
constexpr address timer0_address  = apb1periph_base + 0x0000;
constexpr address uart0_address   = apb1periph_base + 0x4000;

constexpr address apb2_periph_base = apb1periph_base + 0x02000;

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

    uart0 << "CPU ID: 0x" << hex_out << scb_handle->get_cpu_id().raw << "\r\n"
          << "System freq: " << dec_out
          << frequency_cast<armpp::frequency::megahertz>(clock.system_frequency()) << "\r\n"
          << "Ticks per milli: " << ticks_per_milli << "\r\n"
          << "Frequency period duration: "
          << clock.system_frequency().period_duration<armpp::chrono::nanoseconds>() << " "
          << clock.system_frequency().period_duration<armpp::chrono::picoseconds>() << "\r\n"
          << "Motor device address " << motor.operator->() << "\r\n"
          << "PWM cycle length " << motor->pwm_cycle() << "\r\n";

    std::uint32_t              hall_values = 0;
    bldc::rotation_direction_t dir         = bldc::rotation_direction_t::none;

    motor->enable();
    motor->set_direction(bldc::rotation_direction_t::cw);
    auto cycle = motor->pwm_cycle();
    motor->set_pwm_duty(cycle / 5);

    uart0 << "Start the main loop\r\n";

    while (1) {
        timer0.delay(ticks_per_milli * 10);
        if (motor->hall_values() != hall_values || motor->detected_rotation() != dir) {
            hall_values = motor->hall_values();
            dir         = motor->detected_rotation();

            uart0 << "t: " << width_out(10) << sysclock::now().time_since_epoch()
                  << " hall: " << width_out(3) << bin_out << hall_values
                  << " phase enable: " << bin_out << width_out(6) << motor->phase_enable()
                  << " sector: " << dec_out << width_out(0) << motor->sector()
                  << " dir: " << motor->direction() << " detected: " << motor->detected_rotation()
                  << " cnt: " << width_out(10) << motor->enc_counter() << " rpm " << width_out(5)
                  << motor->rpm() << "\r\n";
        }
    }
}

extern "C" void
hardware_fault()
{
    // Reconfigure UART0
    uart_handle uart0{uart0_address, {.enable{.tx = true}, .baud_rate = 230400}};
    uart0 << "*** Hardware fault ***\r\n";
    while (1) {}
}

extern "C" void
mem_manage_fault()
{
    uart_handle uart0{uart0_address, {.enable{.tx = true}, .baud_rate = 230400}};
    uart0 << "*** MPU fault ***\r\n";
    while (1) {}
}

extern "C" void
bus_fault()
{
    uart_handle uart0{uart0_address, {.enable{.tx = true}, .baud_rate = 230400}};
    uart0 << "*** Bus fault ***\r\n";
    while (1) {}
}

extern "C" void
usage_fault()
{
    uart_handle uart0{uart0_address, {.enable{.tx = true}, .baud_rate = 230400}};
    uart0 << "*** Usage fault ***\r\n";
    while (1) {}
}
