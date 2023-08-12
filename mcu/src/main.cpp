#include <armpp/fpm/fixed.hpp>
#include <armpp/hal/addresses.hpp>
#include <armpp/hal/nvic.hpp>
#include <armpp/hal/scb.hpp>
#include <armpp/hal/system.hpp>
#include <armpp/hal/systick.hpp>
#include <armpp/hal/timer.hpp>
#include <armpp/hal/uart.hpp>
#include <armpp/hal/uart_io.hpp>
#include <bldc/motor.hpp>

#include <array>
#include <chrono>
#include <string_view>

using address      = armpp::hal::address;
using uart_handle  = armpp::hal::uart::uart_handle;
using timer_handle = armpp::hal::timer::timer_handle;
using irqn_t       = armpp::hal::irqn_t;

// TODO Move the addresses to vendor-specific headers
constexpr address timer0_address = armpp::hal::timer0_address;
constexpr address uart0_address  = armpp::hal::uart0_address;

constexpr irqn_t uart0_irqn{0};
constexpr irqn_t uart1_irqn{1};

constexpr auto baud_rate = 230400;

struct command {
    using function_type
        = std::function<void(bldc::bldc_motor_handle&, armpp::hal::uart::uart_handle&)>;
    std::string_view name;
    function_type    fn;

    void
    operator()(bldc::bldc_motor_handle& m, armpp::hal::uart::uart_handle& device) const
    {
        fn(m, device);
    }
};

auto const&
get_commands()
{
    constexpr std::uint32_t turn_count = 100;
    using armpp::hal::uart::dec_out;
    using bldc::rotation_direction_t;
    static std::array<command, 11> const commands{{
        {std::string_view{"stop"},
         [](bldc::bldc_motor_handle& m, armpp::hal::uart::uart_handle& uart) {
             uart << "Stop\r\n";
             m->stop();
         }},
        {"brake",
         [](bldc::bldc_motor_handle& m, armpp::hal::uart::uart_handle& uart) {
             uart << "Brake\r\n";
             m->brake();
         }},
        {"cw",
         [](bldc::bldc_motor_handle& m, armpp::hal::uart::uart_handle& uart) {
             uart << "Go CW\r\n";
             m->run(bldc::rotation_direction_t::cw);
         }},
        {"ccw",
         [](bldc::bldc_motor_handle& m, armpp::hal::uart::uart_handle& uart) {
             uart << "Go CCW\r\n";
             m->run(bldc::rotation_direction_t::ccw);
         }},
        {"up",
         [](bldc::bldc_motor_handle& m, armpp::hal::uart::uart_handle& uart) {
             uart << "Accelerate\r\n";
             auto pwm = m->pwm_duty() + 50;
             if (pwm > m->pwm_cycle() / 2) {
                 pwm = m->pwm_cycle() / 2;
             }
             uart << "New PWM " << dec_out << pwm << "\r\n";
             m->set_pwm_duty(pwm);
         }},
        {"down",
         [](bldc::bldc_motor_handle& m, armpp::hal::uart::uart_handle& uart) {
             uart << "Deccelerate\r\n";
             auto pwm = m->pwm_duty();
             if (pwm >= 50) {
                 pwm = pwm - 50;
             }
             uart << "New PWM " << dec_out << pwm << "\r\n";
             m->set_pwm_duty(pwm);
         }},
        {"turncw", 
         [](bldc::bldc_motor_handle& m, armpp::hal::uart::uart_handle& uart) {
             uart << dec_out << turn_count <<" turns CW\r\n"; 
             m->step(rotation_direction_t::cw, turn_count * 6 * m->pole_pairs()); 
         }},
        {"turnccw", 
         [](bldc::bldc_motor_handle& m, armpp::hal::uart::uart_handle& uart) {
             uart << dec_out << turn_count << " turns CCW\r\n"; 
             m->step(rotation_direction_t::ccw, turn_count * 6 * m->pole_pairs()); 
         }},
        {"stepcw", 
         [](bldc::bldc_motor_handle& m, armpp::hal::uart::uart_handle& uart) {
             uart << dec_out << "Step CW\r\n"; 
             m->step(rotation_direction_t::cw, 1); 
         }},
        {"stepccw", 
         [](bldc::bldc_motor_handle& m, armpp::hal::uart::uart_handle& uart) {
             uart << dec_out << "Step CCW\r\n"; 
             m->step(rotation_direction_t::ccw, 1); 
         }},
        {"zero", 
         [](bldc::bldc_motor_handle& m, armpp::hal::uart::uart_handle& uart) {
             uart << " go to zero\r\n"; 
             m->go_to_position(0); 
         }},
    }};
    return commands;
}

class command_parser {
public:
    static constexpr std::size_t buffer_size = 16;

    command_parser() noexcept = default;

    void
    operator()(armpp::hal::uart::uart_handle& device, char c)
    {
        switch (c) {
        case '\r':
            return;
        case '\n': {
            command_done(device);
            break;
        }
        default: {
            buffer_[current_++] = c;
        }
        }

        if (current_ == buffer_size - 2) {
            command_done(device);
        }
    }

private:
    void
    command_done(armpp::hal::uart::uart_handle& device)
    {
        using armpp::hal::uart::dec_out;
        buffer_[current_] = 0;
        std::string_view cmd_in{buffer_.data(), current_};
        current_ = 0;

        device << "Received command " << cmd_in << "\r\n";

        for (auto const& cmd : get_commands()) {
            if (cmd_in == cmd.name) {
                cmd(motor_, device);
                return;
            }
        }
        // TODO try parse number
        // std::uint32_t pwm = std::atoi(cmd_in.data());
        // if (pwm >= 0) {
        //     if (pwm > motor_->pwm_cycle() / 2) {
        //         pwm = motor_->pwm_cycle() / 2;
        //     }
        //     device << "Set pwm to " << pwm << "\r\n";
        //     motor_->set_pwm_duty(pwm);
        //     return;
        // }
        device << "Unknown command `" << cmd_in << "`\r\n";
    }

private:
    bldc::bldc_motor_handle       motor_;
    std::array<char, buffer_size> buffer_;
    std::size_t                   current_;
};

extern "C" int
main()
{
    using armpp::hal::uart::bin_out;
    using armpp::hal::uart::dec_out;
    using armpp::hal::uart::hex_out;
    using armpp::hal::uart::width_out;
    using sysclock = armpp::hal::system::clock;

    armpp::hal::scb::scb_handle   scb_handle;
    armpp::hal::nvic::nvic_handle nvic_handle;
    nvic_handle->enable_irq(uart0_irqn);

    auto const& clock           = armpp::hal::system::clock::instance();
    auto const  ticks_per_milli = clock.ticks_per_millisecond();

    uart_handle uart0{
        uart0_address,
        {.enable{.tx = true, .rx = true}, .enable_interrupt{.rx = true}, .baud_rate = baud_rate}};

    timer_handle timer0{timer0_address, {.enable = false, .interrupt_enable = false}};

    bldc::bldc_motor_handle motor;

    uart0 << "CPU ID: 0x" << hex_out << scb_handle->get_cpu_id().raw << "\r\n"
          << "System freq: " << dec_out
          << frequency_cast<armpp::frequency::megahertz>(clock.system_frequency()) << "\r\n"
          << "Ticks per milli: " << ticks_per_milli << "\r\n"
          << "Frequency period duration: "
          << clock.system_frequency().period_duration<armpp::chrono::nanoseconds>() << "\r\n"
          << "Pole pairs " << width_out(0) << motor->pole_pairs() << "\r\n"
          << "PWM cycle length " << width_out(0) << motor->pwm_cycle() << "\r\n";

    motor->set_invert_phases(true);

    command_parser cp;

    uart0->set_rx_handler(cp);

    auto cycle = motor->pwm_cycle();
    motor->set_pwm_duty(cycle / 20);

    std::uint32_t              hall_values = 0;
    bldc::rotation_direction_t dir         = bldc::rotation_direction_t::none;
    auto                       state       = motor->state();
    bool                       fault       = motor->driver_fault();

    uart0 << "Start the main loop\r\n";

    while (1) {
        timer0.delay(ticks_per_milli * 10);
        if (motor->hall_values() != hall_values || motor->detected_rotation() != dir
            || motor->state() != state || motor->driver_fault() != fault) {
            hall_values = motor->hall_values();
            dir         = motor->detected_rotation();
            state       = motor->state();
            fault       = motor->driver_fault();

            uart0 << "t: " << width_out(10) << sysclock::now().time_since_epoch() << " "
                  << motor->state() << " hall: " << width_out(3) << bin_out << hall_values
                  << " phase enable: " << bin_out << width_out(6) << motor->phase_enable()    //
                  << " sector: " << dec_out << width_out(0) << motor->sector()                //
                  << " dir: " << motor->direction() << " detected: " << motor->detected_rotation()
                  << " cnt: " << width_out(10) << motor->enc_counter()    //
                  << " tgt: " << motor->target_position()                 //
                  << " rpm: " << width_out(4) << motor->rpm()             //
                  << " pwm: " << motor->pwm_duty()                        //
                  << "/" << width_out(0) << cycle                         //
                  << (motor->pos_clt_enabled() ? " POS" : "")
                  << (motor->driver_fault() ? " FAULT" : "")
                  << (motor->overcurrent() ? " OVER" : "");
            uart0 << "\r\n";
        }
    }
}

extern "C" void
hardware_fault()
{
    // Reconfigure UART0
    uart_handle uart0{uart0_address, {.enable{.tx = true}, .baud_rate = baud_rate}};
    uart0 << "*** Hardware fault ***\r\n";
    while (1) {}
}

extern "C" void
mem_manage_fault()
{
    uart_handle uart0{uart0_address, {.enable{.tx = true}, .baud_rate = baud_rate}};
    uart0 << "*** MPU fault ***\r\n";
    while (1) {}
}

extern "C" void
bus_fault()
{
    uart_handle uart0{uart0_address, {.enable{.tx = true}, .baud_rate = baud_rate}};
    uart0 << "*** Bus fault ***\r\n";
    while (1) {}
}

extern "C" void
usage_fault()
{
    uart_handle uart0{uart0_address, {.enable{.tx = true}, .baud_rate = baud_rate}};
    uart0 << "*** Usage fault ***\r\n";
    while (1) {}
}
