#![no_std]
#![no_main]

use esp_hal::analog::adc::{Adc, AdcConfig, Attenuation};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{DriveMode, Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::ledc::channel::{self, ChannelIFace};
use esp_hal::ledc::timer::{self, TimerIFace};
use esp_hal::ledc::{LSGlobalClkSource, Ledc, LowSpeed};
use esp_hal::main;
use esp_hal::time::{Duration, Instant, Rate};
use log::{error, info};

#[panic_handler]
fn panic(panic_info: &core::panic::PanicInfo) -> ! {
    error!("{}", panic_info);
    loop {}
}

esp_bootloader_esp_idf::esp_app_desc!();

const RAW_MAX: u32 = 4095;

// ---- Timing ----
const LOOP_MS: u64 = 20;
const WINDOW_ROUNDS: u32 = 25; // 25 x 20 ms = 500 ms slow speed count
const WINDOW_MS: u32 = 500;

// ---- Controller ----
const OPEN_LOOP: bool = false; // true: knob = duty directly. false: PID speed controller
const FAST_FEEDBACK: bool = true; // false: P and I use the 500 ms count
                                   // true: P and I use the fast pulse-interval speed
const MAX_SPEED_CPS: f32 = 120.0; // knob at 100% asks for 120 counts/s
const TOLERANCE_CPS: f32 = 8.0; // |error| inside this counts as "on target"
const KP: f32 = 0.1;
const KI: f32 = 1.0;
const KD: f32 = 0.02; // % of PWM per (count/s per second) of speed change

// ---- Fast speed measurement ----
const SPEED_FILTER: f32 = 0.3; // 0..1, smaller = smoother but slower
const STOP_TIMEOUT_US: u64 = 300_000; // no pulse for 300 ms = stopped

/// Convert ADC reading to a requested percentage (0..=100).
fn adc_to_duty(raw: u16) -> u8 {
    let raw = (raw as u32).min(RAW_MAX);
    let pct = (raw * 100 / RAW_MAX) as u8;
    if pct < 2 { 0 } else { pct }
}

/// Safety state: remembers a fault until the button is released
/// and the knob is back at zero.
struct Controller {
    fault_latched: bool,
}

impl Controller {
    fn new() -> Self {
        Self { fault_latched: false }
    }

    fn is_faulted(&self) -> bool {
        self.fault_latched
    }

    fn update(&mut self, raw: u16, fault_pressed: bool) -> u8 {
        let requested = adc_to_duty(raw);

        if fault_pressed {
            self.fault_latched = true;
        }
        if self.fault_latched && !fault_pressed && requested == 0 {
            self.fault_latched = false;
        }

        if self.fault_latched { 0 } else { requested }
    }
}

/// Watches encoder channel A (falling edges); channel B gives the direction.
/// Keeps the position, plus the time of the last pulse and the gap before it.
struct Encoder<'d> {
    clk: Input<'d>,
    dt: Input<'d>,
    last_clk: bool,
    position: i32,
    last_edge_us: u64,
    interval_us: u64,
    direction: f32, // +1.0 forward, -1.0 reverse
}

impl<'d> Encoder<'d> {
    fn new(clk: Input<'d>, dt: Input<'d>) -> Self {
        let last_clk = clk.is_high();
        Self {
            clk,
            dt,
            last_clk,
            position: 0,
            last_edge_us: 0,
            interval_us: 0,
            direction: 1.0,
        }
    }

    fn poll(&mut self) {
        let now = self.clk.is_high();
        if self.last_clk && !now {
            // A pulse just happened: remember when, and the gap since the last one
            let now_us = Instant::now().duration_since_epoch().as_micros() as u64;
            if self.last_edge_us != 0 {
                self.interval_us = now_us.saturating_sub(self.last_edge_us);
            }
            self.last_edge_us = now_us;

            if self.dt.is_high() {
                self.position = self.position.saturating_add(1);
                self.direction = 1.0;
            } else {
                self.position = self.position.saturating_sub(1);
                self.direction = -1.0;
            }
        }
        self.last_clk = now;
    }

    /// Instantaneous speed in counts/second, from the time between pulses.
    fn speed_cps(&self, now_us: u64) -> f32 {
        if self.interval_us == 0 {
            return 0.0;
        }
        let since = now_us.saturating_sub(self.last_edge_us);
        if since > STOP_TIMEOUT_US {
            return 0.0;
        }
        // Never report faster than "one pulse per time since the last pulse"
        let period = if since > self.interval_us { since } else { self.interval_us };
        self.direction * 1_000_000.0 / period as f32
    }
}

fn wait_ms_polling(ms: u64, enc: &mut Encoder<'_>) {
    let start = Instant::now();
    while start.elapsed() < Duration::from_millis(ms) {
        enc.poll();
    }
}

#[main]
fn main() -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);
    info!("motor controller starting (PID, fast speed)");

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // PWM output: GPIO5 (goes to the motor chip's V+)
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut pwm_timer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    pwm_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty10Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(5),
        })
        .unwrap();

    let mut pwm = ledc.channel(channel::Number::Channel0, peripherals.GPIO5);
    pwm.configure(channel::config::Config {
        timer: &pwm_timer,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    })
    .unwrap();

    // Potentiometer: GPIO3
    let mut adc_config = AdcConfig::new();
    let mut pot = adc_config.enable_pin(peripherals.GPIO3, Attenuation::_11dB);
    let mut adc = Adc::new(peripherals.ADC1, adc_config);

    // Fault button: GPIO6, fault lamp: GPIO7
    let fault_button = Input::new(
        peripherals.GPIO6,
        InputConfig::default().with_pull(Pull::Up),
    );
    let mut fault_lamp = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());

    // Encoder from the motor chip: ENC_A on GPIO0, ENC_B on GPIO1
    let enc_clk = Input::new(peripherals.GPIO0, InputConfig::default().with_pull(Pull::Up));
    let enc_dt = Input::new(peripherals.GPIO1, InputConfig::default().with_pull(Pull::Up));
    let mut encoder = Encoder::new(enc_clk, enc_dt);

    // Error lamps: yellow GPIO10 (too slow), blue GPIO4 (too fast)
    let mut slow_lamp = Output::new(peripherals.GPIO10, Level::Low, OutputConfig::default());
    let mut fast_lamp = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());

    let mut controller = Controller::new();

    let mut round: u32 = 0;
    let mut measured_cps: i32 = 0; // slow: counted over 500 ms
    let mut last_position: i32 = 0;
    let mut integral: f32 = 0.0;
    let mut speed_f: f32 = 0.0; // fast: from pulse intervals, filtered
    let mut prev_speed_f: f32 = 0.0;
    let mut d_peak: f32 = 0.0;

    loop {
        let mut window_done = false;
        let dt = LOOP_MS as f32 / 1000.0;

        // 1. SENSE the knob (averaged; keep watching the encoder meanwhile)
        let mut sum: u32 = 0;
        for _ in 0..8 {
            sum += nb::block!(adc.read_oneshot(&mut pot)).unwrap() as u32;
            encoder.poll();
        }
        let raw = (sum / 8) as u16;
        let fault_pressed = fault_button.is_low();

        // 2. The knob sets the request (a fault forces it to zero)
        let requested = controller.update(raw, fault_pressed);
        let target_cps = requested as f32 * MAX_SPEED_CPS / 100.0;

        // 3a. MEASURE slowly: position change over 500 ms (forward +, reverse -)
        round += 1;
        if round >= WINDOW_ROUNDS {
            let moved = encoder.position - last_position;
            last_position = encoder.position;
            measured_cps = moved * 1000 / WINDOW_MS as i32;
            round = 0;
            window_done = true;
        }

        // 3b. MEASURE fast: speed from the time between pulses, then filtered
        let now_us = Instant::now().duration_since_epoch().as_micros() as u64;
        let speed_inst = encoder.speed_cps(now_us);
        speed_f += SPEED_FILTER * (speed_inst - speed_f);

        // 3c. D TERM: rate of change of the fast speed (acts on the measurement)
        let d_term = -KD * (speed_f - prev_speed_f) / dt;
        prev_speed_f = speed_f;
        if d_term.abs() > d_peak.abs() {
            d_peak = d_term;
        }
        // 4. DECIDE the duty
        let speed_for_control = if FAST_FEEDBACK { speed_f } else { measured_cps as f32 };
        let error_cps = target_cps - speed_for_control;

        let pid_duty: u8 = if controller.is_faulted() || requested == 0 {
            integral = 0.0;
            0
        } else if OPEN_LOOP {
            requested
        } else {
            // PID controller with anti-windup
            let candidate = integral + error_cps * dt;
            let unsaturated = KP * error_cps + KI * candidate + d_term;
            if unsaturated > 0.0 && unsaturated < 100.0 {
                integral = candidate;
            }
            let output = KP * error_cps + KI * integral + d_term;
            output.clamp(0.0, 100.0) as u8
        };

        // 5. ACT: PWM to the motor chip
        pwm.set_duty(pid_duty).unwrap();

        // 6. Lamps
        if controller.is_faulted() {
            fault_lamp.set_high();
        } else {
            fault_lamp.set_low();
        }

        if error_cps > TOLERANCE_CPS {
            slow_lamp.set_high();
        } else {
            slow_lamp.set_low();
        }
        if error_cps < -TOLERANCE_CPS {
            fast_lamp.set_high();
        } else {
            fast_lamp.set_low();
        }

        // 7. Log once per window (9 placeholders, 9 values)
        if window_done {
            info!(
                "raw={} target={:.0} meas={} fast={:.0} err={:.0} duty={}% int={:.1} d={:.1} enc={}",
                raw,
                target_cps,
                measured_cps,
                speed_f,
                error_cps,
                pid_duty,
                integral,
                d_peak,
                encoder.position
            );
            d_peak = 0.0;
        }

        wait_ms_polling(LOOP_MS, &mut encoder);
    }
}