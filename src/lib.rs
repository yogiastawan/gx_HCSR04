//! # Introduction
//! This is the library used to access the HC-SR04 sensor.
//! This library inspired by [`hc-sr04`](https://github.com/nordmoen/hc-sr04/tree/master).
//!For more detail how to use this library see [`Example`](https://github.com/yogiastawan/gx_HCSR04/blob/v0.1.0/examples/use_rtic.rs)
//!
//! ## How to use?
//! To use this library, there are several things that must be done, such as:
//! - Implementing external interrupt (`EXTI`) with `RAISING` and `FAILING` trigger to the pin `echo` (pin microcontroller that connected to the pin `echo` HC-SR04 sensor).
//! ```rust
//! let mut echo_pin = gpioa.pa3.into_pull_down_input(&mut gpioa.crl);
//! echo_pin.make_interrupt_source(&mut afio);
//! let mut exti = ctx.device.EXTI;
//! echo_pin.enable_interrupt(&mut exti);
//! echo_pin.trigger_on_edge(&mut exti, Edge::RisingFalling);
//! ```
//! - Create an object that impelements trait [`us_timer::TickerUs`]. This object is used to count the number of ticks.
//! ```rust
//! use stm32f1xx_hal::timer::{CounterUs, Instance};
//! pub struct MyCounter<TIM> {
//!     counter: CounterUs<TIM>,
//! }
//!
//! impl<TIM: Instance> MyCounter<TIM> {
//!     fn new(counter: CounterUs<TIM>) -> Self {
//!         Self { counter }
//!     }
//! }
//!
//! impl<TIM: Instance> TickerUs for MyCounter<TIM> {
//!     fn get_tick(&self) -> u32 {
//!         self.counter.now().ticks()
//!     }
//!
//!     fn get_frequency(&self) -> u32 {
//!         1_000_000
//!     }
//! }
//! ```
//!
//! - Create HC-SR04 object and use it.
//! ```rust
//! let hcsr04 = HcSR04::hc_sr04_new(trig_pin, &mut delay, &mut my_counter);
//! let distance=hcsr04.get_distance::<f32>(DistanceUnit::MilliMeter);
//! ```

#![no_std]

/// This module containt traits that used to count number of ticks.
pub mod us_timer;

use core::marker::PhantomData;

use embedded_hal::{blocking, digital::v2};
use num_traits::{float::FloatCore, NumCast};

const ULTRASONIC_SPEED_HALF: f32 = 171_605.0; //mm per second

const TRIGGER_TIME_IN_US: u8 = 10;

/// Error variant for HC-SR04
pub enum HsError {
    ///Error when HC-SR04 still waiting for soundwave back.
    OnWaitingEcho,
    ///Error when HC-SR04 waiting soundwave back too long (more than 38 ms).
    PulseTimeOut,
    ///Error when HC_SR04 on wrong state.
    WrongState,
}

struct Elapsed<T>(u32, PhantomData<T>); //start and end

impl<TICK> Elapsed<TICK>
where
    TICK: us_timer::TickerUs,
{
    fn new(ticker: &TICK) -> Self {
        Elapsed(ticker.get_tick(), PhantomData)
    }

    fn get_elapsed_time_us(&self, ticker: &TICK) -> u32 {
        (ticker.get_tick() - self.0) * 1_000_000 / ticker.get_frequency()
    }
}

///Struct HC-SR04 sensor object
pub struct HcSR04</*IN,*/ OUT, DELAY, COUNTER> {
    // pin_echo: IN,
    pin_trigger: OUT,
    delay: *mut DELAY,
    timer: *mut COUNTER,
    state: State<COUNTER>,
    last_length_time: u32,
}

///Distance unit variants
pub enum DistanceUnit {
    ///To measure distance in millimeter.
    MilliMeter,
    ///To measure distance in centimeter.
    CentiMeter,
    ///To measure distance in meter.
    Meter,
}

///Time unit variants
pub enum TimeUnit {
    ///To measure time in microsecond.
    MicroSecond,
    ///To measure time in millisceond.
    MilliSecond,
    ///To measure time in second.
    Second,
}

enum State<T> {
    Idle,
    Triggered,
    MeasurePulse(Elapsed<T>),
    Measure(u32),
}

impl</*IN,*/ OUT, DELAY, COUNTER> HcSR04</*IN,*/ OUT, DELAY, COUNTER>
where
    // IN: v2::InputPin,
    OUT: v2::OutputPin,
    DELAY: blocking::delay::DelayUs<u8>,
    COUNTER: us_timer::TickerUs,
{
    ///Create new `Hc-SR04` object.
    pub fn hc_sr04_new(
        // pin_echo: IN,
        pin_trigger: OUT,
        delay: &mut DELAY,
        timer: &mut COUNTER,
    ) -> Self {
        HcSR04 {
            // pin_echo,
            pin_trigger,
            delay,
            timer,
            state: State::Idle,
            last_length_time: 0,
        }
    }

    // pub fn get_echo_pin(&self) -> &IN {
    //     &self.pin_echo
    // }

    ///Measure distance. where T is `f32` or `f64`.
    pub fn get_distance<T>(&mut self, unit: DistanceUnit) -> Result<T, HsError>
    where
        T: FloatCore,
    {
        match self.state {
            State::Idle => {
                self.send_ping_force();
                self.state = State::Triggered;
                Err(HsError::OnWaitingEcho)
            }
            State::Triggered => Err(HsError::OnWaitingEcho),
            State::MeasurePulse(_) => Err(HsError::OnWaitingEcho),
            State::Measure(time) => {
                self.state = State::Idle;
                if time > 38000 {
                    return Err(HsError::PulseTimeOut);
                }
                let divider: u8 = match unit {
                    DistanceUnit::MilliMeter => 1,
                    DistanceUnit::CentiMeter => 10,
                    DistanceUnit::Meter => 100,
                };
                let distance = ULTRASONIC_SPEED_HALF * time as f32 / 1_000_000.0;
                Ok(NumCast::from(distance / (divider as f32)).unwrap())
            }
        }
    }

    ///Get last lenght of time soundwave detected back by sensor. Where T is number variants.
    pub fn get_last_length_echo_time<T>(&mut self, unit: TimeUnit) -> T
    where
        T: NumCast,
    {
        let divider: u32 = match unit {
            TimeUnit::MicroSecond => return NumCast::from(self.last_length_time).unwrap(),
            TimeUnit::MilliSecond => 1000,
            TimeUnit::Second => 1_000_000,
        };

        NumCast::from(self.last_length_time / divider).unwrap()
    }

    ///Send ulrasonic sundwave.
    pub fn send_ping_force(&mut self) {
        //make sure first pin trigger set to LOW
        let _ = self.pin_trigger.set_low();
        unsafe { self.delay.as_mut().unwrap().delay_us(2) };
        // then set trigger pin HIGH for 10us
        let _ = self.pin_trigger.set_high();
        unsafe { self.delay.as_mut().unwrap().delay_us(TRIGGER_TIME_IN_US) };
        let _ = self.pin_trigger.set_low();
    }

    ///Update state of `HC-SR04` sensor. This function must called inside external interrupt (`EXTI`) callback.
    pub fn on_echo_pulse(&mut self) -> Result<(), HsError> {
        self.state = match self.state {
            State::Triggered => {
                State::MeasurePulse(Elapsed::new(unsafe { self.timer.as_ref().unwrap() }))
            }
            State::MeasurePulse(ref x) => {
                let time = x.get_elapsed_time_us(unsafe { self.timer.as_ref().unwrap() });
                self.last_length_time = time;
                State::Measure(time)
            }
            _ => return Err(HsError::WrongState),
        };

        Ok(())
    }
}

unsafe impl<OUT, DELAY, COUNTER> Send for HcSR04<OUT, DELAY, COUNTER> {}
