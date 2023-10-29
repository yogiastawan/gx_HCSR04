#![no_std]

pub mod us_timer;

use core::marker::PhantomData;

use embedded_hal::{blocking, digital::v2};
use num_traits::{float::FloatCore, NumCast};

const ULTRASONIC_SPEED_HALF: f32 = 171_605.0; //mm per second

const TRIGGER_TIME_IN_US: u8 = 10;

pub enum HsError {
    OnWaitingEcho,
    PulseTimeOut,
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
pub struct HcSR04</*IN,*/ OUT, DELAY, COUNTER> {
    // pin_echo: IN,
    pin_trigger: OUT,
    delay: *mut DELAY,
    timer: *mut COUNTER,
    state: State<COUNTER>,
    last_length_time: u32,
}

pub enum DistanceUnit {
    MilliMeter,
    CentyMeter,
    Meter,
}

pub enum TimeUnit {
    MicroSecond,
    MilliSecond,
    Second,
}

enum State<T> {
    Idle,
    Triggered,
    MeasurePulse(Elapsed<T>),
    Measure(f32),
}

impl</*IN,*/ OUT, DELAY, COUNTER> HcSR04</*IN,*/ OUT, DELAY, COUNTER>
where
    // IN: v2::InputPin,
    OUT: v2::OutputPin,
    DELAY: blocking::delay::DelayUs<u8>,
    COUNTER: us_timer::TickerUs,
{
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
            State::Measure(distance) => {
                let divider: u8 = match unit {
                    DistanceUnit::MilliMeter => 1,
                    DistanceUnit::CentyMeter => 10,
                    DistanceUnit::Meter => 100,
                };
                self.state = State::Idle;
                Ok(NumCast::from(distance / (divider as f32)).unwrap())
            }
        }
    }

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

    pub fn send_ping_force(&mut self) {
        //make sure first pin trigger set to LOW
        let _ = self.pin_trigger.set_low();
        unsafe { self.delay.as_mut().unwrap().delay_us(2) };
        // then set trigger pin HIGH for 10us
        let _ = self.pin_trigger.set_high();
        unsafe { self.delay.as_mut().unwrap().delay_us(TRIGGER_TIME_IN_US) };
        let _ = self.pin_trigger.set_low();
    }

    pub fn on_echo_pulse(&mut self) -> Result<(), HsError> {
        self.state = match self.state {
            State::Triggered => {
                State::MeasurePulse(Elapsed::new(unsafe { self.timer.as_ref().unwrap() }))
            }
            State::MeasurePulse(ref x) => {
                let time = x.get_elapsed_time_us(unsafe { self.timer.as_ref().unwrap() });
                self.last_length_time = time;
                if time > 38000 {
                    return Err(HsError::PulseTimeOut);
                }
                let distance = ULTRASONIC_SPEED_HALF * time as f32 / 1_000_000.0;

                State::Measure(distance)
            }
            _ => return Err(HsError::WrongState),
        };
        Ok(())
    }
}

unsafe impl<OUT, DELAY, COUNTER> Send for HcSR04<OUT, DELAY, COUNTER> {}
