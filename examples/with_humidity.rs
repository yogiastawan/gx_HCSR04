#![no_std]
#![no_main]

use core::mem::MaybeUninit;

use cortex_m_semihosting::hprint;
use gihex_hc_sr04::{us_timer::TickerUs, HcSR04};
use panic_semihosting as _;

use cortex_m_rt::entry;
use stm32f1xx_hal::{
    gpio::{gpioa, Edge, ExtiPin, Input, Output, PullDown, PushPull, PA4},
    pac::{self, interrupt, TIM1},
    prelude::*,
    timer::{CounterUs, Instance, SysDelay},
};

static mut ECHO_PIN: MaybeUninit<gpioa::PA3<Input<PullDown>>> = MaybeUninit::uninit();
static mut SENSOR: MaybeUninit<HcSR04<PA4<Output<PushPull>>, SysDelay, MyCounter<TIM1>>> =
    MaybeUninit::uninit();

#[interrupt]
fn EXTI3() {
    let echo_pin = unsafe { &mut *ECHO_PIN.as_mut_ptr() };
    let sensor = unsafe { &mut *SENSOR.as_mut_ptr() };

    let _ = sensor.on_echo_pulse();

    echo_pin.clear_interrupt_pending_bit();
}

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let rcc = p.RCC.constrain();

    let clock = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(56.MHz())
        .pclk1(28.MHz())
        .adcclk(14.MHz())
        .freeze(&mut flash.acr);

    let mut afio = p.AFIO.constrain();

    let mut gpioa = p.GPIOA.split();

    let echo_pin = unsafe { &mut *ECHO_PIN.as_mut_ptr() };
    *echo_pin = gpioa.pa3.into_pull_down_input(&mut gpioa.crl);
    echo_pin.make_interrupt_source(&mut afio);
    let mut exti = p.EXTI;
    echo_pin.enable_interrupt(&mut exti);
    echo_pin.trigger_on_edge(&mut exti, Edge::RisingFalling);

    let trig_pin = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);

    let cp = cortex_m::Peripherals::take().unwrap();
    let mut delay = cp.SYST.delay(&clock);
    let counter = p.TIM1.counter_us(&clock);
    let mut my_counter = MyCounter::new(counter);

    let sensor = unsafe { &mut *SENSOR.as_mut_ptr() };
    *sensor = HcSR04::hc_sr04_new(trig_pin, &mut delay, &mut my_counter);

    sensor.set_temperature(35.0);
    sensor.set_humidity(60);

    loop {
        match sensor.get_distance::<u32>(gihex_hc_sr04::DistanceUnit::MilliMeter) {
            Ok(x) => hprint!("distance: {}", x),
            Err(_e) => hprint!("waiting wave return or error"),
        }

        hprint!(
            "last duration echo: {}",
            sensor.get_last_length_echo_time::<u32>(gihex_hc_sr04::TimeUnit::MicroSecond)
        )
    }
}

struct MyCounter<TIM> {
    counter: CounterUs<TIM>,
}

impl<TIM: Instance> MyCounter<TIM> {
    fn new(counter: CounterUs<TIM>) -> Self {
        Self { counter }
    }
}

impl<TIM: Instance> TickerUs for MyCounter<TIM> {
    fn get_tick(&self) -> u32 {
        self.counter.now().ticks()
    }

    fn get_frequency(&self) -> u32 {
        1_000_000
    }
}
