#![no_std]
#![no_main]

use gihex_hc_sr04::us_timer::TickerUs;
use panic_semihosting as _;
use stm32f1xx_hal::timer::{CounterUs, Instance};
pub struct MyCounter<TIM> {
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

// unsafe impl<TIM> Send for MyCounter<TIM> where TIM: Send {}

#[rtic::app(device=stm32f1xx_hal::pac)]
mod app {

    use cortex_m::asm::wfi;
    use cortex_m_semihosting::hprint;
    use gihex_hc_sr04::HcSR04;
    use stm32f1xx_hal::{
        gpio::{Edge, ExtiPin, Input, Output, PullDown, PushPull, PA3, PA4},
        pac::TIM1,
        prelude::*,
        timer::SysDelay,
    };

    use crate::MyCounter;

    #[shared]
    struct Shared {
        hcsr04: HcSR04<PA4<Output<PushPull>>, SysDelay, MyCounter<TIM1>>,
    }

    #[local]
    struct Local {
        echo_pin: PA3<Input<PullDown>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let mut flash = ctx.device.FLASH.constrain();
        let rcc = ctx.device.RCC.constrain();

        let mut afio = ctx.device.AFIO.constrain();

        let clock = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(56.MHz())
            .pclk1(28.MHz())
            .adcclk(14.MHz())
            .freeze(&mut flash.acr);
        let mut gpioa = ctx.device.GPIOA.split();
        let mut echo_pin = gpioa.pa3.into_pull_down_input(&mut gpioa.crl);
        echo_pin.make_interrupt_source(&mut afio);
        let mut exti = ctx.device.EXTI;
        echo_pin.enable_interrupt(&mut exti);
        echo_pin.trigger_on_edge(&mut exti, Edge::RisingFalling);

        let trig_pin = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);

        let cp = cortex_m::Peripherals::take().unwrap();
        let mut delay = cp.SYST.delay(&clock);
        let counter = ctx.device.TIM1.counter_us(&clock);
        let mut my_counter = MyCounter::new(counter);
        let hcsr04 = HcSR04::hc_sr04_new(trig_pin, &mut delay, &mut my_counter);

        (Shared { hcsr04 }, Local { echo_pin })
    }

    #[idle(shared=[hcsr04])]
    fn idle(ctx: idle::Context) -> ! {
        let mut sensor = ctx.shared.hcsr04;
        sensor.lock(
            |x| match x.get_distance::<f32>(gihex_hc_sr04::DistanceUnit::MilliMeter) {
                Ok(value) => hprint!("Distance: {}", value),
                Err(_x) => hprint!("still waiting for sound return"),
            },
        );

        loop {
            wfi();
        }
    }

    #[task(binds=EXTI3, shared=[hcsr04],local=[echo_pin])]
    fn echo(ctx: echo::Context) {
        let mut sens = ctx.shared.hcsr04;
        sens.lock(|x| match x.on_echo_pulse() {
            Ok(_) => hprint!("Success update HcSr04 sensor state"),
            Err(_) => hprint!("Error on triggered echo pin"),
        });
        let ec = ctx.local.echo_pin;
        ec.clear_interrupt_pending_bit();
    }
}
