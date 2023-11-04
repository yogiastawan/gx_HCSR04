# Gihex HC-SR04

[![Continuous integration](https://github.com/yogiastawan/gx_HCSR04/actions/workflows/rust.yml/badge.svg)](https://github.com/yogiastawan/gx_HCSR04)
[![GitHub](https://img.shields.io/github/license/yogiastawan/gx_HCSR04)](https://github.com/yogiastawan/gx_HCSR04/blob/master/LICENSE)
[![GitHub release (with filter)](https://img.shields.io/github/v/release/yogiastawan/gx_HCSR04)](https://github.com/yogiastawan/gx_HCSR04/releases)
[![GitHub issues](https://img.shields.io/github/issues/yogiastawan/gx_HCSR04)](https://github.com/yogiastawan/gx_HCSR04/issues)



## Introduction
This is the library used to access the HC-SR04 ultrasonic sensor. This library inspired by [`hc-sr04`](https://github.com/nordmoen/hc-sr04/tree/master).  
For more detail how to use this library see [`Example`](https://github.com/yogiastawan/gx_HCSR04/blob/v0.1.0/examples/use_rtic.rs).

## Features
- [x] Measure distance.
- [x] Get last duration ultrasonic wave returned
- [x] Set environment temperature for compensation. Only available if feature *`temperature`* or *`humidity`* enabled.
- [x] Set environment humidity for compensation. Only available if feature *`humidity`* enabled.

## How to Use?
To use this library, there are several things that must be done, such as:
- **Implementing external interrupt (`EXTI`) with `RAISING` and `FAILING` trigger to the pin `echo` (pin microcontroller that connected to the pin `echo` HC-SR04 sensor).**
 ```rust
 let mut echo_pin = gpioa.pa3.into_pull_down_input(&mut gpioa.crl);
 echo_pin.make_interrupt_source(&mut afio);
 let mut exti = ctx.device.EXTI;
 echo_pin.enable_interrupt(&mut exti);
 echo_pin.trigger_on_edge(&mut exti, Edge::RisingFalling);
 ```
 - **Create an object that impelements trait [`us_timer::TickerUs`]. This object is used to count the number of ticks.**
 ```rust
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
 ```

 - **Create HC-SR04 object and use it.**
 ```rust
 let hcsr04 = HcSR04::hc_sr04_new(trig_pin, &mut delay, &mut my_counter);
 let distance=hcsr04.get_distance::<f32>(DistanceUnit::MilliMeter);
 ```