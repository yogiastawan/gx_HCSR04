use crate::DistanceUnit;

pub trait UltraSonicSensor<T> {
    fn get_distance(&mut self, unit: DistanceUnit) -> T;
}

pub trait Ultrasonic {
    fn send_ping(&mut self);

    fn get_echo_delay(&mut self) -> u64;
}
