


pub trait TickerUs {
    fn get_tick(&self) -> u32;

    fn get_frequency(&self) -> u32;
}
