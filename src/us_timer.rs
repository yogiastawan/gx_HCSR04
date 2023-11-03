///Trait that used to count number of ticks
pub trait TickerUs {
    ///Get number of current ticks
    fn get_tick(&self) -> u32;

    ///Get frequency of tick
    fn get_frequency(&self) -> u32;
}
