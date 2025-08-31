#[derive(Clone, Debug)]
pub enum LunabotAction {
    /// default state, goes into soft stop when disconnected
    SoftStop,
    /// navigational autonomy
    Navigate(f64, f64),
    /// reverse if stuck
    Reverse,
    /// Dig cycle
    Dig,
    /// Dump cycle
    Dump,
}
