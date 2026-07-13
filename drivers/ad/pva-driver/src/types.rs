/// Commands sent from the driver to the PVA monitor task.
pub enum PvaCommand {
    Start,
    Stop,
    /// C++ `pvaDriver::writeOctet` calling `connectPv(value)` on a `PVAPvName`
    /// write: tear down the current channel/monitor and connect to the new PV.
    Reconnect(String),
}
