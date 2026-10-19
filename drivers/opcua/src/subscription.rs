//! Subscription configuration (`Subscription.cpp`, `SubscriptionOpen62541.cpp`).

use epics_rs::base::runtime::log::errlog_vprintf;

use crate::defaults;
use crate::session::strtoul;

/// `opcuaSubscription(NAME, SESSION, INTERVAL, [options])`
/// (`iocshIntegration.cpp:159-201`).
#[derive(Debug, Clone, PartialEq)]
pub struct SubscriptionConfig {
    pub name: String,
    pub session: String,
    /// Publishing interval [ms].
    pub publishing_interval: f64,
    pub priority: u8,
    pub debug: u32,
    /// The three settings the C leaves to the client library's defaults. OPC UA
    /// Part 4 §5.13.2 requires the lifetime count to be at least three times the
    /// keep-alive count, which is where these come from; a notification limit of
    /// zero means the server chooses.
    pub lifetime_count: u32,
    pub max_keep_alive_count: u32,
    pub max_notifications_per_publish: u32,
}

impl SubscriptionConfig {
    pub fn new(name: impl Into<String>, session: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            session: session.into(),
            publishing_interval: defaults::DEFAULT_PUBLISH_INTERVAL.get(),
            priority: 0,
            debug: 0,
            max_keep_alive_count: 10,
            lifetime_count: 30,
            max_notifications_per_publish: 0,
        }
    }

    /// One `key=value` subscription option (`SubscriptionOpen62541::setOption`);
    /// a bad name or value is reported and passed over, as in the C.
    pub fn set_option(&mut self, name: &str, value: &str) {
        if self.debug > 0 || name == "debug" {
            epics_rs::base::stderr_println!(
                "Subscription {}: setting option {name} to {value}",
                self.name
            );
        }
        match name {
            "debug" => self.debug = strtoul(value) as u32,
            "priority" => match u8::try_from(strtoul(value)) {
                Ok(priority) => self.priority = priority,
                Err(_) => errlog_vprintf(format_args!(
                    "option '{name}' value out of range - ignored\n"
                )),
            },
            _ => errlog_vprintf(format_args!("unknown option '{name}' - ignored\n")),
        }
    }
}
