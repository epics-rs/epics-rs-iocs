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
}

impl SubscriptionConfig {
    pub fn new(name: impl Into<String>, session: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            session: session.into(),
            publishing_interval: defaults::DEFAULT_PUBLISH_INTERVAL.get(),
            priority: 0,
            debug: 0,
        }
    }

    /// The keep-alive and notification limits are open62541's request defaults
    /// (`UA_CreateSubscriptionRequest_default`), which the C does not change.
    pub const MAX_KEEP_ALIVE_COUNT: u32 = 10;
    pub const MAX_NOTIFICATIONS_PER_PUBLISH: u32 = 0;

    /// The C keeps open62541's default timeout, 500 ms × 10000, at the configured
    /// interval (`SubscriptionOpen62541.cpp:46-49`). Past `u32::MAX` its cast is
    /// undefined; this saturates.
    pub fn lifetime_count(&self) -> u32 {
        (500.0 * 10000.0 / self.publishing_interval) as u32
    }

    /// One `key=value` subscription option (`SubscriptionOpen62541::setOption`);
    /// a bad name or value is reported and passed over, as in the C.
    pub fn set_option(&mut self, name: &str, value: &str) {
        if self.debug > 0 || name == "debug" {
            eprintln!(
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
