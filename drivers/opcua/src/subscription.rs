//! Subscription configuration (`Subscription.cpp`, `SubscriptionOpen62541.cpp`).

use crate::defaults;

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

    /// The subscription options of `opcuaSubscription`
    /// (`SubscriptionOpen62541.cpp:80-107`).
    pub fn set_option(&mut self, name: &str, value: &str) -> Result<(), String> {
        match name {
            "debug" => {
                self.debug = value
                    .parse()
                    .map_err(|_| format!("invalid value '{value}' for option 'debug'"))?;
            }
            "priority" => {
                self.priority = value
                    .parse()
                    .map_err(|_| format!("invalid value '{value}' for option 'priority'"))?;
            }
            _ => return Err(format!("unknown subscription option '{name}'")),
        }
        Ok(())
    }
}
