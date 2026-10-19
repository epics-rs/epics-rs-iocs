//! Subscription configuration (`Subscription.cpp`, `SubscriptionOpen62541.cpp`).

use async_opcua::types::{CreateSubscriptionRequest, RequestHeader};
use epics_rs::base::runtime::log::errlog_vprintf;

use crate::defaults;
use crate::item::stream_double;
use crate::session::{SessionHandle, strtoul};

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

    /// The CreateSubscription request `SubscriptionOpen62541::create` sends.
    pub fn request(&self, request_header: RequestHeader) -> CreateSubscriptionRequest {
        CreateSubscriptionRequest {
            request_header,
            requested_publishing_interval: self.publishing_interval,
            requested_lifetime_count: self.lifetime_count(),
            requested_max_keep_alive_count: Self::MAX_KEEP_ALIVE_COUNT,
            max_notifications_per_publish: Self::MAX_NOTIFICATIONS_PER_PUBLISH,
            publishing_enabled: true,
            priority: self.priority,
        }
    }

    /// `SubscriptionOpen62541::show` (`SubscriptionOpen62541.cpp:78-96`) for this
    /// subscription on `session`; the enable state is "?" there too.
    pub fn show(&self, session: &SessionHandle, level: i32) -> String {
        let items: Vec<_> = session
            .items
            .lock()
            .iter()
            .filter(|item| item.lock().subscription() == Some(self.name.as_str()))
            .cloned()
            .collect();
        let revised = session
            .revised_publishing_intervals
            .lock()
            .get(&self.name)
            .copied()
            .unwrap_or(self.publishing_interval);
        let mut out = format!(
            "subscription={} session={} interval={}({}) prio={} enable=?(Y) debug={} items={}\n",
            self.name,
            session.name,
            stream_double(revised),
            stream_double(self.publishing_interval),
            self.priority,
            self.debug,
            items.len(),
        );
        if level >= 1 {
            for item in &items {
                out += &item.lock().show(level - 1);
            }
        }
        out
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
