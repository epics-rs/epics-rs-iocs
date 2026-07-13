//! Protocol layer of `quadEMApp/FX4Src/drvFX4.cpp` — the Pyramid FX4
//! 4-channel picoammeter.
//!
//! The FX4 speaks JSON over a WebSocket. The IOC subscribes to the four ADC
//! value paths plus the gate GPIO path and then polls with a `get` event; the
//! meter answers with an `update` event carrying, per path, a list of
//! `[value, nanosecond-timestamp]` pairs.
//!
//! Everything here is a pure function of the received JSON: message encoding,
//! message decoding, and [`Fx4Cache`] — the per-channel sample cache, the
//! timestamp merge and the gate/trigger state machine of C++'s
//! `onMessageEvent`. The socket itself lives in [`crate::fx4`].

use std::collections::VecDeque;

use serde_json::{Map, Value, json};

use crate::drv_quad_em::{QeTriggerMode, QeTriggerPolarity};

/// C++ `FX4_NUM_CHANS`.
pub const FX4_NUM_CHANS: usize = 4;

/// C++ `ADC_PATHS`.
pub const ADC_PATHS: [&str; FX4_NUM_CHANS] = [
    "/fx4/adc/channel_1/value",
    "/fx4/adc/channel_2/value",
    "/fx4/adc/channel_3/value",
    "/fx4/adc/channel_4/value",
];

/// C++ `GATE_PATH`.
pub const GATE_PATH: &str = "/fx4/gpio_0/22/readback/value";

/// C++ `resolution_ = 24`.
pub const RESOLUTION: i32 = 24;

/// C++ `numAverage_(1)`: the trigger filter counts against this until the
/// first `setAcquireParams`.
pub const DEFAULT_NUM_AVERAGE: i32 = 1;

/// The FX4 samples its ADCs at 100 kHz; one published value is
/// `ValuesPerRead` samples long (C++ `sampleTime = 10e-6 * valuesPerRead`).
pub const ADC_SAMPLE_PERIOD: f64 = 10e-6;

/// C++ `drvFX4::setAcquireParams`.
pub fn sample_time(values_per_read: i32) -> f64 {
    ADC_SAMPLE_PERIOD * values_per_read as f64
}

// ===========================================================================
// Messages
// ===========================================================================

/// C++ `sendEventData`: `{"event": <event>, "data": <data>}`.
fn event_message(event: &str, data: Value) -> String {
    json!({ "event": event, "data": data }).to_string()
}

/// C++ `sendSubscribeEvent`: subscribe to the four ADC paths and the gate.
pub fn subscribe_message() -> String {
    let mut data = Map::new();
    for path in ADC_PATHS {
        data.insert(path.to_string(), Value::Bool(true));
    }
    data.insert(GATE_PATH.to_string(), Value::Bool(true));
    event_message("subscribe", Value::Object(data))
}

/// C++ `sendUnsubscribeEvent`: subscribing to nothing is how a subscription is
/// dropped.
pub fn unsubscribe_message() -> String {
    event_message("subscribe", Value::Object(Map::new()))
}

/// C++ `sendGetEvent`: ask for the samples accumulated since the last `get`.
pub fn get_message() -> String {
    event_message("get", Value::Null)
}

/// C++ `drvFX4::onMessage`: split a received frame into its event name and
/// payload. A frame without an `event` member is ignored, as it is upstream.
pub fn parse_message(payload: &str) -> Option<(String, Value)> {
    let response: Value = serde_json::from_str(payload).ok()?;
    let event = response.get("event")?.as_str()?.to_string();
    let data = response.get("data").cloned().unwrap_or(Value::Null);
    Some((event, data))
}

// ===========================================================================
// Event cache
// ===========================================================================

/// C++ `gateLevel_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GateLevel {
    Low,
    High,
    #[default]
    Unknown,
}

/// The acquisition settings `onMessageEvent` reads out of its members.
#[derive(Debug, Clone, Copy)]
pub struct TriggerConfig {
    pub mode: QeTriggerMode,
    pub polarity: QeTriggerPolarity,
    /// C++ `numAverage_`: how many samples one external trigger admits.
    pub num_average: i32,
}

/// What the driver must do with a merged event, in timestamp order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Fx4Action {
    /// C++ `computePositions(element.values)`.
    Sample([f64; FX4_NUM_CHANS]),
    /// C++ `triggerCallbacks()` on the closing edge of an external bulb.
    BulbTrigger,
}

/// C++ `ADCSample`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct AdcSample {
    val: f64,
    time: f64,
}

/// One entry of C++'s `std::multiset<sortedListElement>`.
#[derive(Debug, Clone, Copy)]
enum EventKind {
    Gate(bool),
    Adc([f64; FX4_NUM_CHANS]),
}

#[derive(Debug, Clone, Copy)]
struct TimedEvent {
    time: f64,
    kind: EventKind,
}

/// The state `drvFX4` carries between `update` messages.
#[derive(Debug, Default)]
pub struct Fx4Cache {
    adc: [VecDeque<AdcSample>; FX4_NUM_CHANS],
    /// C++ `startTime_`: the first timestamp seen, subtracted from every other.
    start_time: i64,
    gate_level: GateLevel,
    synchronized: bool,
    timestamp_mismatch: bool,
    trigger_active: bool,
    num_trigger_values: i32,
}

impl Fx4Cache {
    pub fn new() -> Self {
        Self::default()
    }

    /// C++ `drvFX4::setAcquire(1)`'s preamble.
    pub fn reset(&mut self) {
        for q in &mut self.adc {
            q.clear();
        }
        self.start_time = 0;
        self.gate_level = GateLevel::Unknown;
        self.synchronized = false;
        self.timestamp_mismatch = false;
        self.trigger_active = false;
        self.num_trigger_values = 0;
    }

    pub fn gate_level(&self) -> GateLevel {
        self.gate_level
    }

    pub fn is_trigger_active(&self) -> bool {
        self.trigger_active
    }

    /// C++ `drvFX4::onMessageEvent` for an `update` payload: cache the ADC
    /// samples, merge the four channels into whole samples, and run the merged
    /// stream through the gate/trigger filter.
    ///
    /// Upstream drops every gate event of a message whose ADC caches cannot be
    /// merged (`if (adcCache_[0].empty()) goto done;`, and the desynchronised
    /// branch that clears the caches): the gate transitions carried by that
    /// message are lost, so `gateLevel_` — and with it the external-gate filter
    /// and the external-trigger arming — keeps a stale value. Here the gate
    /// events are always applied; only the ADC merge is conditional.
    pub fn ingest(&mut self, data: &Value, cfg: &TriggerConfig) -> Vec<Fx4Action> {
        let Some(object) = data.as_object() else {
            return Vec::new();
        };

        // C++ inserts the gate events into the multiset while scanning the
        // JSON object and the merged ADC events afterwards, so a gate event
        // sorts ahead of an ADC event with the same timestamp.
        let mut events: Vec<TimedEvent> = Vec::new();

        for (path, values) in object {
            let is_gate = path == GATE_PATH;
            let chan = ADC_PATHS.iter().position(|p| p == path);
            if !is_gate && chan.is_none() {
                continue;
            }
            let Some(list) = values.as_array() else {
                continue;
            };

            for v in list {
                let Some(pair) = v.as_array() else { continue };
                if pair.len() < 2 {
                    continue;
                }
                // Upstream catches a non-integer timestamp but not a non-bool
                // gate value: `v[0].get<bool>()` throws out of onMessageEvent,
                // which aborts the message *and* the `sendGetEvent` that keeps
                // the poll loop turning. Both malformed values are skipped here.
                let Some(time) = pair[1].as_i64() else {
                    continue;
                };
                if self.start_time == 0 {
                    self.start_time = time;
                }
                let timestamp = (time - self.start_time) as f64 / 1e9;

                if is_gate {
                    let Some(level) = pair[0].as_bool() else {
                        continue;
                    };
                    events.push(TimedEvent {
                        time: timestamp,
                        kind: EventKind::Gate(level),
                    });
                } else {
                    let Some(val) = pair[0].as_f64() else {
                        continue;
                    };
                    self.adc[chan.expect("adc path")].push_back(AdcSample {
                        val,
                        time: timestamp,
                    });
                }
            }
        }

        self.merge_adc(&mut events);

        // std::multiset orders by timestamp and keeps insertion order among
        // equal keys; a stable sort by timestamp does the same.
        events.sort_by(|a, b| a.time.total_cmp(&b.time));

        self.apply(&events, cfg)
    }

    /// C++'s merge of the four per-channel caches into whole samples.
    fn merge_adc(&mut self, events: &mut Vec<TimedEvent>) {
        if self.adc[0].is_empty() {
            return;
        }
        let min_size = self.adc.iter().map(|q| q.len()).min().unwrap_or(0);
        let max_size = self.adc.iter().map(|q| q.len()).max().unwrap_or(0);

        if min_size != max_size {
            if !self.synchronized {
                log::error!(
                    "drvFX4: not synchronized and different number of samples per channel={} {} {} {}",
                    self.adc[0].len(),
                    self.adc[1].len(),
                    self.adc[2].len(),
                    self.adc[3].len()
                );
                for q in &mut self.adc {
                    q.clear();
                }
                return;
            }
        } else {
            self.synchronized = true;
        }

        for _ in 0..min_size {
            let mut values = [0.0; FX4_NUM_CHANS];
            let mut times = [0.0; FX4_NUM_CHANS];
            for j in 0..FX4_NUM_CHANS {
                let sample = self.adc[j].pop_front().expect("min_size samples cached");
                values[j] = sample.val;
                times[j] = sample.time;
            }

            if times[1..].iter().any(|t| *t != times[0]) {
                if !self.timestamp_mismatch {
                    log::error!(
                        "drvFX4: timestamps are not the same for a sample: {} {} {} {}",
                        times[0],
                        times[1],
                        times[2],
                        times[3]
                    );
                    self.timestamp_mismatch = true;
                }
            } else if self.timestamp_mismatch {
                log::error!("drvFX4: timestamps back to normal");
                self.timestamp_mismatch = false;
            }

            events.push(TimedEvent {
                time: times[0],
                kind: EventKind::Adc(values),
            });
        }
    }

    /// C++'s walk over the sorted event list: gate events move the trigger
    /// state machine, ADC events survive it or are dropped.
    fn apply(&mut self, events: &[TimedEvent], cfg: &TriggerConfig) -> Vec<Fx4Action> {
        let mut actions = Vec::new();

        for event in events {
            match event.kind {
                EventKind::Gate(high) => {
                    self.gate_level = if high {
                        GateLevel::High
                    } else {
                        GateLevel::Low
                    };
                    match cfg.mode {
                        QeTriggerMode::ExtTrigger
                            if self.gate_level == active_level(cfg.polarity) =>
                        {
                            self.trigger_active = true;
                            self.num_trigger_values = 0;
                        }
                        QeTriggerMode::ExtBulb
                            if self.gate_level == inactive_level(cfg.polarity) =>
                        {
                            actions.push(Fx4Action::BulbTrigger);
                        }
                        _ => {}
                    }
                }
                EventKind::Adc(values) => {
                    if cfg.mode == QeTriggerMode::ExtTrigger {
                        if !self.trigger_active {
                            continue;
                        }
                        self.num_trigger_values += 1;
                        if self.num_trigger_values > cfg.num_average {
                            self.trigger_active = false;
                            continue;
                        }
                    }
                    if matches!(cfg.mode, QeTriggerMode::ExtGate | QeTriggerMode::ExtBulb)
                        && self.gate_level == inactive_level(cfg.polarity)
                    {
                        continue;
                    }
                    actions.push(Fx4Action::Sample(values));
                }
            }
        }

        actions
    }
}

/// The gate level that opens the gate / arms the trigger.
fn active_level(polarity: QeTriggerPolarity) -> GateLevel {
    match polarity {
        QeTriggerPolarity::Positive => GateLevel::High,
        QeTriggerPolarity::Negative => GateLevel::Low,
    }
}

/// The gate level that closes the gate / fires the bulb callback.
fn inactive_level(polarity: QeTriggerPolarity) -> GateLevel {
    match polarity {
        QeTriggerPolarity::Positive => GateLevel::Low,
        QeTriggerPolarity::Negative => GateLevel::High,
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn free_run() -> TriggerConfig {
        TriggerConfig {
            mode: QeTriggerMode::FreeRun,
            polarity: QeTriggerPolarity::Positive,
            num_average: 1,
        }
    }

    /// One `update` payload: `chans[i]` is channel i+1's `[value, time]` list,
    /// `gate` the gate path's.
    fn update(chans: [&[(f64, i64)]; 4], gate: &[(bool, i64)]) -> Value {
        let mut map = Map::new();
        for (i, samples) in chans.iter().enumerate() {
            if samples.is_empty() {
                continue;
            }
            let list: Vec<Value> = samples.iter().map(|(v, t)| json!([v, t])).collect();
            map.insert(ADC_PATHS[i].to_string(), Value::Array(list));
        }
        if !gate.is_empty() {
            let list: Vec<Value> = gate.iter().map(|(v, t)| json!([v, t])).collect();
            map.insert(GATE_PATH.to_string(), Value::Array(list));
        }
        Value::Object(map)
    }

    fn samples(actions: &[Fx4Action]) -> Vec<[f64; 4]> {
        actions
            .iter()
            .filter_map(|a| match a {
                Fx4Action::Sample(v) => Some(*v),
                Fx4Action::BulbTrigger => None,
            })
            .collect()
    }

    #[test]
    fn subscribe_get_and_unsubscribe_messages() {
        let sub: Value = serde_json::from_str(&subscribe_message()).unwrap();
        assert_eq!(sub["event"], "subscribe");
        for path in ADC_PATHS {
            assert_eq!(sub["data"][path], Value::Bool(true));
        }
        assert_eq!(sub["data"][GATE_PATH], Value::Bool(true));

        let unsub: Value = serde_json::from_str(&unsubscribe_message()).unwrap();
        assert_eq!(unsub["event"], "subscribe");
        assert_eq!(unsub["data"], json!({}));

        let get: Value = serde_json::from_str(&get_message()).unwrap();
        assert_eq!(get["event"], "get");
        assert_eq!(get["data"], Value::Null);
    }

    #[test]
    fn parse_message_needs_an_event_name() {
        let (event, data) = parse_message(r#"{"event":"update","data":{"a":1}}"#).unwrap();
        assert_eq!(event, "update");
        assert_eq!(data["a"], json!(1));

        // No event member, a non-string event, and malformed JSON are ignored.
        assert!(parse_message(r#"{"data":{}}"#).is_none());
        assert!(parse_message(r#"{"event":7}"#).is_none());
        assert!(parse_message("not json").is_none());

        // A frame with no data member parses with a null payload.
        let (event, data) = parse_message(r#"{"event":"update"}"#).unwrap();
        assert_eq!(event, "update");
        assert_eq!(data, Value::Null);
    }

    #[test]
    fn sample_time_is_ten_microseconds_per_value() {
        assert_eq!(sample_time(1), 10e-6);
        assert_eq!(sample_time(1000), 0.01);
    }

    #[test]
    fn free_run_merges_the_four_channels_and_rebases_the_timestamps() {
        let mut cache = Fx4Cache::new();
        let data = update(
            [
                &[(1.0, 1_000_000_000), (5.0, 1_000_010_000)],
                &[(2.0, 1_000_000_000), (6.0, 1_000_010_000)],
                &[(3.0, 1_000_000_000), (7.0, 1_000_010_000)],
                &[(4.0, 1_000_000_000), (8.0, 1_000_010_000)],
            ],
            &[],
        );
        let actions = cache.ingest(&data, &free_run());
        assert_eq!(
            samples(&actions),
            vec![[1.0, 2.0, 3.0, 4.0], [5.0, 6.0, 7.0, 8.0]]
        );
        // Every cached sample was consumed.
        assert_eq!(cache.adc.iter().map(|q| q.len()).sum::<usize>(), 0);
    }

    #[test]
    fn a_partial_message_is_completed_by_the_next_one() {
        let mut cache = Fx4Cache::new();
        // Channels 1-3 deliver, channel 4 lags: nothing can be merged yet, and
        // the caches must survive because minSize == 0 == the empty channel.
        let first = update(
            [
                &[(1.0, 1_000_000_000)],
                &[(2.0, 1_000_000_000)],
                &[(3.0, 1_000_000_000)],
                &[],
            ],
            &[],
        );
        assert!(cache.ingest(&first, &free_run()).is_empty());
        // Not synchronized and unequal sizes: upstream clears the caches.
        assert_eq!(cache.adc.iter().map(|q| q.len()).sum::<usize>(), 0);

        // A whole message afterwards is merged and marks the cache synchronized.
        let second = update(
            [
                &[(9.0, 1_000_010_000)],
                &[(10.0, 1_000_010_000)],
                &[(11.0, 1_000_010_000)],
                &[(12.0, 1_000_010_000)],
            ],
            &[],
        );
        let actions = cache.ingest(&second, &free_run());
        assert_eq!(samples(&actions), vec![[9.0, 10.0, 11.0, 12.0]]);
        assert!(cache.synchronized);
    }

    #[test]
    fn once_synchronized_a_short_channel_only_holds_back_the_extra_samples() {
        let mut cache = Fx4Cache::new();
        cache.synchronized = true;
        let data = update(
            [
                &[(1.0, 1_000_000_000), (5.0, 1_000_010_000)],
                &[(2.0, 1_000_000_000)],
                &[(3.0, 1_000_000_000), (7.0, 1_000_010_000)],
                &[(4.0, 1_000_000_000), (8.0, 1_000_010_000)],
            ],
            &[],
        );
        let actions = cache.ingest(&data, &free_run());
        assert_eq!(samples(&actions), vec![[1.0, 2.0, 3.0, 4.0]]);
        // The three second samples stay cached for channel 2's next one.
        assert_eq!(cache.adc[0].len(), 1);
        assert_eq!(cache.adc[1].len(), 0);
    }

    #[test]
    fn a_gate_event_is_applied_even_when_no_adc_sample_can_be_merged() {
        // Upstream's `if (adcCache_[0].empty()) goto done;` throws this gate
        // transition away, leaving gateLevel_ Unknown; the port applies it.
        let mut cache = Fx4Cache::new();
        let cfg = TriggerConfig {
            mode: QeTriggerMode::ExtGate,
            polarity: QeTriggerPolarity::Positive,
            num_average: 10,
        };

        let gate_only = update([&[], &[], &[], &[]], &[(true, 1_000_000_000)]);
        assert!(cache.ingest(&gate_only, &cfg).is_empty());
        assert_eq!(cache.gate_level(), GateLevel::High);

        // The gate is open, so the samples of the next message pass.
        let data = update(
            [
                &[(1.0, 1_000_010_000)],
                &[(2.0, 1_000_010_000)],
                &[(3.0, 1_000_010_000)],
                &[(4.0, 1_000_010_000)],
            ],
            &[],
        );
        let actions = cache.ingest(&data, &cfg);
        assert_eq!(samples(&actions), vec![[1.0, 2.0, 3.0, 4.0]]);
    }

    #[test]
    fn ext_gate_positive_drops_the_samples_taken_while_the_gate_is_low() {
        let mut cache = Fx4Cache::new();
        let cfg = TriggerConfig {
            mode: QeTriggerMode::ExtGate,
            polarity: QeTriggerPolarity::Positive,
            num_average: 10,
        };
        // The gate closes at t1 and reopens at t3; one sample sits on each side
        // of each edge. The very first sample precedes any gate event, and an
        // unknown gate level is neither Low nor High, so upstream's filter lets
        // it through — the port keeps that.
        let data = update(
            [
                &[(1.0, 1_000_000_000), (5.0, 1_000_020_000)],
                &[(2.0, 1_000_000_000), (6.0, 1_000_020_000)],
                &[(3.0, 1_000_000_000), (7.0, 1_000_020_000)],
                &[(4.0, 1_000_000_000), (8.0, 1_000_020_000)],
            ],
            &[(false, 1_000_010_000)],
        );
        let actions = cache.ingest(&data, &cfg);
        assert_eq!(samples(&actions), vec![[1.0, 2.0, 3.0, 4.0]]);
        assert_eq!(cache.gate_level(), GateLevel::Low);

        let data = update(
            [
                &[(9.0, 1_000_040_000)],
                &[(10.0, 1_000_040_000)],
                &[(11.0, 1_000_040_000)],
                &[(12.0, 1_000_040_000)],
            ],
            &[(true, 1_000_030_000)],
        );
        let actions = cache.ingest(&data, &cfg);
        assert_eq!(samples(&actions), vec![[9.0, 10.0, 11.0, 12.0]]);
        assert_eq!(cache.gate_level(), GateLevel::High);
    }

    #[test]
    fn ext_gate_negative_inverts_the_open_level() {
        let mut cache = Fx4Cache::new();
        let cfg = TriggerConfig {
            mode: QeTriggerMode::ExtGate,
            polarity: QeTriggerPolarity::Negative,
            num_average: 10,
        };
        let data = update(
            [
                &[(1.0, 1_000_020_000)],
                &[(2.0, 1_000_020_000)],
                &[(3.0, 1_000_020_000)],
                &[(4.0, 1_000_020_000)],
            ],
            &[(true, 1_000_010_000)],
        );
        assert!(samples(&cache.ingest(&data, &cfg)).is_empty());

        let data = update(
            [
                &[(5.0, 1_000_040_000)],
                &[(6.0, 1_000_040_000)],
                &[(7.0, 1_000_040_000)],
                &[(8.0, 1_000_040_000)],
            ],
            &[(false, 1_000_030_000)],
        );
        assert_eq!(
            samples(&cache.ingest(&data, &cfg)),
            vec![[5.0, 6.0, 7.0, 8.0]]
        );
    }

    #[test]
    fn ext_trigger_admits_num_average_samples_per_edge() {
        let mut cache = Fx4Cache::new();
        let cfg = TriggerConfig {
            mode: QeTriggerMode::ExtTrigger,
            polarity: QeTriggerPolarity::Positive,
            num_average: 2,
        };
        let time = |i: i64| 1_000_000_000 + i * 10_000;
        let data = update(
            [
                &[(1.0, time(2)), (2.0, time(3)), (3.0, time(4))],
                &[(1.0, time(2)), (2.0, time(3)), (3.0, time(4))],
                &[(1.0, time(2)), (2.0, time(3)), (3.0, time(4))],
                &[(1.0, time(2)), (2.0, time(3)), (3.0, time(4))],
            ],
            &[(true, time(1))],
        );
        let actions = cache.ingest(&data, &cfg);
        // The rising edge arms the trigger; the first two samples pass, the
        // third disarms it.
        assert_eq!(
            samples(&actions),
            vec![[1.0, 1.0, 1.0, 1.0], [2.0, 2.0, 2.0, 2.0]]
        );
        assert!(!cache.is_trigger_active());
    }

    #[test]
    fn ext_trigger_drops_everything_before_the_first_edge() {
        let mut cache = Fx4Cache::new();
        let cfg = TriggerConfig {
            mode: QeTriggerMode::ExtTrigger,
            polarity: QeTriggerPolarity::Positive,
            num_average: 2,
        };
        let data = update(
            [
                &[(1.0, 1_000_000_000)],
                &[(1.0, 1_000_000_000)],
                &[(1.0, 1_000_000_000)],
                &[(1.0, 1_000_000_000)],
            ],
            &[],
        );
        assert!(cache.ingest(&data, &cfg).is_empty());
    }

    #[test]
    fn ext_bulb_fires_the_callback_on_the_closing_edge() {
        let mut cache = Fx4Cache::new();
        let cfg = TriggerConfig {
            mode: QeTriggerMode::ExtBulb,
            polarity: QeTriggerPolarity::Positive,
            num_average: 0,
        };
        let time = |i: i64| 1_000_000_000 + i * 10_000;
        let data = update(
            [
                &[(1.0, time(2))],
                &[(1.0, time(2))],
                &[(1.0, time(2))],
                &[(1.0, time(2))],
            ],
            &[(true, time(1)), (false, time(3))],
        );
        let actions = cache.ingest(&data, &cfg);
        assert_eq!(
            actions,
            vec![
                Fx4Action::Sample([1.0, 1.0, 1.0, 1.0]),
                Fx4Action::BulbTrigger
            ]
        );
    }

    #[test]
    fn malformed_entries_are_skipped() {
        let mut cache = Fx4Cache::new();
        let mut map = Map::new();
        // Missing timestamp, non-numeric value, scalar instead of a pair.
        map.insert(ADC_PATHS[0].to_string(), json!([[1.0], ["x", 2], 3]));
        // A non-bool gate value: upstream throws out of the whole message here.
        map.insert(GATE_PATH.to_string(), json!([["high", 1_000_000_000]]));
        // An unknown path is ignored.
        map.insert("/fx4/nope".to_string(), json!([[1.0, 1_000_000_000]]));
        let actions = cache.ingest(&Value::Object(map), &free_run());
        assert!(actions.is_empty());
        assert_eq!(cache.gate_level(), GateLevel::Unknown);

        // A payload that is not an object at all.
        assert!(cache.ingest(&Value::Null, &free_run()).is_empty());
    }

    #[test]
    fn reset_clears_the_cache_and_the_trigger_state() {
        let mut cache = Fx4Cache::new();
        let cfg = TriggerConfig {
            mode: QeTriggerMode::ExtTrigger,
            polarity: QeTriggerPolarity::Positive,
            num_average: 4,
        };
        let data = update(
            [
                &[(1.0, 1_000_020_000)],
                &[(1.0, 1_000_020_000)],
                &[(1.0, 1_000_020_000)],
                &[],
            ],
            &[(true, 1_000_010_000)],
        );
        cache.ingest(&data, &cfg);
        assert!(cache.is_trigger_active());
        assert_eq!(cache.gate_level(), GateLevel::High);
        assert_ne!(cache.start_time, 0);

        cache.reset();
        assert!(!cache.is_trigger_active());
        assert_eq!(cache.gate_level(), GateLevel::Unknown);
        assert_eq!(cache.start_time, 0);
        assert!(!cache.synchronized);
        assert_eq!(cache.adc.iter().map(|q| q.len()).sum::<usize>(), 0);
    }
}
