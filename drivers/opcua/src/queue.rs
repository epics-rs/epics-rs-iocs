//! The per-element queue of incoming updates (`UpdateQueue.h`, `Update.h`).
//!
//! Each record's data element owns one. The client pushes an update whenever the
//! server sends new data, a read completes or fails, or the connection drops;
//! the record pops exactly one per process cycle. When the queue is full, one
//! update must be dropped, and the survivor counts how many were lost — the
//! record can then report the overrun rather than silently skipping values.

use std::collections::VecDeque;

use async_opcua::types::{StatusCode, Variant};

/// `enum ProcessReason` (`devOpcua.h:60`) — why a record is being processed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ProcessReason {
    /// Nothing queued.
    #[default]
    None,
    /// A monitored item delivered a new value.
    IncomingData,
    /// The session went down.
    ConnectionLoss,
    /// A read this record asked for finished.
    ReadComplete,
    /// A read this record asked for failed.
    ReadFailure,
    /// A write this record asked for finished.
    WriteComplete,
    /// A write this record asked for failed.
    WriteFailure,
    /// The record wants the client to read the node.
    ReadRequest,
    /// The record wants the client to write the node.
    WriteRequest,
}

impl ProcessReason {
    pub fn as_str(self) -> &'static str {
        match self {
            ProcessReason::None => "none",
            ProcessReason::IncomingData => "incomingData",
            ProcessReason::ConnectionLoss => "connectionLoss",
            ProcessReason::ReadComplete => "readComplete",
            ProcessReason::ReadFailure => "readFailure",
            ProcessReason::WriteComplete => "writeComplete",
            ProcessReason::WriteFailure => "writeFailure",
            ProcessReason::ReadRequest => "readRequest",
            ProcessReason::WriteRequest => "writeRequest",
        }
    }

    /// The reasons that carry a value the record should take
    /// (the `incomingData`/`readComplete` arms of every `readScalar`).
    pub fn carries_data(self) -> bool {
        matches!(
            self,
            ProcessReason::IncomingData | ProcessReason::ReadComplete
        )
    }
}

/// `enum ConnectionStatus` (`devOpcua.h:73`) — where an item is in its lifecycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConnectionStatus {
    /// No session.
    #[default]
    Down,
    /// The session is up and the item's initial read is in flight.
    InitialRead,
    /// The initial read is done and the item's `bini=write` write is in flight.
    InitialWrite,
    /// Steady state.
    Up,
}

/// One queued update (`Update<UA_Variant, UA_StatusCode>`, `Update.h:45`).
#[derive(Debug, Clone, PartialEq)]
pub struct Update {
    pub reason: ProcessReason,
    /// The value, for the reasons that carry one.
    pub data: Option<Variant>,
    pub status: StatusCode,
    /// The timestamp the record takes (`prec->time`) — already selected between
    /// server, source and data time by the item's link option.
    pub timestamp: std::time::SystemTime,
    /// How many updates this one replaced because the queue was full
    /// (`Update::overrides`).
    pub overrides: u64,
}

impl Update {
    pub fn new(
        reason: ProcessReason,
        data: Option<Variant>,
        status: StatusCode,
        timestamp: std::time::SystemTime,
    ) -> Self {
        Self {
            reason,
            data,
            status,
            timestamp,
            overrides: 0,
        }
    }

    /// This update takes over from `other`, which is lost (`Update::override`,
    /// `Update.h:116`): the newer content wins, and the loss count carries.
    fn take_over_from(&mut self, other: Update) {
        self.reason = other.reason;
        self.data = other.data;
        self.status = other.status;
        self.timestamp = other.timestamp;
        self.overrides += other.overrides + 1;
    }

    /// `count` updates were lost ahead of this one (`Update::override(unsigned
    /// long)`, `Update.h:135` — the `+ 1` counts the dropped update itself).
    fn absorb_lost(&mut self, count: u64) {
        self.overrides += count + 1;
    }
}

/// A bounded queue of updates, one per record (`UpdateQueue<T>`,
/// `UpdateQueue.h:39`).
///
/// The capacity comes from the link's `cqsize` (client queue size); the discard
/// policy from its `discard` option.
#[derive(Debug)]
pub struct UpdateQueue {
    capacity: usize,
    discard_oldest: bool,
    queue: VecDeque<Update>,
}

impl UpdateQueue {
    /// A capacity of 0 would leave nowhere to put an update, so it is raised to
    /// 1 — the C would enter its overrun branch immediately and dereference an
    /// empty `std::queue`.
    pub fn new(capacity: usize, discard_oldest: bool) -> Self {
        Self {
            capacity: capacity.max(1),
            discard_oldest,
            queue: VecDeque::new(),
        }
    }

    /// Push an update, dropping one if the queue is full. Returns `true` if the
    /// queue *was* empty, which is the client's signal to request record
    /// processing (`pushUpdate`'s `wasFirst`, `UpdateQueue.h:56`).
    ///
    /// Upstream C defect fixed at source: on overrun with `discardOldest` the C
    /// pops the front and then dereferences `updq.front()` again
    /// (`UpdateQueue.h:64-68`) to hand the loss count to the *next* update. With
    /// a capacity of one — reachable from a link with `cqsize=1`, or by setting
    /// `opcua_MinimumClientQueueSize` to 1 — the queue is empty at that point
    /// and `std::queue::front()` is undefined behaviour. Here the loss count
    /// goes to the update being pushed when nothing else remains, so the count
    /// is never lost and no capacity is special.
    pub fn push(&mut self, mut update: Update) -> bool {
        let was_empty = self.queue.is_empty();
        if self.queue.len() < self.capacity {
            self.queue.push_back(update);
            return was_empty;
        }

        if self.discard_oldest {
            let dropped = self.queue.pop_front().expect("queue is full, so non-empty");
            match self.queue.front_mut() {
                Some(next) => next.absorb_lost(dropped.overrides),
                None => update.absorb_lost(dropped.overrides),
            }
            self.queue.push_back(update);
        } else {
            // Discard the newest: the update at the back takes on the new
            // content, so the newest value still wins — only a queue *slot* is
            // lost, not the value.
            self.queue
                .back_mut()
                .expect("queue is full, so non-empty")
                .take_over_from(update);
        }
        false
    }

    /// Pop the update the record is to process, and report what the *next*
    /// process cycle would see (`popUpdate`'s `nextReason`, `UpdateQueue.h:87`).
    pub fn pop(&mut self) -> Option<(Update, ProcessReason)> {
        let update = self.queue.pop_front()?;
        let next = self.queue.front().map_or(ProcessReason::None, |u| u.reason);
        Some((update, next))
    }

    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }

    pub fn len(&self) -> usize {
        self.queue.len()
    }

    pub fn capacity(&self) -> usize {
        self.capacity
    }
}
