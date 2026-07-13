//! Boundary tests for the per-record update queue.

use std::time::{Duration, SystemTime};

use async_opcua::types::{StatusCode, Variant};
use opcua::queue::{ProcessReason, Update, UpdateQueue};

fn data(v: i32) -> Update {
    Update::new(
        ProcessReason::IncomingData,
        Some(Variant::Int32(v)),
        StatusCode::Good,
        SystemTime::UNIX_EPOCH + Duration::from_secs(v as u64),
    )
}

fn value_of(u: &Update) -> i32 {
    match &u.data {
        Some(Variant::Int32(v)) => *v,
        other => panic!("expected an Int32 update, got {other:?}"),
    }
}

#[test]
fn an_update_pushed_into_an_empty_queue_is_the_one_that_asks_for_processing() {
    let mut q = UpdateQueue::new(3, true);
    assert!(q.push(data(1)));
    assert!(!q.push(data(2)));
    assert_eq!(q.len(), 2);
}

#[test]
fn nothing_is_dropped_below_the_capacity() {
    let mut q = UpdateQueue::new(3, true);
    for v in 1..=3 {
        q.push(data(v));
    }
    for v in 1..=3 {
        let (u, _) = q.pop().expect("update");
        assert_eq!(value_of(&u), v);
        assert_eq!(u.overrides, 0);
    }
    assert!(q.pop().is_none());
}

#[test]
fn pop_reports_the_reason_the_next_process_pass_will_see() {
    let mut q = UpdateQueue::new(3, true);
    q.push(data(1));
    q.push(Update::new(
        ProcessReason::ConnectionLoss,
        None,
        StatusCode::Good,
        SystemTime::UNIX_EPOCH,
    ));

    let (_, next) = q.pop().expect("update");
    assert_eq!(next, ProcessReason::ConnectionLoss);
    let (_, next) = q.pop().expect("update");
    assert_eq!(next, ProcessReason::None);
}

#[test]
fn discarding_the_oldest_hands_its_loss_count_to_the_survivor() {
    let mut q = UpdateQueue::new(2, true);
    q.push(data(1));
    q.push(data(2));
    q.push(data(3)); // drops 1
    q.push(data(4)); // drops 2

    let (first, _) = q.pop().expect("update");
    assert_eq!(value_of(&first), 3);
    // Two updates were lost ahead of this one, and both are accounted for on it.
    assert_eq!(first.overrides, 2);

    let (second, _) = q.pop().expect("update");
    assert_eq!(value_of(&second), 4);
    assert_eq!(second.overrides, 0);
}

#[test]
fn a_queue_of_one_still_counts_every_loss() {
    // The C pops the front and then reads `updq.front()` again to hand over the
    // loss count (`UpdateQueue.h:64-68`); at capacity 1 the queue is empty at
    // that point, which is undefined behaviour. A link with `cqsize=1` reaches
    // it.
    let mut q = UpdateQueue::new(1, true);
    q.push(data(1));
    q.push(data(2)); // drops 1
    q.push(data(3)); // drops 2
    assert_eq!(q.len(), 1);

    let (u, next) = q.pop().expect("update");
    assert_eq!(value_of(&u), 3);
    assert_eq!(u.overrides, 2);
    assert_eq!(next, ProcessReason::None);
}

#[test]
fn a_capacity_of_zero_still_holds_the_newest_update() {
    let mut q = UpdateQueue::new(0, true);
    q.push(data(1));
    q.push(data(2));
    let (u, _) = q.pop().expect("update");
    assert_eq!(value_of(&u), 2);
    assert_eq!(u.overrides, 1);
}

#[test]
fn discarding_the_newest_keeps_the_newest_value_and_loses_a_slot() {
    // `discard=new`: the update at the back takes on the incoming content, so
    // the latest value still arrives — one queue slot's worth of history is what
    // is lost.
    let mut q = UpdateQueue::new(2, false);
    q.push(data(1));
    q.push(data(2));
    q.push(data(3)); // 2 is overridden by 3
    q.push(data(4)); // 3 is overridden by 4

    let (first, _) = q.pop().expect("update");
    assert_eq!(value_of(&first), 1);
    assert_eq!(first.overrides, 0);

    let (second, _) = q.pop().expect("update");
    assert_eq!(value_of(&second), 4);
    assert_eq!(second.overrides, 2);
    // The newest update's timestamp and reason come with it.
    assert_eq!(
        second.timestamp,
        SystemTime::UNIX_EPOCH + Duration::from_secs(4)
    );
    assert_eq!(second.reason, ProcessReason::IncomingData);
}

#[test]
fn an_overrun_does_not_ask_for_processing_again() {
    // The record has already been asked to process the update that is still
    // queued; a second request would process it twice.
    let mut q = UpdateQueue::new(1, true);
    assert!(q.push(data(1)));
    assert!(!q.push(data(2)));

    let mut q = UpdateQueue::new(1, false);
    assert!(q.push(data(1)));
    assert!(!q.push(data(2)));
}
