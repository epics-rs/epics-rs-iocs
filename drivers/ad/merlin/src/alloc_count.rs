//! Counts allocations of one exact size, so a frame test can hold a frame to
//! the one buffer its pool allocates: C copies the socket bytes into `pData`
//! once (merlinDetector.cpp:464-490).

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

/// The size counted; 0 counts nothing.
static SIZE: AtomicUsize = AtomicUsize::new(0);
static COUNT: AtomicUsize = AtomicUsize::new(0);
/// One counting caller at a time, for runners that share the process.
static TURN: parking_lot::Mutex<()> = parking_lot::Mutex::new(());

struct Counting;

fn note(size: usize) {
    if size == SIZE.load(Ordering::Relaxed) {
        COUNT.fetch_add(1, Ordering::Relaxed);
    }
}

// SAFETY: forwards every call to `System` unchanged; the counter is atomic.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        note(layout.size());
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        note(layout.size());
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        note(new_size);
        unsafe { System.realloc(ptr, layout, new_size) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

/// Counting of allocations of exactly one size, on any thread, until
/// [`Counted::stop`].
pub(crate) struct Counted {
    _turn: parking_lot::MutexGuard<'static, ()>,
}

pub(crate) fn count(size: usize) -> Counted {
    let turn = TURN.lock();
    COUNT.store(0, Ordering::Relaxed);
    SIZE.store(size, Ordering::Relaxed);
    Counted { _turn: turn }
}

impl Counted {
    pub(crate) fn stop(self) -> usize {
        SIZE.store(0, Ordering::Relaxed);
        COUNT.load(Ordering::Relaxed)
    }
}
