//! The array-publishing task.
//!
//! `NDDriverStdArrays::doCallbacks` calls `doCallbacksGenericPointer` inline
//! from the asyn write handler (NDDriverStdArrays.cpp:328). In `epics-rs` the
//! write handler runs synchronously inside the port actor's current-thread
//! runtime, where the `async` [`ArrayPublisher::publish`] cannot be awaited (a
//! nested `block_on` would panic). The driver therefore does the `doCallbacks`
//! bookkeeping synchronously — increment `NDArrayCounter`, stamp `uniqueId` and
//! the timestamps onto an independent snapshot of `pArrays[0]` — and hands the
//! finished `Arc<NDArray>` to this task, which performs the actual fan-out.
//!
//! Publishing in FIFO order on a single consumer preserves the driver's
//! callback ordering.

use std::sync::Arc;

use epics_rs::ad_core::ndarray::NDArray;
use epics_rs::ad_core::plugin::channel::ArrayPublisher;
use epics_rs::ad_core::runtime as rt;

pub(crate) struct PublisherContext {
    pub rx: rt::CommandReceiver<Arc<NDArray>>,
    pub publisher: ArrayPublisher,
}

pub(crate) fn start_publisher_task(ctx: PublisherContext) -> std::thread::JoinHandle<()> {
    rt::run_thread_named("NDSAPublish", move || publisher_task(ctx))
}

async fn publisher_task(mut ctx: PublisherContext) {
    // `recv` returns `None` when every `CommandSender` (held by the driver) has
    // been dropped, i.e. the driver is gone — end the task.
    while let Some(array) = ctx.rx.recv().await {
        ctx.publisher.publish(array).await;
    }
}
