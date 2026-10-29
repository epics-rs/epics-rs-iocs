//! The `report(fp, level)` overrides of the model-3 Newport drivers.

use epics_rs::asyn::port::InterfaceMask;
use motor_common::{AxisReport, ControllerParams, ControllerReport};

/// A controller whose `report` prints its own lines, from `head`, before
/// `asynMotorController::report`.
pub(crate) struct ControllerHead<F>(pub F);

impl<F> ControllerParams for ControllerHead<F>
where
    F: Fn(&ControllerReport<'_>, &mut dyn std::fmt::Write) + Send + Sync + 'static,
{
    const INTERFACE_MASK: InterfaceMask = InterfaceMask::NONE;
    const INTERRUPT_MASK: InterfaceMask = InterfaceMask::NONE;

    fn report(&self, ctx: &ControllerReport<'_>, out: &mut dyn std::fmt::Write, level: i32) {
        (self.0)(ctx, out);
        ctx.report(out, level);
    }
}

/// An axis `report` that prints `lines(out, level)`.
pub(crate) struct AxisLines<F>(pub F);

impl<F> AxisReport for AxisLines<F>
where
    F: Fn(&mut dyn std::fmt::Write, i32) + Send + Sync,
{
    fn report(&self, out: &mut dyn std::fmt::Write, level: i32) {
        (self.0)(out, level);
    }
}

/// C's `movingPollPeriod_`/`idlePollPeriod_`: the create command's ms, in
/// seconds.
pub(crate) fn period_s(ms: u64) -> f64 {
    ms as f64 / 1000.0
}
