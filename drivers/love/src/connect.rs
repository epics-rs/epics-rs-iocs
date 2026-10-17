//! Octet-port connect helpers for the `drvLoveInit` iocsh command. The
//! underlying asyn port is created first by `drvAsynSerialPortConfigure` or
//! `drvAsynIPPortConfigure`; [`init_serial_port`] looks it up by name (C
//! `initSerialPort`, `drvLove.c:485-560`) and [`set_default_eos`] applies the
//! fixed input/output EOS (C `setDefaultEos`, `drvLove.c:640-660`, called
//! unconditionally from `drvLoveInit` itself — unlike the `delaygen`
//! drivers, no `.cmd` startup fragment sets EOS for Love).

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::exception::ExceptionCallbackGuard;
use epics_rs::asyn::port_handle::PortHandle;
use epics_rs::asyn::trace::TraceMask;

/// C sets the EOS on a line that may still be down (`lockPort` in
/// `asynOctetSyncIO.c:306`, `asynSetEos`'s QUEUE_EVEN_IF_NOT_CONNECTED), so a
/// powered-off device must not fail the configure.
fn eos_user(addr: i32) -> epics_rs::asyn::user::AsynUser {
    epics_rs::asyn::user::AsynUser::default()
        .with_addr(addr)
        .queue_even_if_not_connected()
}

/// C `initSerialPort`: the pre-configured octet port `port_name`. `Err`
/// carries C's console line for the failure.
pub fn init_serial_port(port_name: &str, addr: i32) -> Result<PortHandle, String> {
    let port = get_port(port_name).ok_or_else(|| {
        format!("initSerialPort::failure to connect with device {port_name} @ {addr}")
    })?;
    if port.handle.is_multi_device() {
        return Err(format!(
            "initSerialPort::{port_name} cannot be configured as multi device"
        ));
    }
    Ok(port.handle)
}

/// C `exceptCallback` (drvLove.c:663-687), which `drvLoveInit` adds on the
/// serial port's user (:442): any exception that leaves the serial port down
/// takes the love port down too, without its `disconnectIt`. The love port
/// comes back through its own auto-connect, whose `connectIt` refuses until
/// the serial port is up. C never removes the callback; the caller keeps the
/// guard for the life of the port.
pub fn follow_serial_port(
    serial: &PortHandle,
    ser_addr: i32,
    love: &PortHandle,
) -> ExceptionCallbackGuard {
    let under = serial.connect_device(ser_addr);
    let over = love.connect_device(-1);
    let trace = serial.trace().clone();
    let ser_name = serial.port_name().to_owned();
    let love = love.clone();
    serial.exception_callback_add(ser_addr, move |_| {
        trace.print_source(
            Some(&ser_name),
            Some(ser_addr),
            0,
            TraceMask::FLOW,
            (file!(), line!()),
            format_args!("drvLove::exceptionCallback\n"),
        );
        if under.connected() || !over.connected() {
            return;
        }
        // Runs on the serial port's actor: queued, and C ignores its status.
        let _ = love.exception_disconnect(-1);
    })
}

/// C `setDefaultEos`: its two console lines, and whether `drvLoveInit`
/// succeeds — C attempts both sets and returns only the output status
/// (drvLove.c:640-660), so an input refusal is printed and ignored.
pub fn set_default_eos(
    handle: &PortHandle,
    addr: i32,
    input_eos: &[u8],
    output_eos: &[u8],
) -> ([String; 2], bool) {
    let line = |dir: &str, ok: bool, eos: u8| {
        let verb = if ok { "set" } else { "set failed" };
        format!("drvLove::setDefaultEos {dir} EOS {verb} to \\0{eos}")
    };
    let input = handle
        .set_input_eos_blocking(eos_user(addr), input_eos)
        .is_ok();
    let output = handle
        .set_output_eos_blocking(eos_user(addr), output_eos)
        .is_ok();
    (
        [
            line("Input", input, input_eos[0]),
            line("Output", output, output_eos[0]),
        ],
        output,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use epics_rs::asyn::error::AsynResult;
    use epics_rs::asyn::port::InterfaceMask;
    use epics_rs::asyn::port::{PortDriver, PortDriverBase, PortFlags};
    use epics_rs::asyn::registry::{claim_port, open_owner};
    use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
    use epics_rs::asyn::user::AsynUser;

    /// A port with no EOS support of its own: both sets are refused.
    struct Bare(PortDriverBase);

    impl PortDriver for Bare {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
    }

    /// Takes the output EOS and refuses the input one.
    struct OutputOnly(PortDriverBase);

    impl PortDriver for OutputOnly {
        fn base(&self) -> &PortDriverBase {
            &self.0
        }
        fn base_mut(&mut self) -> &mut PortDriverBase {
            &mut self.0
        }
        fn set_output_eos(&mut self, _user: &AsynUser, _eos: &[u8]) -> AsynResult<()> {
            Ok(())
        }
    }

    fn publish<D: PortDriver>(name: &str, driver: impl FnOnce(PortDriverBase) -> D) -> PortHandle {
        let base = PortDriverBase::new(
            name,
            1,
            PortFlags::default(),
            InterfaceMask::UNMIGRATED_ALL,
            InterfaceMask::UNMIGRATED_ALL,
        );
        let claim = claim_port(name).unwrap();
        let (rt, _actor) =
            create_port_runtime(&claim, driver(base), RuntimeConfig::default()).unwrap();
        let handle = rt.port_handle().clone();
        claim.publish(handle.clone(), &open_owner()).unwrap();
        std::mem::forget(rt);
        handle
    }

    /// drvLove.c:663-687: the serial port dropping takes the love port down
    /// (C `exceptionDisconnect`), and `connectIt` brings it back only once
    /// the serial port is up again (:1003-1014).
    #[test]
    fn a_serial_drop_takes_the_love_port_down_until_the_serial_port_returns() {
        use crate::driver::LoveDriver;
        use crate::registry::{K_INSTRMAX, Model};
        use std::sync::{Arc, Mutex};
        use std::time::Duration;

        let serial = publish("loveFollowS0", Bare);
        let models = Arc::new(Mutex::new([Model::default(); K_INSTRMAX]));
        let driver = LoveDriver::new(
            "loveFollowL0",
            "loveFollowS0",
            serial.clone(),
            0,
            Duration::from_millis(100),
            models,
        )
        .unwrap();
        let claim = claim_port("loveFollowL0").unwrap();
        let (rt, _actor) = create_port_runtime(&claim, driver, RuntimeConfig::default()).unwrap();
        let love = rt.port_handle().clone();
        let _follow = follow_serial_port(&serial, 0, &love);
        assert!(love.is_connected_blocking(-1).unwrap());

        serial.disconnect_blocking().unwrap();
        assert!(!love.is_connected_blocking(-1).unwrap());
        let e = love.connect_blocking().unwrap_err();
        assert_eq!(
            e.message(),
            "port loveFollowL0 not connected to loveFollowS0"
        );

        serial.connect_blocking().unwrap();
        assert!(!love.is_connected_blocking(-1).unwrap());
        love.connect_blocking().unwrap();
        assert!(love.is_connected_blocking(-1).unwrap());
    }

    #[test]
    fn missing_port_gives_init_serial_port_line() {
        let Err(e) = init_serial_port("loveNoSuchPort", 3) else {
            panic!("loveNoSuchPort found");
        };
        assert_eq!(
            e,
            "initSerialPort::failure to connect with device loveNoSuchPort @ 3"
        );
    }

    #[test]
    fn multi_device_port_is_refused() {
        let base = PortDriverBase::new(
            "loveMultiSer",
            2,
            PortFlags {
                multi_device: true,
                ..PortFlags::default()
            },
            InterfaceMask::UNMIGRATED_ALL,
            InterfaceMask::UNMIGRATED_ALL,
        );
        let claim = claim_port("loveMultiSer").unwrap();
        let (rt, _actor) =
            create_port_runtime(&claim, Bare(base), RuntimeConfig::default()).unwrap();
        claim
            .publish(rt.port_handle().clone(), &open_owner())
            .unwrap();
        std::mem::forget(rt);
        let Err(e) = init_serial_port("loveMultiSer", 0) else {
            panic!("multi-device port accepted");
        };
        assert_eq!(
            e,
            "initSerialPort::loveMultiSer cannot be configured as multi device"
        );
    }

    #[test]
    fn input_eos_refusal_is_printed_and_ignored() {
        let h = publish("loveEosOutputOnly", OutputOnly);
        let (lines, ok) = set_default_eos(&h, 0, b"\x06", b"\x03");
        assert!(ok);
        assert_eq!(
            lines,
            [
                "drvLove::setDefaultEos Input EOS set failed to \\06",
                "drvLove::setDefaultEos Output EOS set to \\03",
            ]
        );
    }

    #[test]
    fn output_eos_refusal_fails_drv_love_init() {
        let h = publish("loveEosBare", Bare);
        let (lines, ok) = set_default_eos(&h, 0, b"\x06", b"\x03");
        assert!(!ok);
        assert_eq!(
            lines[1],
            "drvLove::setDefaultEos Output EOS set failed to \\03"
        );
    }
}
