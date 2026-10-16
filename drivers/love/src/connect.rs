//! Octet-port connect helpers for the `drvLoveInit` iocsh command. The
//! underlying asyn port is created first by `drvAsynSerialPortConfigure` or
//! `drvAsynIPPortConfigure`; [`init_serial_port`] looks it up by name (C
//! `initSerialPort`, `drvLove.c:485-560`) and [`set_default_eos`] applies the
//! fixed input/output EOS (C `setDefaultEos`, `drvLove.c:640-660`, called
//! unconditionally from `drvLoveInit` itself — unlike the `delaygen`
//! drivers, no `.cmd` startup fragment sets EOS for Love).

use epics_rs::asyn::asyn_record::get_port;
use epics_rs::asyn::port_handle::PortHandle;

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
        let base = PortDriverBase::new(name, 1, PortFlags::default());
        let claim = claim_port(name).unwrap();
        let (rt, _actor) =
            create_port_runtime(&claim, driver(base), RuntimeConfig::default()).unwrap();
        let handle = rt.port_handle().clone();
        claim.publish(handle.clone(), &open_owner()).unwrap();
        std::mem::forget(rt);
        handle
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
