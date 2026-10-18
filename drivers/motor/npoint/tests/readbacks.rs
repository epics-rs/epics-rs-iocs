//! `LC400Axis::poll` sets its readbacks in the controller port's parameter
//! library, for the callParamCallbacks that ends the poll
//! (LC400MotorDriver.cpp, asynMotorController.cpp:319).

#![cfg(target_os = "linux")]

use std::sync::Arc;
use std::time::Duration;

use epics_rs::asyn::error::AsynResult;
use epics_rs::asyn::manager::PortManager;
use epics_rs::asyn::port::{DrvUserRequest, InterfaceMask, PortDriver, PortDriverBase, PortFlags};
use epics_rs::asyn::registry::{claim_port, open_owner};
use epics_rs::asyn::runtime::{RuntimeConfig, create_port_runtime};
use epics_rs::asyn::user::AsynUser;
use epics_rs::base::runtime::task::{BlockingBridge, sleep};
use epics_rs::base::server::database::PvDatabase;
use epics_rs::base::server::iocsh::registry::{ArgValue, CommandContext};

/// An LC400 line whose every `readSingle` answers `VALUE`.
struct Wire {
    base: PortDriverBase,
    /// The last frame written.
    frame: Vec<u8>,
}

const VALUE: i32 = 1234;

impl PortDriver for Wire {
    fn base(&self) -> &PortDriverBase {
        &self.base
    }
    fn base_mut(&mut self) -> &mut PortDriverBase {
        &mut self.base
    }
    fn io_write_octet(&mut self, _user: &mut AsynUser, data: &[u8]) -> AsynResult<usize> {
        self.frame = data.to_vec();
        Ok(data.len())
    }
    fn io_read_octet(&mut self, _user: &AsynUser, buf: &mut [u8]) -> AsynResult<usize> {
        // `A0`, the register, the value, `EOS`.
        let mut reply = self.frame.get(..5).unwrap_or_default().to_vec();
        reply.extend_from_slice(&VALUE.to_le_bytes());
        reply.push(0x55);
        let n = reply.len().min(buf.len());
        buf[..n].copy_from_slice(&reply[..n]);
        self.frame.clear();
        Ok(n)
    }
}

#[epics_rs::base::epics_test]
async fn a_poll_s_digital_position_lands_in_the_parameter_library() {
    let line = "lc400ReadbackLine";
    let claim = claim_port(line).unwrap();
    let wire = Wire {
        base: PortDriverBase::new(
            line,
            1,
            PortFlags::default(),
            InterfaceMask::UNMIGRATED_ALL,
            InterfaceMask::UNMIGRATED_INTERRUPTS,
        ),
        frame: Vec::new(),
    };
    let (rt, _jh) = create_port_runtime(&claim, wire, RuntimeConfig::default()).unwrap();
    claim
        .publish(rt.port_handle().clone(), &open_owner())
        .unwrap();
    let cmd = motor_npoint::ioc_lc400::lc400_commands()
        .into_iter()
        .find(|c| c.name == "LC400CreateController")
        .unwrap();
    let args = vec![
        ArgValue::String("lc400Readback".into()),
        ArgValue::String(line.into()),
        ArgValue::Int(1),
        ArgValue::Int(50),
        ArgValue::Int(50),
    ];
    let bridge = BlockingBridge::capture();
    let shell = std::thread::spawn(move || {
        let ctx = CommandContext::new(Arc::new(PvDatabase::new()), bridge);
        cmd.handler.call(&args, &ctx).unwrap();
        motor_common::testing::await_first_polls();
        let port = PortManager::global()
            .find_port_handle("lc400Readback")
            .unwrap();
        let reason = port
            .drv_user_create_blocking(&DrvUserRequest::new("LC400_DIGITAL_POS", 0))
            .unwrap()
            .reason;
        port.read_float64_blocking(reason, 0).ok()
    });
    while !shell.is_finished() {
        sleep(Duration::from_millis(5)).await;
    }
    assert_eq!(shell.join().unwrap(), Some(f64::from(VALUE)));
}
