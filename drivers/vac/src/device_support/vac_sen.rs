//! `devVacSen` — the `asyn VacSen` device support for the `vs` record.

use epics_rs::asyn::adapter::AsynLink;
use epics_rs::asyn::asyn_record::get_port;
use epics_rs::base::error::{CaError, CaResult};
use epics_rs::base::server::device_support::{DeviceReadOutcome, DeviceSupport};
use epics_rs::base::server::record::Record;

use super::PortIo;
use crate::protocol::vac_sen::{
    self, Config, DevType, Readings, ResponseBuf, check_control_reply, configure, control_command,
    decode, place_offset, read_command, skips_read, strip_read_reply,
};
use crate::records::vs::{DGS_FIELD, IG1_FIELD, IG2_FIELD, VsRecord};

/// `asyn VacSen`.
pub const DTYP: &str = "asyn VacSen";

pub struct VacSen {
    link: AsynLink,
    io: Option<PortIo>,
    cfg: Option<Config>,
    /// C `pPvt->errCount`, persisted across process cycles.
    err_count: i32,
}

impl VacSen {
    /// Build from the parsed `INP` link. Configuration (which needs the
    /// record's `TYPE`) and the port connection are deferred to `init`.
    pub fn new(link: AsynLink) -> Self {
        Self {
            link,
            io: None,
            cfg: None,
            err_count: 0,
        }
    }

    /// One `devVacSenWriteRead`: write, read up to `READ_SIZE`, and flush only
    /// when the read filled the buffer (`nread == vacSen_READ_SIZE`) — the C
    /// heuristic for "no terminator seen, drop the junk".
    fn write_read(io: &PortIo, send: &[u8]) -> Vec<u8> {
        io.write(send);
        let reply = io.read(vac_sen::READ_SIZE);
        if reply.len() == vac_sen::READ_SIZE {
            io.flush();
        }
        reply
    }
}

impl DeviceSupport for VacSen {
    fn dtyp(&self) -> &str {
        DTYP
    }

    fn init(&mut self, record: &mut dyn Record) -> CaResult<()> {
        let rec = record
            .as_any_mut()
            .and_then(|a| a.downcast_mut::<VsRecord>())
            .ok_or_else(|| CaError::TypeMismatch("VacSen requires a vs record".into()))?;

        let dev = DevType::from_index(rec.tipe)
            .ok_or_else(|| CaError::FieldNotFound(format!("vs TYPE index {}", rec.tipe)))?;
        let cfg =
            configure(dev, self.link.addr, &self.link.drv_info).map_err(CaError::LinkError)?;

        let port = get_port(&self.link.port_name).ok_or_else(|| {
            CaError::LinkError(format!(
                "asyn port '{}' not found (call drvAsynSerialPortConfigure first)",
                self.link.port_name
            ))
        })?;
        self.io = Some(PortIo {
            handle: port.handle,
            addr: self.link.addr,
            timeout: self.link.timeout,
        });
        self.cfg = Some(cfg);
        Ok(())
    }

    fn read(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        let cfg = self
            .cfg
            .clone()
            .ok_or_else(|| CaError::LinkError("VacSen not initialised".into()))?;
        let io = self
            .io
            .as_ref()
            .ok_or_else(|| CaError::LinkError("VacSen has no port".into()))?;
        let dev = cfg.dev;

        let rec = record
            .as_any_mut()
            .and_then(|a| a.downcast_mut::<VsRecord>())
            .ok_or_else(|| CaError::TypeMismatch("VacSen requires a vs record".into()))?;

        rec.read_alarm = false;

        // C `readWrite_vs` (pact == 0): one control command per cycle when a
        // `SPC_MOD` field changed, else a plain read. MM200/CC10/MX200 have no
        // control commands, so `control_command` returns `None` and the change
        // flag is simply cleared.
        let control = if rec.chgc != 0 {
            let cmd = if rec.chgc & IG1_FIELD != 0 {
                rec.ig1s as usize
            } else if rec.chgc & IG2_FIELD != 0 {
                2 + rec.ig2s as usize
            } else if rec.chgc & DGS_FIELD != 0 {
                4 + rec.dgss as usize
            } else {
                0
            };
            rec.chgc = 0;
            control_command(&cfg, cmd)
        } else {
            None
        };

        let mut response = ResponseBuf::default();

        // C `devVacSenCallback`. A control command that fails or is rejected
        // jumps to `finish` before the read loop (leaving `errCount` untouched);
        // a rejected reply also raises READ_ALARM.
        let mut run_loop = true;
        if let Some(sendbuf) = control {
            let reply = Self::write_read(io, &sendbuf);
            if reply.is_empty() {
                run_loop = false;
            } else if check_control_reply(dev, &reply).is_err() {
                rec.read_alarm = true;
                run_loop = false;
            }
        }

        // The eight-slot read loop. A short/long reply or a device-flagged
        // error increments `errCount` and jumps to `finish`; a clean sweep
        // resets it to zero.
        let mut loop_completed = false;
        if run_loop {
            loop_completed = true;
            for i in 0..8 {
                if skips_read(&cfg, i) {
                    continue;
                }
                let reply = Self::write_read(io, &read_command(&cfg, i));
                if reply.is_empty() || reply.len() > 50 {
                    self.err_count += 1;
                    loop_completed = false;
                    break;
                }
                match strip_read_reply(dev, i, &reply) {
                    Ok(payload) => response.strcpy_at(place_offset(dev, i), &payload),
                    Err(_) => {
                        self.err_count += 1;
                        loop_completed = false;
                        break;
                    }
                }
            }
        }
        if loop_completed {
            self.err_count = 0;
        }

        // C `finish`: `recBuf` holds the response only on a clean read.
        let recbuf = if self.err_count == 0 {
            response
        } else {
            ResponseBuf::default()
        };

        // C `readWrite_vs` (pact == 1): the error-count gate.
        let err_limit = rec.err as i32;
        if self.err_count >= err_limit {
            // recGblSetSevr(READ_ALARM, INVALID); udf = 0.
            rec.read_alarm = true;
            rec.dev_ran = true;
            return Ok(DeviceReadOutcome::computed());
        }
        if self.err_count > 0 {
            // Transient error: keep the last good readings, clear UDF.
            rec.dev_ran = true;
            return Ok(DeviceReadOutcome::computed());
        }

        // Full decode. Fields the device type does not rewrite keep their
        // previous record values (C never resets CGAP/CGBP/SP*/SP*R).
        let prev = Readings {
            cgap: rec.cgap,
            cgbp: rec.cgbp,
            sp: rec.sp,
            spr: rec.spr,
            ..Readings::default()
        };
        let out = decode(dev, cfg.spt, &recbuf, &prev);

        rec.val = out.val;
        rec.cgap = out.cgap;
        rec.cgbp = out.cgbp;
        rec.ig1r = out.ig1r;
        rec.ig2r = out.ig2r;
        rec.dgsr = out.dgsr;
        rec.sp = out.sp;
        rec.spr = out.spr;

        // C: `lprs = log10(val); lcap = log10(cgap); lcbp = log10(cgbp);
        // pres = val; udf = 0;`
        rec.lprs = rec.val.log10();
        rec.lcap = rec.cgap.log10();
        rec.lcbp = rec.cgbp.log10();
        rec.pres = rec.val;
        rec.dev_ran = true;

        Ok(DeviceReadOutcome::computed())
    }

    fn write(&mut self, _record: &mut dyn Record) -> CaResult<()> {
        // The `vs` record is read-only from the wire's perspective; control
        // commands are issued inside `read` on the same processing pass as the
        // status read, exactly as the C `readWrite_vs` does.
        Ok(())
    }
}
