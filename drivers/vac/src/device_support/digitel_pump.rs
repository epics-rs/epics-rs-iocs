//! `devDigitelPump` — the `asyn DigitelPump` device support for the `digitel`
//! record.

use epics_rs::asyn::adapter::AsynLink;
use epics_rs::asyn::asyn_record::get_port;
use epics_rs::base::error::{CaError, CaResult};
use epics_rs::base::server::device_support::{DeviceReadOutcome, DeviceSupport};
use epics_rs::base::server::record::Record;
use epics_rs::base::types::PvString;

use super::PortIo;
use crate::protocol::digitel::{
    self, Config, ControlFields, DevType, MAX_CONSEC_ERRORS, RESET_COMMANDS, ReadSlot, Readings,
    build_command, check_control_reply, configure, control_command, decode, initial_err_count,
    read_slot, strip_read_reply,
};
use crate::records::digitel::DigitelRecord;

/// `asyn DigitelPump`.
pub const DTYP: &str = "asyn DigitelPump";

/// `SIMM`'s `YES` menu index.
const YES: u16 = 1;

pub struct DigitelPump {
    link: AsynLink,
    io: Option<PortIo>,
    cfg: Option<Config>,
    /// C `pPvt->errCount`, persisted across process cycles. The Digitels are
    /// primed to 3 so the first cycle issues the `SL3`/`SL4` reset pair.
    err_count: i32,
}

impl DigitelPump {
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

    /// One `devDigitelPumpProcess`: flush, write, read. A QPC command shorter
    /// than ten bytes never reaches the wire — C leaves `*nread` uninitialised
    /// there; this port reports zero bytes, the "reply too small" path.
    fn process_dg(io: &PortIo, cfg: &Config, send: &[u8]) -> Vec<u8> {
        if cfg.dev == DevType::Qpc && send.len() < 10 {
            return Vec::new();
        }
        io.flush();
        io.write(send);
        io.read(digitel::READ_SIZE)
    }
}

impl DeviceSupport for DigitelPump {
    fn dtyp(&self) -> &str {
        DTYP
    }

    fn init(&mut self, record: &mut dyn Record) -> CaResult<()> {
        let rec = record
            .as_any_mut()
            .and_then(|a| a.downcast_mut::<DigitelRecord>())
            .ok_or_else(|| CaError::TypeMismatch("DigitelPump requires a digitel record".into()))?;

        let dev = DevType::from_index(rec.tipe)
            .ok_or_else(|| CaError::FieldNotFound(format!("digitel TYPE index {}", rec.tipe)))?;
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
        self.err_count = initial_err_count(dev);
        self.cfg = Some(cfg);
        Ok(())
    }

    fn read(&mut self, record: &mut dyn Record) -> CaResult<DeviceReadOutcome> {
        let cfg = self
            .cfg
            .clone()
            .ok_or_else(|| CaError::LinkError("DigitelPump not initialised".into()))?;
        let io = self
            .io
            .as_ref()
            .ok_or_else(|| CaError::LinkError("DigitelPump has no port".into()))?;
        let dev = cfg.dev;

        let rec = record
            .as_any_mut()
            .and_then(|a| a.downcast_mut::<DigitelRecord>())
            .ok_or_else(|| CaError::TypeMismatch("DigitelPump requires a digitel record".into()))?;

        rec.read_alarm = false;

        // C `readWrite_dg` (pact == 0) snapshots the readbacks into the `I*`
        // shadow fields before any I/O. Only setpoints 1-3 (`isp1..isp3`) are
        // captured; `ISP4` is deliberately left untouched, as in C.
        rec.ival = rec.val;
        rec.ilva = rec.lval;
        rec.imod = rec.modr;
        rec.ibak = rec.bakr;
        rec.icol = rec.cool;
        rec.isp[0] = rec.set[0];
        rec.isp[1] = rec.set[1];
        rec.isp[2] = rec.set[2];
        rec.iacw = rec.accw;
        rec.iaci = rec.acci;
        rec.ipty = rec.ptyp;
        rec.ibkn = rec.bkin;
        rec.is_ = rec.spr;
        rec.ih = rec.shr;
        rec.im = rec.smr;
        rec.ii = rec.svr;
        rec.ib3 = rec.s3br;
        rec.it3 = rec.s3tr;
        rec.iton = rec.tonl;
        rec.icrn = rec.crnt;
        rec.ivol = rec.volt;
        rec.ierr = rec.err;

        // Simulation mode does no wire I/O; the record's `process()` computes
        // VAL/MODR/SET/CRNT from SVMO/SVS1/SVS2/SVCR.
        if rec.simm == YES {
            return Ok(DeviceReadOutcome::computed());
        }

        // C `readWrite_dg` (pact == 0) command selection: a changed control
        // field wins, else a Digitel with a pending error count is reset, else
        // a plain read.
        enum Command {
            Control(Vec<u8>),
            Reset,
            Read,
        }
        let command = if rec.flgs != 0 {
            let fields = ControlFields {
                dspl: rec.dspl,
                mods: rec.mods,
                klck: rec.klck,
                baks: rec.baks,
                bkin: rec.bkin,
                spfg: rec.spfg,
                sps: rec.sps,
                spr: rec.spr,
                shs: rec.shs,
                shr: rec.shr,
                sms: rec.sms,
                smr: rec.smr,
                svs: rec.svs,
                svr: rec.svr,
            };
            let cc = control_command(&cfg, rec.flgs, &fields);
            if cc.clear_spfg {
                rec.spfg = 0;
            }
            rec.flgs = 0;
            Command::Control(cc.payload)
        } else if self.err_count != 0 && dev.is_digitel() {
            Command::Reset
        } else {
            Command::Read
        };

        let mut response = digitel::ResponseBuf::default();

        // C `devDigitelPumpCallback`. A control command that fails or is
        // rejected jumps to `finish` before the read loop (leaving `errCount`
        // untouched); a rejected reply also raises READ_ALARM.
        let mut run_loop = true;
        match command {
            Command::Control(payload) => {
                let sendbuf = build_command(&cfg, &payload);
                let reply = Self::process_dg(io, &cfg, &sendbuf);
                if reply.is_empty() {
                    run_loop = false;
                } else if check_control_reply(dev, &reply).is_err() {
                    rec.read_alarm = true;
                    run_loop = false;
                }
            }
            Command::Reset => {
                // `SL3`/`SL4` are sent verbatim and their replies ignored.
                Self::process_dg(io, &cfg, RESET_COMMANDS[0]);
                Self::process_dg(io, &cfg, RESET_COMMANDS[1]);
            }
            Command::Read => {}
        }

        // The eleven-slot read loop. Each stored reply lands at `30*i`. A
        // short reply or a device-flagged error increments `errCount` and jumps
        // to `finish`; a clean sweep resets it to zero. Unvisited slots (Digitel
        // beyond `noSPT + 1`, QPC setpoint slots 6-8) store nothing.
        let mut loop_completed = false;
        if run_loop {
            loop_completed = true;
            for i in 0..digitel::READ_SLOTS {
                match read_slot(&cfg, i) {
                    ReadSlot::Skip => continue,
                    ReadSlot::Send(payload) => {
                        let sendbuf = build_command(&cfg, &payload);
                        let reply = Self::process_dg(io, &cfg, &sendbuf);
                        if reply.is_empty() {
                            self.err_count += 1;
                            loop_completed = false;
                            break;
                        }
                        match strip_read_reply(dev, i, &reply) {
                            Ok(stripped) => {
                                response.strcpy_at(30 * i, &stripped);
                            }
                            Err(_) => {
                                self.err_count += 1;
                                loop_completed = false;
                                break;
                            }
                        }
                    }
                }
            }
        }
        if loop_completed {
            self.err_count = 0;
        }

        // C `finish`: unlike `devVacSen`, the Digitel copy is unconditional —
        // `recBuf` holds whatever the (possibly partial) read produced.
        let recbuf = response;

        // C `readWrite_dg` (pact == 1): `pr->err = errCount`, then the gate.
        rec.err = self.err_count as i16;
        if self.err_count > MAX_CONSEC_ERRORS {
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

        // Full decode. Fields the reply does not rewrite keep their previous
        // record values, matching C's carry-over of VOLT/CRNT/TONL/ACCW/ACCI/
        // COOL/BKIN/SP*R/S*HR/S3TR.
        let prev = Readings {
            volt: rec.volt,
            crnt: rec.crnt,
            tonl: rec.tonl as i64,
            accw: rec.accw,
            acci: rec.acci,
            cool: rec.cool as i64,
            bkin: rec.bkin,
            spr: rec.spr,
            shr: rec.shr,
            s3tr: rec.s3tr,
            ..Readings::default()
        };
        let out = decode(&cfg, &recbuf, &prev);

        rec.val = out.val;
        rec.lval = out.lval;
        rec.volt = out.volt;
        rec.crnt = out.crnt;
        rec.tonl = out.tonl as u32;
        rec.modr = out.modr;
        rec.cmor = out.cmor;
        rec.bakr = out.bakr;
        rec.set = out.set;
        rec.accw = out.accw;
        rec.acci = out.acci;
        rec.cool = out.cool as f64;
        rec.ptyp = out.ptyp;
        rec.bkin = out.bkin;
        rec.spr = out.spr;
        rec.shr = out.shr;
        rec.smr = out.smr;
        rec.svr = out.svr;
        rec.s3br = out.s3br;
        rec.s3tr = out.s3tr;
        if let Some(modl) = out.modl {
            rec.modl = PvString::from_bytes(modl);
        }
        if let Some(vers) = out.vers {
            rec.vers = PvString::from_bytes(vers);
        }
        rec.dev_ran = true;

        Ok(DeviceReadOutcome::computed())
    }

    fn write(&mut self, _record: &mut dyn Record) -> CaResult<()> {
        // Control commands are issued inside `read` on the same processing pass
        // as the status read, exactly as C's `readWrite_dg` does.
        Ok(())
    }
}
