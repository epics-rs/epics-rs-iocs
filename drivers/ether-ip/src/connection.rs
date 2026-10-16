//! The TCP session with one PLC.
//!
//! Port of `ether_ip.c`'s `EIPConnection` half: connect with a timeout,
//! `ListServices`, `RegisterSession`, read the Identity object, then run
//! CIP request/response transactions until shutdown.
//!
//! Blocking sockets on a dedicated thread, exactly like the C's per-PLC scan
//! task. The codec below it (`cip`, `encap`) is pure and unit-tested; this
//! module is the only part that touches a socket.

use std::io::{self, Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

use crate::cip::{self, CipClass, CipType, ParsedTag};
use crate::encap::{self, TransactionId};

#[derive(Debug, thiserror::Error)]
pub enum EipError {
    #[error("I/O: {0}")]
    Io(#[from] io::Error),
    /// The request never left: C's `EIP_send_connection_buffer` failure.
    #[error("send: {0}")]
    Send(io::Error),
    #[error("cannot resolve '{0}'")]
    Resolve(String),
    #[error("target does not support CIP PDU encapsulation")]
    NoCipSupport,
    #[error("malformed response")]
    Malformed,
    #[error("transaction id mismatch: got {got}, expected {want}")]
    TransactionMismatch { got: String, want: String },
    #[error("{command}: encapsulation status 0x{status:08X} ({text})")]
    EncapStatus {
        command: &'static str,
        status: u32,
        text: &'static str,
    },
    #[error("CIP service 0x{service:02X}: status 0x{status:02X} ({text})")]
    CipStatus {
        service: u8,
        status: u8,
        text: &'static str,
    },
    #[error("request of {size} bytes exceeds the {limit}-byte buffer limit")]
    TooLarge { size: usize, limit: usize },
}

pub type Result<T> = std::result::Result<T, EipError>;

/// Identity object attributes (`ether_ip.c:EIP_check_interface`).
#[derive(Clone, Debug, Default)]
pub struct IdentityInfo {
    pub vendor: u16,
    pub device_type: u16,
    pub revision: u16,
    pub serial_number: u32,
    pub name: String,
}

pub struct Connection {
    stream: TcpStream,
    session: u32,
    slot: u8,
    buffer_limit: usize,
    pub identity: IdentityInfo,
}

impl Connection {
    /// Connect, negotiate, and register a session. Mirrors `EIP_startup`.
    pub fn startup(
        host: &str,
        port: u16,
        slot: u8,
        timeout: Duration,
        buffer_limit: usize,
    ) -> Result<Connection> {
        let Some(addr) = (host, port)
            .to_socket_addrs()
            .ok()
            .and_then(|mut a| a.next())
        else {
            crate::eip_printf!(2, "EIP cannot find IP for '{host}'\n");
            return Err(EipError::Resolve(host.to_string()));
        };

        let stream = TcpStream::connect_timeout(&addr, timeout).inspect_err(|_| {
            crate::eip_printf!(3, "EIP cannot connect to {host}:{port}\n");
        })?;
        stream.set_read_timeout(Some(timeout))?;
        stream.set_write_timeout(Some(timeout))?;
        stream.set_nodelay(true).inspect_err(|_| {
            crate::eip_printf!(2, "EIP cannot set socket option to TCP_NODELAY\n");
        })?;

        let mut c = Connection {
            stream,
            session: 0,
            slot,
            buffer_limit,
            identity: IdentityInfo::default(),
        };

        if let Err(e) = c.list_services().and_then(|()| c.register_session()) {
            crate::eip_printf!(1, "EIP_startup: target {host} does not respond\n");
            return Err(e);
        }
        // The C treats a failed identity read as a warning, not a failure.
        match c.read_identity() {
            Ok(id) => {
                crate::eip_printf!(9, "------------------------------\n");
                crate::eip_printf!(9, "Identity information of target:\n");
                crate::eip_printf!(9, "    UINT vendor         = 0x{:04X}\n", id.vendor);
                crate::eip_printf!(9, "    UINT device_type    = 0x{:04X}\n", id.device_type);
                crate::eip_printf!(9, "    UINT revision       = 0x{:04X}\n", id.revision);
                crate::eip_printf!(9, "    UDINT serial_number = 0x{:08X}\n", id.serial_number);
                crate::eip_printf!(9, "    USINT name          = '{}'\n", id.name);
                crate::eip_printf!(9, "------------------------------\n");
                c.identity = id;
            }
            Err(_) => crate::eip_printf!(1, "EIP_startup: cannot determine target's identity\n"),
        }
        Ok(c)
    }

    pub fn session(&self) -> u32 {
        self.session
    }

    pub fn buffer_limit(&self) -> usize {
        self.buffer_limit
    }

    /// `UnRegisterSession`, best effort; the socket closes on drop either way.
    pub fn shutdown(&mut self) {
        let msg = encap::encode_unregister_session(self.session, TransactionId::generate());
        let _ = self.stream.write_all(&msg);
    }

    // -- framing ------------------------------------------------------------

    /// Send one encapsulation frame and read the reply frame, unchecked:
    /// `EIP_send_connection_buffer` then `EIP_read_connection_buffer`.
    fn transact(&mut self, request: &[u8]) -> Result<Vec<u8>> {
        if request.len() > encap::BUFFER_SIZE {
            return Err(EipError::TooLarge {
                size: request.len(),
                limit: encap::BUFFER_SIZE,
            });
        }
        self.stream.write_all(request).map_err(EipError::Send)?;

        let mut buf = vec![0u8; encap::HEADER_SIZE];
        let mut got = 0;
        while got < buf.len() {
            match self.stream.read(&mut buf[got..]) {
                Ok(0) => {
                    crate::eip_printf!(2, "EIP end-of-data after receiving {got} bytes\n");
                    return Err(io::Error::from(io::ErrorKind::UnexpectedEof).into());
                }
                Ok(n) => got += n,
                Err(e) if e.kind() == io::ErrorKind::Interrupted => {}
                Err(e) => {
                    if matches!(
                        e.kind(),
                        io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
                    ) {
                        crate::eip_printf!(2, "EIP read timeout after receiving {got} bytes\n");
                    } else {
                        crate::eip_printf!(2, "EIP end-of-data after receiving {got} bytes\n");
                    }
                    return Err(e.into());
                }
            }
            if got == encap::HEADER_SIZE && buf.len() == encap::HEADER_SIZE {
                let length = u16::from_le_bytes([buf[2], buf[3]]) as usize;
                let needed = encap::HEADER_SIZE + length;
                if needed > encap::BUFFER_SIZE {
                    crate::eip_printf!(2, "EIP response of {needed} bytes exceeds buffer\n");
                    return Err(io::Error::from(io::ErrorKind::InvalidData).into());
                }
                buf.resize(needed, 0);
            }
        }
        Ok(buf)
    }

    /// Send one encapsulation frame and read the reply frame.
    ///
    /// The C reads until it has the 24-byte header, takes `header.length`, then
    /// reads until it has that many more bytes (`EIP_read_connection_buffer`).
    fn exchange(&mut self, request: &[u8], command: u16, tid: TransactionId) -> Result<Vec<u8>> {
        let buf = self.transact(request)?;
        let header = encap::EncapHeader::decode(&buf).ok_or(EipError::Malformed)?;

        if header.command != command {
            return Err(EipError::Malformed);
        }
        if header.status != 0 {
            return Err(EipError::EncapStatus {
                command: command_name(command),
                status: header.status,
                text: encap::encap_status_text(header.status),
            });
        }
        if header.trans_id != tid {
            return Err(EipError::TransactionMismatch {
                got: header.trans_id.to_string(),
                want: tid.to_string(),
            });
        }
        Ok(buf)
    }

    /// Send an MR_Request inside a `SendRRData` and return the MR_Response.
    pub fn send_rr(&mut self, mr_request: &[u8]) -> Result<Vec<u8>> {
        let tid = TransactionId::generate();
        let frame = encap::encode_send_rr_data(self.session, tid, mr_request);
        let reply = self.exchange(&frame, encap::EC_SEND_RR_DATA, tid)?;
        let rr = encap::decode_rr_data(&reply).ok_or(EipError::Malformed)?;
        Ok(rr.response.to_vec())
    }

    /// Send `message` routed over the backplane to this PLC's slot -- i.e.
    /// wrapped in a `CM_Unconnected_Send` -- and return the MR_Response.
    pub fn send_routed(&mut self, message: &[u8]) -> Result<Vec<u8>> {
        let size = cip::unconnected_send_size(message.len());
        if size > self.buffer_limit {
            return Err(EipError::TooLarge {
                size,
                limit: self.buffer_limit,
            });
        }
        let mut req = Vec::with_capacity(size);
        cip::encode_unconnected_send(&mut req, message, self.slot);
        self.send_rr(&req)
    }

    // -- encapsulation commands ---------------------------------------------

    fn list_services(&mut self) -> Result<()> {
        let tid = TransactionId::generate();
        let reply = self
            .transact(&encap::encode_list_services(tid))
            .inspect_err(|e| match e {
                EipError::Send(_) => crate::eip_printf!(2, "EIP list_services: send failed\n"),
                _ => crate::eip_printf!(2, "EIP list_services: No response\n"),
            })?;
        let header = encap::EncapHeader::decode(&reply).ok_or(EipError::Malformed)?;
        if header.command != encap::EC_LIST_SERVICES || header.status != 0 {
            crate::eip_printf!(2, "EIP list_services: Invalid response\n");
            dump_encap_header(&header);
            return Err(EipError::Malformed);
        }
        if header.trans_id != tid {
            crate::eip_printf!(
                2,
                "EIP list_services: Transaction id mismatch, got {} expected {tid}\n",
                header.trans_id
            );
            dump_encap_header(&header);
            return Err(EipError::TransactionMismatch {
                got: header.trans_id.to_string(),
                want: tid.to_string(),
            });
        }
        let items =
            encap::decode_list_services(&reply[encap::HEADER_SIZE..]).ok_or(EipError::Malformed)?;
        crate::eip_printf!(10, "ListServices reply\n");
        crate::eip_printf!(10, "    UINT count     = {}\n", items.len());
        for i in &items {
            crate::eip_printf!(10, "    UINT type     = 0x{:04X}\n", i.item_type);
            crate::eip_printf!(10, "    UINT length   = {}\n", i.length);
            crate::eip_printf!(10, "    UINT version  = 0x{:04X}\n", i.version);
            crate::eip_printf!(10, "    UINT flags    = 0x{:04X} ", i.flags);
            if i.supports_cip() {
                crate::eip_printf!(10, "(CIP PDU encap.)\n");
            } else {
                crate::eip_printf!(
                    2,
                    "\nEIP list_services: NO SUPPORT for CIP PDU encapsulation.!\n"
                );
            }
            crate::eip_printf!(10, "    USINT name[16]= '{}'\n", i.name);
        }
        if items.is_empty() || !items.iter().all(|i| i.supports_cip()) {
            return Err(EipError::NoCipSupport);
        }
        Ok(())
    }

    fn register_session(&mut self) -> Result<()> {
        let tid = TransactionId::generate();
        let reply = self
            .transact(&encap::encode_register_session(tid))
            .inspect_err(|e| match e {
                EipError::Send(_) => crate::eip_printf!(2, "EIP register_session: send failed\n"),
                _ => crate::eip_printf!(2, "EIP register_session: no response\n"),
            })?;
        let header = encap::EncapHeader::decode(&reply).ok_or(EipError::Malformed)?;
        let verbose = crate::EIP_VERBOSITY.load(std::sync::atomic::Ordering::Relaxed) >= 3;
        if header.command != encap::EC_REGISTER_SESSION || header.status != 0 {
            crate::eip_printf!(2, "EIP register_session received error\n");
            if verbose {
                dump_encap_header(&header);
            }
            return Err(EipError::Malformed);
        }
        // Keep the session id the target handed back.
        self.session = header.session;
        if header.trans_id != tid {
            crate::eip_printf!(
                2,
                "EIP register_session: Transaction id mismatch, got {} expected {tid}\n",
                header.trans_id
            );
            if verbose {
                dump_encap_header(&header);
            }
            return Err(EipError::TransactionMismatch {
                got: header.trans_id.to_string(),
                want: tid.to_string(),
            });
        }
        Ok(())
    }

    // -- CIP services --------------------------------------------------------

    /// Unconnected `Get_Attribute_Single` -- not routed to the slot, it targets
    /// the Ethernet interface itself.
    pub fn get_attribute_single(
        &mut self,
        class: CipClass,
        instance: u32,
        attr: u8,
    ) -> Result<Vec<u8>> {
        let words = cip::cia_path_words(instance, attr);
        let mut req = Vec::with_capacity(cip::mr_request_size(words));
        req.push(cip::service::GET_ATTRIBUTE_SINGLE);
        req.push(words as u8);
        cip::encode_cia_path(&mut req, class, instance, attr);

        let response = self.send_rr(&req)?;
        let r = cip::MrResponse::parse(&response).ok_or(EipError::Malformed)?;
        if r.service & 0x7F != cip::service::GET_ATTRIBUTE_SINGLE || !r.is_ok() {
            return Err(EipError::CipStatus {
                service: cip::service::GET_ATTRIBUTE_SINGLE,
                status: r.general_status,
                text: cip::cip_error_text(r.general_status),
            });
        }
        Ok(r.data(response.len()).to_vec())
    }

    /// A routed request for a diagnostic, with C's `<who>: send failed` /
    /// `No response` / `Transaction id mismatch` lines; None after printing.
    fn diagnostic_request(&mut self, who: &str, msg: &[u8]) -> Option<Vec<u8>> {
        match self.send_routed(msg) {
            Ok(r) => Some(r),
            Err(EipError::Send(_) | EipError::TooLarge { .. }) => {
                crate::eip_printf!(1, "{who}: send failed\n");
                None
            }
            Err(EipError::TransactionMismatch { got, want }) => {
                crate::eip_printf!(
                    1,
                    "{who}: Transaction id mismatch, got {got} expected {want}\n"
                );
                None
            }
            Err(_) => {
                crate::eip_printf!(1, "{who}: No response\n");
                None
            }
        }
    }

    /// `EIP_describe_type`: print a structure type's template.
    pub fn describe_type(&mut self, type_id: u16) -> bool {
        const WHO: &str = "EIP_describe_type";
        crate::eip_printf!(9, "EIP describe type 0x{type_id:04X}\n");
        let instance = u32::from(type_id);
        let words = cip::cia_path_words(instance, 0);

        let mut msg = Vec::new();
        msg.push(cip::service::GET_ATTRIBUTE_LIST);
        msg.push(words as u8);
        cip::encode_cia_path(&mut msg, CipClass::Template, instance, 0);
        for v in [4u16, 4, 5, 2, 1] {
            msg.extend_from_slice(&v.to_le_bytes());
        }
        let Some(response) = self.diagnostic_request(WHO, &msg) else {
            return false;
        };
        let Some(r) = cip::MrResponse::parse(&response) else {
            crate::eip_printf!(1, "{WHO}: No response\n");
            return false;
        };
        if r.service & 0x7F != cip::service::GET_ATTRIBUTE_LIST {
            crate::eip_printf!(
                1,
                "{WHO}: Got response 0x{:X}, not S_Get_Attribute_List\n",
                r.service
            );
            return false;
        }
        if r.general_status != 0 {
            crate::eip_printf!(
                1,
                "Error in S_Get_Instance_Attr_List response ({})\n",
                cip::cip_error_text(r.general_status)
            );
            return false;
        }
        let data = r.data(response.len());
        let attr_count = cip::rd_u16(data, 0).unwrap_or(0);
        if attr_count != 4 {
            crate::eip_printf!(
                1,
                "Expected 4 attributes from S_Get_Instance_Attr_List response, got {attr_count}\n"
            );
            return false;
        }
        let (mut handle, mut members, mut def_words, mut struct_size) = (0u16, 0u16, 0u32, 0u32);
        let mut at = 2;
        for _ in 0..attr_count {
            let attr = cip::rd_u16(data, at).unwrap_or(0);
            let status = cip::rd_u16(data, at + 2).unwrap_or(0);
            at += 4;
            if status != 0 {
                crate::eip_printf!(
                    1,
                    "Status for S_Get_Instance_Attr_List attr. {attr} is 0x{status:04X}\n"
                );
                return false;
            }
            match attr {
                1 => handle = cip::rd_u16(data, at).unwrap_or(0),
                2 => members = cip::rd_u16(data, at).unwrap_or(0),
                4 => def_words = cip::rd_u32(data, at).unwrap_or(0),
                5 => struct_size = cip::rd_u32(data, at).unwrap_or(0),
                _ => {
                    crate::eip_printf!(1, "Unknown S_Get_Instance_Attr_List attr. {attr}\n");
                    return false;
                }
            }
            at += if attr <= 2 { 2 } else { 4 };
        }
        crate::eip_printf!(1, "Type ID            : 0x{handle:04X}\n");
        crate::eip_printf!(1, "Struct member count: {members}\n");
        crate::eip_printf!(1, "Template word size : {def_words}\n");
        crate::eip_printf!(1, "Data byte count    : {struct_size}\n");

        // Template bytes less the 23-byte header (Logix Data Access p.54).
        let template_bytes = def_words.wrapping_mul(4).wrapping_sub(23);
        let mut msg = Vec::new();
        msg.push(cip::service::TEMPLATE_READ_DATA);
        msg.push(words as u8);
        cip::encode_cia_path(&mut msg, CipClass::Template, instance, 0);
        msg.extend_from_slice(&0u32.to_le_bytes());
        msg.extend_from_slice(&(template_bytes as u16).to_le_bytes());
        let Some(response) = self.diagnostic_request(WHO, &msg) else {
            return false;
        };
        let Some(r) = cip::MrResponse::parse(&response) else {
            crate::eip_printf!(1, "{WHO}: No response\n");
            return false;
        };
        if r.service & 0x7F != cip::service::TEMPLATE_READ_DATA {
            crate::eip_printf!(
                1,
                "{WHO}: Got response 0x{:X}, not S_Template_ReadData\n",
                r.service
            );
            return false;
        }
        if r.general_status != 0 {
            crate::eip_printf!(1, "Error in S_Template_ReadData response\n");
            return false;
        }
        let data = r.data(response.len());
        let members: Vec<(u16, u16, u32)> = (0..usize::from(members))
            .map(|i| {
                let at = 8 * i;
                (
                    cip::rd_u16(data, at).unwrap_or(0),
                    cip::rd_u16(data, at + 2).unwrap_or(0),
                    cip::rd_u32(data, at + 4).unwrap_or(0),
                )
            })
            .collect();
        let mut strings = data
            .get(8 * members.len()..)
            .unwrap_or(&[])
            .split(|&c| c == 0)
            .map(String::from_utf8_lossy);
        let name = strings.next().unwrap_or_default();
        // The struct name ends at ';', the rest is undefined.
        let name = name.split(';').next().unwrap_or_default();
        crate::eip_printf!(1, "Structure name: '{name}'\n");
        for (i, (info, t, offset)) in members.iter().enumerate() {
            let member = strings.next().unwrap_or_default();
            crate::eip_printf!(
                1,
                "{:2}) offset 0x{offset:08X} '{member}', {}\n",
                i + 1,
                cip::decode_type_and_info(*t, *info)
            );
        }
        true
    }

    fn read_identity(&mut self) -> Result<IdentityInfo> {
        let u16_attr = |c: &mut Self, a: u8| -> Result<u16> {
            let d = c.get_attribute_single(CipClass::Identity, 1, a)?;
            if d.len() < 2 {
                return Err(EipError::Malformed);
            }
            Ok(u16::from_le_bytes([d[0], d[1]]))
        };

        let vendor = u16_attr(self, 1)?;
        let device_type = u16_attr(self, 2)?;
        let revision = u16_attr(self, 4)?;

        let d = self.get_attribute_single(CipClass::Identity, 1, 6)?;
        if d.len() < 4 {
            return Err(EipError::Malformed);
        }
        let serial_number = u32::from_le_bytes([d[0], d[1], d[2], d[3]]);

        // Attribute 7 is a SHORT_STRING: a length byte then the characters.
        let d = self.get_attribute_single(CipClass::Identity, 1, 7)?;
        let len = *d.first().ok_or(EipError::Malformed)? as usize;
        let text = d.get(1..1 + len).ok_or(EipError::Malformed)?;

        Ok(IdentityInfo {
            vendor,
            device_type,
            revision,
            serial_number,
            name: String::from_utf8_lossy(text).into_owned(),
        })
    }

    /// Read one tag in one request. Returns the raw `[type][data]` buffer.
    pub fn read_tag(&mut self, tag: &ParsedTag, elements: u16) -> Result<Vec<u8>> {
        let mut msg = Vec::with_capacity(cip::read_data_size(tag));
        cip::encode_read_data(&mut msg, tag, elements);

        let response = self.send_routed(&msg)?;
        let n = response.len();
        cip::check_read_data_response(&response, n)
            .map(|d| d.to_vec())
            .ok_or_else(|| cip_error(&response, cip::service::CIP_READ_DATA))
    }

    /// Write one tag in one request. `raw_data` excludes the leading type code.
    pub fn write_tag(
        &mut self,
        tag: &ParsedTag,
        cip_type: CipType,
        elements: u16,
        raw_data: &[u8],
    ) -> Result<()> {
        let mut msg = Vec::new();
        if cip_type == CipType::Struct {
            cip::encode_write_string(&mut msg, tag, elements, raw_data, self.buffer_limit);
        } else {
            cip::encode_write_data(&mut msg, tag, cip_type, elements, raw_data);
        }

        let response = self.send_routed(&msg)?;
        if cip::check_write_data_response(&response) {
            Ok(())
        } else {
            Err(cip_error(&response, cip::service::CIP_WRITE_DATA))
        }
    }
}

/// Turn a failed MR_Response into a typed error, so the caller does not have to
/// re-parse it.
fn cip_error(response: &[u8], service: u8) -> EipError {
    match cip::MrResponse::parse(response) {
        Some(r) => EipError::CipStatus {
            service,
            status: r.general_status,
            text: cip::cip_error_text(r.general_status),
        },
        None => EipError::Malformed,
    }
}

/// `dump_EncapsulationHeader`: printed at level 0, i.e. always.
fn dump_encap_header(h: &encap::EncapHeader) {
    crate::eip_printf!(0, "EncapsulationHeader:\n");
    crate::eip_printf!(
        0,
        "    UINT  command   = 0x{:02X} ({})\n",
        h.command,
        command_name(h.command)
    );
    crate::eip_printf!(0, "    UINT  length    = {} \n", h.length);
    crate::eip_printf!(0, "    UDINT session   = 0x{:08X}\n", h.session);
    crate::eip_printf!(
        0,
        "    UDINT status    = 0x{:08X}  ({})\n",
        h.status,
        encap::encap_status_text(h.status)
    );
    crate::eip_printf!(0, "    USINT context[8]= '{}'\n", h.trans_id);
    crate::eip_printf!(0, "    UDINT options   = 0x{:08X}\n", h.options);
}

/// `EncapsulationHeader_command`.
fn command_name(command: u16) -> &'static str {
    match command {
        0x0000 => "Nop\n",
        0x0064 => "ListInterfaces",
        0x0070 => "SendUnitData",
        encap::EC_LIST_SERVICES => "ListServices",
        encap::EC_REGISTER_SESSION => "RegisterSession",
        encap::EC_UNREGISTER_SESSION => "UnRegisterSession",
        encap::EC_SEND_RR_DATA => "SendRRData",
        _ => "<unknown>",
    }
}

impl Drop for Connection {
    fn drop(&mut self) {
        if self.session != 0 {
            self.shutdown();
        }
    }
}
