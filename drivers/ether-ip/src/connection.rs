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
use std::net::{SocketAddr, TcpStream, ToSocketAddrs};
use std::time::Duration;

use crate::cip::{self, CipClass, CipType, ParsedTag};
use crate::encap::{self, TransactionId};

#[derive(Debug, thiserror::Error)]
pub enum EipError {
    #[error("I/O: {0}")]
    Io(#[from] io::Error),
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
        let addr: SocketAddr = (host, port)
            .to_socket_addrs()
            .map_err(|_| EipError::Resolve(host.to_string()))?
            .next()
            .ok_or_else(|| EipError::Resolve(host.to_string()))?;

        let stream = TcpStream::connect_timeout(&addr, timeout)?;
        stream.set_read_timeout(Some(timeout))?;
        stream.set_write_timeout(Some(timeout))?;
        stream.set_nodelay(true)?;

        let mut c = Connection {
            stream,
            session: 0,
            slot,
            buffer_limit,
            identity: IdentityInfo::default(),
        };

        c.list_services()?;
        c.register_session()?;
        // The C treats a failed identity read as a warning, not a failure.
        match c.read_identity() {
            Ok(id) => c.identity = id,
            Err(e) => log::warn!("EIP {host}: cannot determine target's identity: {e}"),
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

    /// Send one encapsulation frame and read the reply frame.
    ///
    /// The C reads until it has the 24-byte header, takes `header.length`, then
    /// reads until it has that many more bytes (`EIP_read_connection_buffer`).
    fn exchange(&mut self, request: &[u8], command: u16, tid: TransactionId) -> Result<Vec<u8>> {
        if request.len() > encap::BUFFER_SIZE {
            return Err(EipError::TooLarge {
                size: request.len(),
                limit: encap::BUFFER_SIZE,
            });
        }
        self.stream.write_all(request)?;

        let mut buf = vec![0u8; encap::HEADER_SIZE];
        self.stream.read_exact(&mut buf)?;
        let header = encap::EncapHeader::decode(&buf).ok_or(EipError::Malformed)?;

        let body = header.length as usize;
        if encap::HEADER_SIZE + body > encap::BUFFER_SIZE {
            return Err(EipError::TooLarge {
                size: encap::HEADER_SIZE + body,
                limit: encap::BUFFER_SIZE,
            });
        }
        buf.resize(encap::HEADER_SIZE + body, 0);
        self.stream.read_exact(&mut buf[encap::HEADER_SIZE..])?;

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
        let reply = self.exchange(
            &encap::encode_list_services(tid),
            encap::EC_LIST_SERVICES,
            tid,
        )?;
        let items =
            encap::decode_list_services(&reply[encap::HEADER_SIZE..]).ok_or(EipError::Malformed)?;
        if items.is_empty() || !items.iter().all(|i| i.supports_cip()) {
            return Err(EipError::NoCipSupport);
        }
        for i in &items {
            log::debug!(
                "EIP service '{}' type 0x{:04X} version 0x{:04X}",
                i.name,
                i.item_type,
                i.version
            );
        }
        Ok(())
    }

    fn register_session(&mut self) -> Result<()> {
        let tid = TransactionId::generate();
        let reply = self.exchange(
            &encap::encode_register_session(tid),
            encap::EC_REGISTER_SESSION,
            tid,
        )?;
        let header = encap::EncapHeader::decode(&reply).ok_or(EipError::Malformed)?;
        // Keep the session id the target handed back.
        self.session = header.session;
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

fn command_name(command: u16) -> &'static str {
    match command {
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
