//! Blocking ADS client: one TCP connection to the PLC's AMS router.
//!
//! Replaces Beckhoff's `AmsRouter`/`AmsConnection`/`NotificationDispatcher`.
//! Only what the driver needs is here: one target system, one socket, request
//! correlation by invoke id, and a notification callback.
//!
//! `AdsAddRoute` in the standalone AdsLib is purely client-side — it resolves
//! the host and opens the TCP connection. The *PLC-side* route must be added by
//! hand in TwinCAT (`Systems → Routes → Add route`), which is why there is no
//! route-registration frame here: none exists on the wire.

use std::collections::HashMap;
use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender, channel};
use std::thread;
use std::time::Duration;

use parking_lot::Mutex;

use super::defs::*;
use super::error::{AdsError, check};
use super::frame::*;
use super::notification::{NotificationSample, decode_notification};
use super::sumup::{self, SumEntry};
use super::symbol::{SymbolEntry, decode_symbol_entry};

/// Called on the reader thread for every notified sample.
pub type NotificationHandler = Arc<dyn Fn(NotificationSample) + Send + Sync>;
/// Called on the reader thread when the socket dies.
pub type DisconnectHandler = Arc<dyn Fn() + Send + Sync>;

/// One in-flight request's reply slot.
type ResponseSender = Sender<Result<Vec<u8>, AdsError>>;

struct Shared {
    /// Write half. `None` once the connection is torn down.
    tx: Mutex<Option<TcpStream>>,
    /// In-flight requests, keyed by invoke id.
    pending: Mutex<HashMap<u32, ResponseSender>>,
    next_invoke_id: AtomicU32,
    connected: AtomicBool,
    local: AmsAddr,
    target_net_id: AmsNetId,
    timeout: Duration,
}

impl Shared {
    /// Fail every in-flight request and mark the client down. Idempotent: the
    /// reader thread and an explicit `close()` can both reach it.
    fn tear_down(&self) {
        self.connected.store(false, Ordering::SeqCst);
        if let Some(s) = self.tx.lock().take() {
            let _ = s.shutdown(std::net::Shutdown::Both);
        }
        // Waking every waiter is the whole point: a request blocked on `recv`
        // must not sit there for its full timeout after the socket is gone.
        for (_, waiter) in self.pending.lock().drain() {
            let _ = waiter.send(Err(AdsError::NotConnected));
        }
    }
}

/// A connected ADS client.
pub struct AdsClient {
    shared: Arc<Shared>,
    reader: Option<thread::JoinHandle<()>>,
}

impl AdsClient {
    /// Open the TCP connection to the AMS router and start the reader thread.
    ///
    /// `local` is this IOC's AMS address; `target_net_id` is the PLC's. The
    /// AMS port of each request is passed per call, because one driver serves
    /// several PLC runtimes (851, 852, …) over the same socket.
    /// A zero `local.net_id` means "derive it", exactly as Beckhoff's router
    /// does when the application never called `AdsSetLocalAddress`: the local
    /// end of the socket, plus `.1.1` (see [`AmsNetId::from_ipv4`]).
    pub fn connect(
        host: &str,
        local: AmsAddr,
        target_net_id: AmsNetId,
        timeout: Duration,
        on_notification: NotificationHandler,
        on_disconnect: DisconnectHandler,
    ) -> Result<Self, AdsError> {
        let addr = (host, ADS_TCP_SERVER_PORT)
            .to_socket_addrs()?
            .next()
            .ok_or_else(|| {
                std::io::Error::new(
                    std::io::ErrorKind::NotFound,
                    format!("cannot resolve PLC host '{host}'"),
                )
            })?;
        let stream = TcpStream::connect_timeout(&addr, timeout)?;
        stream.set_nodelay(true)?;

        let mut local = local;
        if local.net_id.is_zero() {
            match stream.local_addr()? {
                std::net::SocketAddr::V4(v4) => {
                    local.net_id = AmsNetId::from_ipv4(*v4.ip());
                }
                std::net::SocketAddr::V6(_) => {
                    return Err(AdsError::Io(std::io::Error::new(
                        std::io::ErrorKind::AddrNotAvailable,
                        "no local AMS Net Id configured and the socket is IPv6, \
                         which carries no address to derive one from — \
                         call adsSetLocalAddress",
                    )));
                }
            }
        }

        Self::from_stream(
            stream,
            local,
            target_net_id,
            timeout,
            on_notification,
            on_disconnect,
        )
    }

    /// This client's AMS address (with the derived Net Id, once connected).
    pub fn local_addr(&self) -> AmsAddr {
        self.shared.local
    }

    /// Build a client over an already-connected socket (used by the tests'
    /// in-process PLC).
    pub fn from_stream(
        stream: TcpStream,
        local: AmsAddr,
        target_net_id: AmsNetId,
        timeout: Duration,
        on_notification: NotificationHandler,
        on_disconnect: DisconnectHandler,
    ) -> Result<Self, AdsError> {
        let rx_stream = stream.try_clone()?;
        let shared = Arc::new(Shared {
            tx: Mutex::new(Some(stream)),
            pending: Mutex::new(HashMap::new()),
            next_invoke_id: AtomicU32::new(1),
            connected: AtomicBool::new(true),
            local,
            target_net_id,
            timeout,
        });

        let reader_shared = Arc::clone(&shared);
        let reader = thread::Builder::new()
            .name("ads-rx".into())
            .spawn(move || {
                read_loop(&reader_shared, rx_stream, &on_notification);
                reader_shared.tear_down();
                on_disconnect();
            })
            .map_err(AdsError::Io)?;

        Ok(Self {
            shared,
            reader: Some(reader),
        })
    }

    pub fn is_connected(&self) -> bool {
        self.shared.connected.load(Ordering::SeqCst)
    }

    /// Close the socket and wait for the reader thread to finish.
    pub fn close(&mut self) {
        self.shared.tear_down();
        if let Some(h) = self.reader.take() {
            let _ = h.join();
        }
    }

    /// Send one AoE request and block for its response payload.
    ///
    /// The returned bytes are the payload after the AoE header, starting with
    /// the response's `result` word — which is *not* pre-checked here, because
    /// each command's decoder knows where its result field sits.
    fn request(&self, ams_port: u16, cmd_id: u16, payload: &[u8]) -> Result<Vec<u8>, AdsError> {
        if !self.is_connected() {
            return Err(AdsError::NotConnected);
        }
        let invoke_id = self.shared.next_invoke_id.fetch_add(1, Ordering::Relaxed);
        let target = AmsAddr {
            net_id: self.shared.target_net_id,
            port: ams_port,
        };
        let header = AoeHeader::request(
            target,
            self.shared.local,
            cmd_id,
            payload.len() as u32,
            invoke_id,
        );
        let frame = encode_frame(&header, payload);

        let (tx, rx): (Sender<_>, Receiver<_>) = channel();
        // Register before sending: the response can land on the reader thread
        // before `write_all` even returns.
        self.shared.pending.lock().insert(invoke_id, tx);

        let send = {
            let mut guard = self.shared.tx.lock();
            match guard.as_mut() {
                Some(s) => s.write_all(&frame).map_err(AdsError::Io),
                None => Err(AdsError::NotConnected),
            }
        };
        if let Err(e) = send {
            self.shared.pending.lock().remove(&invoke_id);
            return Err(e);
        }

        match rx.recv_timeout(self.shared.timeout) {
            Ok(result) => result,
            Err(RecvTimeoutError::Timeout) => {
                // Drop the slot, else a late response leaks it forever.
                self.shared.pending.lock().remove(&invoke_id);
                Err(AdsError::Timeout)
            }
            // Sender dropped without answering: the reader thread died.
            Err(RecvTimeoutError::Disconnected) => Err(AdsError::NotConnected),
        }
    }

    /// ADS Read (`indexGroup`/`indexOffset`) of exactly `length` bytes.
    pub fn read(
        &self,
        ams_port: u16,
        index_group: u32,
        index_offset: u32,
        length: u32,
    ) -> Result<Vec<u8>, AdsError> {
        let mut payload = Vec::with_capacity(12);
        push_request_header(&mut payload, index_group, index_offset, length);
        let resp = self.request(ams_port, CMD_READ, &payload)?;
        decode_read_response(&resp, length as usize)
    }

    /// ADS Write.
    pub fn write(
        &self,
        ams_port: u16,
        index_group: u32,
        index_offset: u32,
        data: &[u8],
    ) -> Result<(), AdsError> {
        let mut payload = Vec::with_capacity(12 + data.len());
        push_request_header(&mut payload, index_group, index_offset, data.len() as u32);
        payload.extend_from_slice(data);
        let resp = self.request(ams_port, CMD_WRITE, &payload)?;
        let mut r = Reader::new(&resp);
        check(r.u32()?)
    }

    /// ADS ReadWrite: write `data`, read back up to `read_length` bytes.
    pub fn read_write(
        &self,
        ams_port: u16,
        index_group: u32,
        index_offset: u32,
        read_length: u32,
        data: &[u8],
    ) -> Result<Vec<u8>, AdsError> {
        let mut payload = Vec::with_capacity(16 + data.len());
        push_read_write_header(
            &mut payload,
            index_group,
            index_offset,
            read_length,
            data.len() as u32,
        );
        payload.extend_from_slice(data);
        let resp = self.request(ams_port, CMD_READ_WRITE, &payload)?;
        // A sum-up read returns *fewer* bytes than requested when a sub-request
        // fails, so this decoder must not demand the full count.
        decode_read_response_lenient(&resp)
    }

    /// Sum-up read: fetch many variables in one round trip.
    ///
    /// Returns the raw response; split it with [`sumup::decode_response`] using
    /// the same `entries` slice. The bytes are returned rather than the decoded
    /// slices because the results borrow from them.
    pub fn sum_up_read(&self, ams_port: u16, entries: &[SumEntry]) -> Result<Vec<u8>, AdsError> {
        let req = sumup::build_request(entries);
        self.read_write(
            ams_port,
            req.index_group,
            req.index_offset,
            req.read_length,
            &req.payload,
        )
    }

    /// ADS ReadState → (ads state, device state).
    pub fn read_state(&self, ams_port: u16) -> Result<(AdsState, u16), AdsError> {
        let resp = self.request(ams_port, CMD_READ_STATE, &[])?;
        let mut r = Reader::new(&resp);
        check(r.u32()?)?;
        let ads = AdsState::from_u16(r.u16()?);
        let device = r.u16()?;
        Ok((ads, device))
    }

    /// ADS WriteControl — command the PLC runtime into an ADS state
    /// (C `adsWriteState`, `AdsSyncWriteControlReqEx`).
    pub fn write_control(
        &self,
        ams_port: u16,
        ads_state: u16,
        device_state: u16,
        data: &[u8],
    ) -> Result<(), AdsError> {
        let mut payload = Vec::with_capacity(8 + data.len());
        payload.extend_from_slice(&ads_state.to_le_bytes());
        payload.extend_from_slice(&device_state.to_le_bytes());
        payload.extend_from_slice(&(data.len() as u32).to_le_bytes());
        payload.extend_from_slice(data);
        let resp = self.request(ams_port, CMD_WRITE_CONTROL, &payload)?;
        let mut r = Reader::new(&resp);
        check(r.u32()?)
    }

    /// ADS ReadDeviceInfo → (name, version).
    pub fn read_device_info(&self, ams_port: u16) -> Result<(String, AdsVersion), AdsError> {
        let resp = self.request(ams_port, CMD_READ_DEVICE_INFO, &[])?;
        let mut r = Reader::new(&resp);
        check(r.u32()?)?;
        let version = AdsVersion {
            version: r.u8()?,
            revision: r.u8()?,
            build: r.u16()?,
        };
        // Fixed 16-byte NUL-padded device name.
        let raw = r.bytes(16)?;
        let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
        let name = String::from_utf8_lossy(&raw[..end]).into_owned();
        Ok((name, version))
    }

    /// Resolve a symbol name to a handle (`SYM_HNDBYNAME`).
    pub fn get_symbol_handle(&self, ams_port: u16, name: &str) -> Result<u32, AdsError> {
        let data = self.read_write(ams_port, ADSIGRP_SYM_HNDBYNAME, 0, 4, name.as_bytes())?;
        let mut r = Reader::new(&data);
        r.u32()
    }

    /// Release a symbol handle (`SYM_RELEASEHND`).
    pub fn release_symbol_handle(&self, ams_port: u16, handle: u32) -> Result<(), AdsError> {
        self.write(ams_port, ADSIGRP_SYM_RELEASEHND, 0, &handle.to_le_bytes())
    }

    /// Read a symbol's type/size/address (`SYM_INFOBYNAMEEX`).
    pub fn get_symbol_info(&self, ams_port: u16, name: &str) -> Result<SymbolEntry, AdsError> {
        // The C driver reads into a fixed `AdsSymbolEntry + 3 * field` buffer;
        // 1024 bytes comfortably covers header + name + type + comment.
        let data = self.read_write(ams_port, ADSIGRP_SYM_INFOBYNAMEEX, 0, 1024, name.as_bytes())?;
        decode_symbol_entry(&data)
    }

    /// Read a value by symbol handle (`SYM_VALBYHND`).
    pub fn read_by_handle(
        &self,
        ams_port: u16,
        handle: u32,
        length: u32,
    ) -> Result<Vec<u8>, AdsError> {
        self.read(ams_port, ADSIGRP_SYM_VALBYHND, handle, length)
    }

    /// Write a value by symbol handle (`SYM_VALBYHND`).
    pub fn write_by_handle(&self, ams_port: u16, handle: u32, data: &[u8]) -> Result<(), AdsError> {
        self.write(ams_port, ADSIGRP_SYM_VALBYHND, handle, data)
    }

    /// Subscribe to on-change notifications for one variable.
    ///
    /// `cycle_time` and `max_delay` are given in seconds and converted to the
    /// 100 ns units the wire uses.
    pub fn add_notification(
        &self,
        ams_port: u16,
        index_group: u32,
        index_offset: u32,
        length: u32,
        cycle_time: Duration,
        max_delay: Duration,
    ) -> Result<u32, AdsError> {
        let mut payload = Vec::with_capacity(40);
        push_add_notification_request(
            &mut payload,
            index_group,
            index_offset,
            length,
            ADSTRANS_SERVERONCHA,
            duration_to_100ns(max_delay),
            duration_to_100ns(cycle_time),
        );
        let resp = self.request(ams_port, CMD_ADD_DEVICE_NOTIFICATION, &payload)?;
        let mut r = Reader::new(&resp);
        check(r.u32()?)?;
        r.u32()
    }

    /// Cancel a notification subscription.
    pub fn del_notification(&self, ams_port: u16, handle: u32) -> Result<(), AdsError> {
        let resp = self.request(ams_port, CMD_DEL_DEVICE_NOTIFICATION, &handle.to_le_bytes())?;
        let mut r = Reader::new(&resp);
        check(r.u32()?)
    }
}

impl Drop for AdsClient {
    fn drop(&mut self) {
        self.close();
    }
}

/// Saturating conversion of a duration to FILETIME units.
///
/// The wire field is `u32`, so the largest expressible cycle time is ~429 s;
/// anything longer clamps rather than wrapping to a near-zero cycle time, which
/// would flood the IOC with notifications.
fn duration_to_100ns(d: Duration) -> u32 {
    let ticks = d.as_nanos() / 100;
    ticks.min(u32::MAX as u128) as u32
}

/// `AoEReadResponseHeader` + exactly `expected` data bytes.
fn decode_read_response(resp: &[u8], expected: usize) -> Result<Vec<u8>, AdsError> {
    let mut r = Reader::new(resp);
    check(r.u32()?)?;
    let read_len = r.u32()? as usize;
    if read_len < expected {
        return Err(AdsError::ShortRead {
            need: expected,
            got: read_len,
        });
    }
    Ok(r.bytes(read_len)?.to_vec())
}

/// `AoEReadResponseHeader` + however many bytes the PLC actually returned.
fn decode_read_response_lenient(resp: &[u8]) -> Result<Vec<u8>, AdsError> {
    let mut r = Reader::new(resp);
    check(r.u32()?)?;
    let read_len = r.u32()? as usize;
    Ok(r.bytes(read_len)?.to_vec())
}

/// Reader thread: demultiplex responses and notifications until the socket ends.
fn read_loop(shared: &Arc<Shared>, mut stream: TcpStream, on_notification: &NotificationHandler) {
    let mut tcp_header = [0u8; AMS_TCP_HEADER_LEN];
    loop {
        if stream.read_exact(&mut tcp_header).is_err() {
            return; // EOF or socket error — the caller tears down.
        }
        let Ok(len) = decode_tcp_header(&tcp_header) else {
            return;
        };
        if (len as usize) < AOE_HEADER_LEN {
            return; // Desynchronized stream; there is no safe resync point.
        }
        let mut body = vec![0u8; len as usize];
        if stream.read_exact(&mut body).is_err() {
            return;
        }
        let Ok(header) = AoeHeader::decode(&body) else {
            return;
        };
        let payload = &body[AOE_HEADER_LEN..];

        if header.cmd_id == CMD_DEVICE_NOTIFICATION {
            // Notifications are unsolicited: no invoke id to correlate.
            if let Ok(samples) = decode_notification(payload) {
                for s in samples {
                    on_notification(s);
                }
            }
            continue;
        }

        let Some(waiter) = shared.pending.lock().remove(&header.invoke_id) else {
            continue; // Late response to a timed-out request.
        };
        // A transport-level failure is reported in the AoE header; the payload
        // is then meaningless, so it must win over the payload's result word.
        let result = if header.error_code != 0 {
            Err(AdsError::Ads(header.error_code))
        } else {
            Ok(payload.to_vec())
        };
        let _ = waiter.send(result);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;
    use std::sync::mpsc;

    const PLC_NET: AmsNetId = AmsNetId([192, 168, 88, 44, 1, 1]);
    const LOCAL_NET: AmsNetId = AmsNetId([10, 0, 0, 5, 1, 1]);

    fn local() -> AmsAddr {
        AmsAddr {
            net_id: LOCAL_NET,
            port: LOCAL_PORT_BASE,
        }
    }

    /// A request the fake PLC received, decoded.
    struct Received {
        header: AoeHeader,
        payload: Vec<u8>,
    }

    /// In-process ADS server. `respond` maps a request to zero or more frames
    /// to send back, letting a test drive notifications as well as replies.
    struct FakePlc {
        stream: TcpStream,
        requests: mpsc::Sender<Received>,
    }

    impl FakePlc {
        fn read_request(&mut self) -> Option<Received> {
            let mut tcp = [0u8; AMS_TCP_HEADER_LEN];
            self.stream.read_exact(&mut tcp).ok()?;
            let len = decode_tcp_header(&tcp).ok()? as usize;
            let mut body = vec![0u8; len];
            self.stream.read_exact(&mut body).ok()?;
            let header = AoeHeader::decode(&body).ok()?;
            let payload = body[AOE_HEADER_LEN..].to_vec();
            let rec = Received {
                header,
                payload: payload.clone(),
            };
            let _ = self.requests.send(Received { header, payload });
            Some(rec)
        }

        fn reply(&mut self, req: &AoeHeader, error_code: u32, payload: &[u8]) {
            let header = AoeHeader {
                target: req.source,
                source: req.target,
                cmd_id: req.cmd_id,
                state_flags: STATE_FLAG_RESPONSE,
                length: payload.len() as u32,
                error_code,
                invoke_id: req.invoke_id,
            };
            let _ = self.stream.write_all(&encode_frame(&header, payload));
        }

        fn push_notification(&mut self, target: AmsAddr, payload: &[u8]) {
            let header = AoeHeader {
                target,
                source: AmsAddr {
                    net_id: PLC_NET,
                    port: 851,
                },
                cmd_id: CMD_DEVICE_NOTIFICATION,
                state_flags: STATE_FLAG_REQUEST,
                length: payload.len() as u32,
                error_code: 0,
                invoke_id: 0,
            };
            let _ = self.stream.write_all(&encode_frame(&header, payload));
        }
    }

    /// What [`harness`] hands back: the client, the requests the fake PLC saw,
    /// and the notifications the client's handler received.
    type Harness = (
        AdsClient,
        mpsc::Receiver<Received>,
        mpsc::Receiver<NotificationSample>,
    );

    /// Spin up a fake PLC on localhost and a client connected to it.
    fn harness<F>(plc: F) -> Harness
    where
        F: FnOnce(FakePlc) + Send + 'static,
    {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let (req_tx, req_rx) = mpsc::channel();

        thread::spawn(move || {
            let (stream, _) = listener.accept().unwrap();
            plc(FakePlc {
                stream,
                requests: req_tx,
            });
        });

        let stream = TcpStream::connect(addr).unwrap();
        let (note_tx, note_rx) = mpsc::channel();
        let client = AdsClient::from_stream(
            stream,
            local(),
            PLC_NET,
            Duration::from_millis(500),
            Arc::new(move |s| {
                let _ = note_tx.send(s);
            }),
            Arc::new(|| {}),
        )
        .unwrap();
        (client, req_rx, note_rx)
    }

    fn ok_read_payload(data: &[u8]) -> Vec<u8> {
        let mut p = 0u32.to_le_bytes().to_vec(); // result
        p.extend_from_slice(&(data.len() as u32).to_le_bytes()); // readLength
        p.extend_from_slice(data);
        p
    }

    #[test]
    fn read_sends_request_header_and_returns_data() {
        let (client, reqs, _) = harness(|mut plc| {
            let r = plc.read_request().unwrap();
            plc.reply(&r.header, 0, &ok_read_payload(&42i32.to_le_bytes()));
        });

        let data = client.read(851, ADSIGRP_SYM_VALBYHND, 7, 4).unwrap();
        assert_eq!(data, 42i32.to_le_bytes());

        let req = reqs.recv().unwrap();
        assert_eq!(req.header.cmd_id, CMD_READ);
        assert_eq!(req.header.state_flags, STATE_FLAG_REQUEST);
        assert_eq!(req.header.target.net_id, PLC_NET);
        assert_eq!(req.header.target.port, 851);
        assert_eq!(req.header.source, local());
        let mut r = Reader::new(&req.payload);
        assert_eq!(r.u32().unwrap(), ADSIGRP_SYM_VALBYHND);
        assert_eq!(r.u32().unwrap(), 7);
        assert_eq!(r.u32().unwrap(), 4);
    }

    #[test]
    fn ads_error_in_payload_result_surfaces() {
        let (client, _reqs, _) = harness(|mut plc| {
            let r = plc.read_request().unwrap();
            let mut p = 0x0710u32.to_le_bytes().to_vec(); // SYMBOLNOTFOUND
            p.extend_from_slice(&0u32.to_le_bytes());
            plc.reply(&r.header, 0, &p);
        });
        let err = client.read(851, ADSIGRP_SYM_VALBYHND, 7, 4).unwrap_err();
        assert_eq!(err.code(), Some(0x0710));
    }

    #[test]
    fn ads_error_in_aoe_header_surfaces() {
        // Transport-level failure: the header carries the error and the payload
        // is empty, so decoding the payload's result word would itself fail.
        let (client, _reqs, _) = harness(|mut plc| {
            let r = plc.read_request().unwrap();
            plc.reply(&r.header, GLOBALERR_TARGET_PORT, &[]);
        });
        let err = client.read(851, ADSIGRP_SYM_VALBYHND, 7, 4).unwrap_err();
        assert_eq!(err.code(), Some(GLOBALERR_TARGET_PORT));
    }

    #[test]
    fn short_read_is_rejected() {
        let (client, _reqs, _) = harness(|mut plc| {
            let r = plc.read_request().unwrap();
            plc.reply(&r.header, 0, &ok_read_payload(&[1, 2])); // asked for 4
        });
        assert!(matches!(
            client.read(851, ADSIGRP_SYM_VALBYHND, 7, 4),
            Err(AdsError::ShortRead { need: 4, got: 2 })
        ));
    }

    #[test]
    fn responses_are_correlated_by_invoke_id_out_of_order() {
        // The PLC answers the second request first. Each caller must still get
        // its own reply — this is what the invoke-id map exists for.
        let (client, _reqs, _) = harness(|mut plc| {
            let a = plc.read_request().unwrap();
            let b = plc.read_request().unwrap();
            plc.reply(&b.header, 0, &ok_read_payload(&2i32.to_le_bytes()));
            plc.reply(&a.header, 0, &ok_read_payload(&1i32.to_le_bytes()));
        });

        let client = Arc::new(client);
        let c2 = Arc::clone(&client);
        let (tx, rx) = mpsc::channel();
        let t = thread::spawn(move || {
            // Give the first request time to hit the socket, so the fake PLC's
            // read order is deterministic.
            let v = c2.read(851, ADSIGRP_SYM_VALBYHND, 1, 4);
            let _ = tx.send(v);
        });
        let first = client.read(851, ADSIGRP_SYM_VALBYHND, 2, 4);
        t.join().unwrap();
        let second = rx.recv().unwrap();

        // Both succeeded and neither got the other's bytes.
        let a = first.unwrap();
        let b = second.unwrap();
        assert_ne!(a, b);
        assert!(a == 1i32.to_le_bytes() || a == 2i32.to_le_bytes());
        assert!(b == 1i32.to_le_bytes() || b == 2i32.to_le_bytes());
    }

    #[test]
    fn notification_frame_reaches_the_handler() {
        let (client, _reqs, notes) = harness(|mut plc| {
            let r = plc.read_request().unwrap();
            // Reply to the AddDeviceNotification, then push a sample.
            let mut p = 0u32.to_le_bytes().to_vec();
            p.extend_from_slice(&0x1234u32.to_le_bytes()); // handle
            plc.reply(&r.header, 0, &p);

            let mut body = 1u32.to_le_bytes().to_vec(); // numStamps
            body.extend_from_slice(&123u64.to_le_bytes()); // timestamp
            body.extend_from_slice(&1u32.to_le_bytes()); // numSamples
            body.extend_from_slice(&0x1234u32.to_le_bytes()); // hNotify
            body.extend_from_slice(&4u32.to_le_bytes()); // size
            body.extend_from_slice(&7i32.to_le_bytes());
            let mut payload = (body.len() as u32).to_le_bytes().to_vec();
            payload.extend_from_slice(&body);
            plc.push_notification(r.header.source, &payload);

            // Hold the socket open so the notification is not raced by EOF.
            thread::sleep(Duration::from_millis(200));
        });

        let handle = client
            .add_notification(
                851,
                ADSIGRP_SYM_VALBYHND,
                9,
                4,
                Duration::from_millis(10),
                Duration::from_millis(100),
            )
            .unwrap();
        assert_eq!(handle, 0x1234);

        let sample = notes.recv_timeout(Duration::from_secs(2)).unwrap();
        assert_eq!(sample.handle, 0x1234);
        assert_eq!(sample.timestamp, 123);
        assert_eq!(sample.data, 7i32.to_le_bytes());
    }

    #[test]
    fn add_notification_converts_times_to_100ns_units() {
        let (client, reqs, _) = harness(|mut plc| {
            let r = plc.read_request().unwrap();
            let mut p = 0u32.to_le_bytes().to_vec();
            p.extend_from_slice(&1u32.to_le_bytes());
            plc.reply(&r.header, 0, &p);
        });
        client
            .add_notification(
                851,
                0xF005,
                3,
                8,
                Duration::from_millis(10),  // cycle → 100_000 ticks
                Duration::from_millis(100), // maxDelay → 1_000_000 ticks
            )
            .unwrap();

        let req = reqs.recv().unwrap();
        assert_eq!(req.header.cmd_id, CMD_ADD_DEVICE_NOTIFICATION);
        let mut r = Reader::new(&req.payload);
        assert_eq!(r.u32().unwrap(), 0xF005);
        assert_eq!(r.u32().unwrap(), 3);
        assert_eq!(r.u32().unwrap(), 8);
        assert_eq!(r.u32().unwrap(), ADSTRANS_SERVERONCHA);
        assert_eq!(r.u32().unwrap(), 1_000_000, "maxDelay in 100 ns units");
        assert_eq!(r.u32().unwrap(), 100_000, "cycleTime in 100 ns units");
    }

    #[test]
    fn cycle_time_saturates_instead_of_wrapping() {
        // 1000 s exceeds the u32 100 ns field; wrapping would turn a slow cycle
        // into a near-zero one and flood the IOC.
        assert_eq!(duration_to_100ns(Duration::from_secs(1000)), u32::MAX);
        assert_eq!(duration_to_100ns(Duration::from_millis(10)), 100_000);
        assert_eq!(duration_to_100ns(Duration::ZERO), 0);
    }

    #[test]
    fn get_symbol_handle_writes_the_name_and_reads_four_bytes() {
        let (client, reqs, _) = harness(|mut plc| {
            let r = plc.read_request().unwrap();
            plc.reply(&r.header, 0, &ok_read_payload(&0xABCDu32.to_le_bytes()));
        });
        assert_eq!(client.get_symbol_handle(851, "Main.fTest").unwrap(), 0xABCD);

        let req = reqs.recv().unwrap();
        assert_eq!(req.header.cmd_id, CMD_READ_WRITE);
        let mut r = Reader::new(&req.payload);
        assert_eq!(r.u32().unwrap(), ADSIGRP_SYM_HNDBYNAME);
        assert_eq!(r.u32().unwrap(), 0);
        assert_eq!(r.u32().unwrap(), 4, "readLength");
        assert_eq!(r.u32().unwrap(), 10, "writeLength");
        assert_eq!(r.bytes(10).unwrap(), b"Main.fTest");
    }

    /// The sum-up path end to end: the request the PLC sees, and a response
    /// shorter than `read_length` because one sub-request failed — which the
    /// lenient READ_WRITE decoder must accept rather than calling a short read.
    #[test]
    fn sum_up_read_round_trips_through_read_write() {
        let entries = vec![
            SumEntry {
                index_group: ADSIGRP_SYM_VALBYHND,
                index_offset: 11,
                size: 4,
            },
            SumEntry {
                index_group: ADSIGRP_SYM_VALBYHND,
                index_offset: 22,
                size: 4,
            },
        ];

        let (client, reqs, _) = harness(|mut plc| {
            let r = plc.read_request().unwrap();
            // status[0] = ok, status[1] = SYMBOLNOTFOUND; compacted data area.
            let mut data = 0u32.to_le_bytes().to_vec();
            data.extend_from_slice(&0x0710u32.to_le_bytes());
            data.extend_from_slice(&7i32.to_le_bytes());
            plc.reply(&r.header, 0, &ok_read_payload(&data));
        });

        let raw = client.sum_up_read(851, &entries).unwrap();
        let (out, layout) = sumup::decode_response(&raw, &entries).unwrap();
        assert_eq!(layout, sumup::DataLayout::Compacted);
        assert_eq!(out[0].as_ref().unwrap(), &7i32.to_le_bytes());
        assert_eq!(out[1].as_ref().unwrap_err().code(), Some(0x0710));

        let req = reqs.recv().unwrap();
        assert_eq!(req.header.cmd_id, CMD_READ_WRITE);
        let mut r = Reader::new(&req.payload);
        assert_eq!(r.u32().unwrap(), ADSIGRP_SUMUP_READ);
        assert_eq!(
            r.u32().unwrap(),
            2,
            "sub-request count rides in indexOffset"
        );
        assert_eq!(r.u32().unwrap(), 2 * 4 + 8, "readLength");
        assert_eq!(r.u32().unwrap(), 24, "writeLength = 2 triples");
        assert_eq!(r.u32().unwrap(), ADSIGRP_SYM_VALBYHND);
        assert_eq!(r.u32().unwrap(), 11);
        assert_eq!(r.u32().unwrap(), 4);
    }

    #[test]
    fn read_state_decodes_both_states() {
        let (client, _reqs, _) = harness(|mut plc| {
            let r = plc.read_request().unwrap();
            let mut p = 0u32.to_le_bytes().to_vec();
            p.extend_from_slice(&5u16.to_le_bytes()); // ADSSTATE_RUN
            p.extend_from_slice(&0u16.to_le_bytes());
            plc.reply(&r.header, 0, &p);
        });
        assert_eq!(client.read_state(851).unwrap(), (AdsState::Run, 0));
    }

    #[test]
    fn request_after_peer_close_reports_not_connected() {
        let (client, _reqs, _) = harness(drop);
        // Let the reader thread observe EOF.
        for _ in 0..100 {
            if !client.is_connected() {
                break;
            }
            thread::sleep(Duration::from_millis(10));
        }
        assert!(!client.is_connected());
        assert!(matches!(
            client.read(851, ADSIGRP_SYM_VALBYHND, 1, 4),
            Err(AdsError::NotConnected)
        ));
    }

    #[test]
    fn in_flight_request_is_woken_when_the_socket_dies() {
        // The peer reads the request, then closes without answering. The caller
        // must fail immediately, not sit out the 500 ms timeout.
        let (client, _reqs, _) = harness(|mut plc| {
            plc.read_request();
            drop(plc);
        });
        let start = std::time::Instant::now();
        let err = client.read(851, ADSIGRP_SYM_VALBYHND, 1, 4).unwrap_err();
        assert!(matches!(err, AdsError::NotConnected), "got {err:?}");
        assert!(
            start.elapsed() < Duration::from_millis(400),
            "should be woken by tear_down, not by the timeout"
        );
    }

    #[test]
    fn unanswered_request_times_out_and_frees_its_slot() {
        let (client, _reqs, _) = harness(|mut plc| {
            plc.read_request();
            // Keep the socket open and stay silent.
            thread::sleep(Duration::from_millis(900));
        });
        assert!(matches!(
            client.read(851, ADSIGRP_SYM_VALBYHND, 1, 4),
            Err(AdsError::Timeout)
        ));
        assert!(
            client.shared.pending.lock().is_empty(),
            "timed-out request must not leak its pending slot"
        );
    }

    #[test]
    fn disconnect_handler_fires_once_on_socket_loss() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        thread::spawn(move || {
            let (s, _) = listener.accept().unwrap();
            drop(s);
        });
        let (tx, rx) = mpsc::channel();
        let client = AdsClient::from_stream(
            TcpStream::connect(addr).unwrap(),
            local(),
            PLC_NET,
            Duration::from_millis(200),
            Arc::new(|_| {}),
            Arc::new(move || {
                let _ = tx.send(());
            }),
        )
        .unwrap();
        rx.recv_timeout(Duration::from_secs(2)).unwrap();
        assert!(!client.is_connected());
    }
}
