//! USB transport for the DP5, ported from `CDppLibUsb` (`DppLibUsb.cpp`)
//! and the `CConsoleHelper::LibUsb_*` sequences drvAmptek reaches
//! (`ConsoleHelper.cpp:476-688`). Every libusb call goes through
//! [`UsbBus`] and [`UsbHandle`]; error lines go to stderr as C's
//! `fprintf(stderr, ...)` writes them.

use std::time::Duration;

use crate::protocol::{self, TransmitPacketType};

/// `DppLibUsb.h:20-33`.
pub const AMPTEK_DP5_VENDOR_ID: u16 = 0x10C4;
pub const AMPTEK_DP5_PRODUCT_ID: u16 = 0x842A;
pub const INTERFACE_NUMBER: u8 = 0;
pub const BULK_IN_ENDPOINT: u8 = 0x81;
pub const BULK_OUT_ENDPOINT: u8 = 0x02;
pub const MAX_BULK_IN_TRANSFER_SIZE: usize = 32768;
const DP5_USB_TIMEOUT: Duration = Duration::from_millis(500);
const DP5_DIAGDATA_TIMEOUT: Duration = Duration::from_millis(2500);
const PID1_REQ_SCOPE_MISC_TO: u8 = 0x03;
const PID2_SEND_DIAGNOSTIC_DATA_TO: u8 = 0x05;

/// A `libusb_error`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UsbError {
    Io,
    InvalidParam,
    Access,
    NoDevice,
    NotFound,
    Busy,
    Timeout,
    Overflow,
    Pipe,
    Interrupted,
    NoMem,
    NotSupported,
    Other,
}

impl UsbError {
    /// `CDppLibUsb::libusb_strerror` (`DppLibUsb.cpp:210-243`).
    pub fn strerror(self) -> &'static str {
        match self {
            UsbError::Io => "Input/output error",
            UsbError::InvalidParam => "Invalid parameter",
            UsbError::Access => "Access denied (insufficient permissions)",
            UsbError::NoDevice => "No such device (it may have been disconnected)",
            UsbError::NotFound => "Entity not found",
            UsbError::Busy => "Resource busy",
            UsbError::Timeout => "Operation timed out",
            UsbError::Overflow => "Overflow",
            UsbError::Pipe => "Pipe error",
            UsbError::Interrupted => "System call interrupted (perhaps due to signal)",
            UsbError::NoMem => "Insufficient memory",
            UsbError::NotSupported => "Operation not supported or unimplemented on this platform",
            UsbError::Other => "Other error",
        }
    }
}

/// The libusb device-list and open calls `CDppLibUsb` makes.
pub trait UsbBus: Send + Sync {
    /// `libusb_get_device_list`: each device's `(idVendor, idProduct)`, in
    /// list order.
    fn devices(&mut self) -> Result<Vec<(u16, u16)>, UsbError>;

    /// `libusb_open` of the device at `index` in the last
    /// [`Self::devices`] list.
    fn open(&mut self, index: usize) -> Result<Box<dyn UsbHandle>, UsbError>;
}

/// A `libusb_device_handle`; dropping it is `libusb_close`.
pub trait UsbHandle: Send + Sync {
    fn claim_interface(&mut self, interface: u8) -> Result<(), UsbError>;

    fn release_interface(&mut self, interface: u8) -> Result<(), UsbError>;

    /// `libusb_bulk_transfer` to an OUT endpoint.
    fn bulk_out(&mut self, endpoint: u8, data: &[u8], timeout: Duration) -> Result<(), UsbError>;

    /// `libusb_bulk_transfer` from an IN endpoint into `buf`: the bytes
    /// received.
    fn bulk_in(
        &mut self,
        endpoint: u8,
        buf: &mut [u8],
        timeout: Duration,
    ) -> Result<usize, UsbError>;
}

/// `CDppLibUsb::isAmptekDP5Device` (`DppLibUsb.cpp:140-148`).
fn is_amptek_dp5((vendor, product): (u16, u16)) -> bool {
    vendor == AMPTEK_DP5_VENDOR_ID && product == AMPTEK_DP5_PRODUCT_ID
}

/// glibc `atoi`: `(int) strtol(s, NULL, 10)`.
fn atoi(s: &str) -> i32 {
    let s = s.trim_start_matches([' ', '\t', '\n', '\x0b', '\x0c', '\r']);
    let (negative, digits) = match s.as_bytes().first() {
        Some(b'-') => (true, &s[1..]),
        Some(b'+') => (false, &s[1..]),
        _ => (false, s),
    };
    let mut value: i64 = 0;
    for d in digits.bytes().take_while(u8::is_ascii_digit) {
        let d = i64::from(d - b'0');
        value = value.saturating_mul(10);
        value = if negative {
            value.saturating_sub(d)
        } else {
            value.saturating_add(d)
        };
    }
    value as i32
}

/// `CDppLibUsb` with `CConsoleHelper`'s `LibUsb_*` state.
pub struct DppLibUsb {
    bus: Box<dyn UsbBus>,
    /// `DppLibusbHandle`, `Some` exactly while `bDeviceConnected`.
    handle: Option<Box<dyn UsbHandle>>,
}

impl DppLibUsb {
    pub fn new(bus: Box<dyn UsbBus>) -> Self {
        DppLibUsb { bus, handle: None }
    }

    /// `CDppLibUsb::bDeviceConnected`.
    pub fn is_connected(&self) -> bool {
        self.handle.is_some()
    }

    /// `CDppLibUsb::CountDP5LibusbDevices` (`DppLibUsb.cpp:152-175`): -1
    /// when the list fails.
    fn count_devices(&mut self) -> i32 {
        match self.bus.devices() {
            Ok(list) => list.into_iter().filter(|&ids| is_amptek_dp5(ids)).count() as i32,
            Err(e) => {
                epics_rs::base::stderr_println!("failed to get device list: {}", e.strerror());
                -1
            }
        }
    }

    /// `CDppLibUsb::FindUSBDevice` (`DppLibUsb.cpp:30-88`): open and claim
    /// the `idx`-th DP5 on the bus, counting from 1.
    fn find_device(&mut self, idx: i32) -> Option<Box<dyn UsbHandle>> {
        let list = match self.bus.devices() {
            Ok(list) => list,
            Err(e) => {
                epics_rs::base::stderr_println!("failed to get device list: {}", e.strerror());
                return None;
            }
        };
        let nth = usize::try_from(idx - 1).ok()?;
        let (index, _) = list
            .into_iter()
            .enumerate()
            .filter(|&(_, ids)| is_amptek_dp5(ids))
            .nth(nth)?;
        let mut handle = match self.bus.open(index) {
            Ok(handle) => handle,
            Err(e) => {
                epics_rs::base::stderr_println!("Cannot open USB device: {}", e.strerror());
                return None;
            }
        };
        if let Err(e) = handle.claim_interface(INTERFACE_NUMBER) {
            epics_rs::base::stderr_println!("libusb_claim_interface error {}", e.strerror());
            return None;
        }
        Some(handle)
    }

    /// `bDeviceConnected = false; CloseUSBDevice(DppLibusbHandle)`
    /// (`DppLibUsb.cpp:90-100`).
    fn close_device(&mut self) {
        if let Some(mut handle) = self.handle.take()
            && let Err(e) = handle.release_interface(INTERFACE_NUMBER)
        {
            epics_rs::base::stderr_println!("libusb_release_interface error {}", e.strerror());
        }
    }

    /// `CConsoleHelper::ConnectDpp`'s USB branch (`ConsoleHelper.cpp:39-55`):
    /// the first DP5 for an empty address, else the one whose serial number
    /// is `atoi(address)`; a zero serial connects nothing. `matches_serial`
    /// is `ReceiveData` on a status reply, then the serial comparison.
    /// Returns `(LibUsb_isConnected, LibUsb_NumDevices)`.
    pub fn connect(
        &mut self,
        address: &str,
        matches_serial: impl FnMut(&[u8], u32) -> bool,
    ) -> (bool, usize) {
        if address.is_empty() {
            return self.connect_default();
        }
        match atoi(address) {
            0 => (false, 0),
            sn => self.connect_by_sn(sn as u32, matches_serial),
        }
    }

    /// `CConsoleHelper::LibUsb_Connect_Default_DPP` (`ConsoleHelper.cpp:476-503`).
    fn connect_default(&mut self) -> (bool, usize) {
        let num_devices = self.count_devices();
        if num_devices > 0 {
            self.handle = self.find_device(1);
        }
        match self.handle {
            Some(_) => (true, num_devices as usize),
            None => (false, 0),
        }
    }

    /// `CConsoleHelper::LibUsb_Connect_DPP_by_SN` (`ConsoleHelper.cpp:536-578`):
    /// each DP5 in turn is opened and asked for its status until one
    /// reports serial `sn`; the others are closed again.
    fn connect_by_sn(
        &mut self,
        sn: u32,
        mut matches_serial: impl FnMut(&[u8], u32) -> bool,
    ) -> (bool, usize) {
        let num_devices = self.count_devices();
        let status = protocol::build_command(TransmitPacketType::SendStatus);
        for idx in 1..=num_devices {
            self.handle = self.find_device(idx);
            if self.handle.is_none() {
                continue;
            }
            let reply = self.send_packet(&status);
            if !reply.is_empty() && matches_serial(&reply, sn) {
                return (true, num_devices as usize);
            }
            self.close_device();
        }
        (false, 0)
    }

    /// `CConsoleHelper::LibUsb_Close_Connection` (`ConsoleHelper.cpp:580-589`).
    pub fn close_connection(&mut self) {
        self.close_device();
    }

    /// `LibUsb_SendCommand`/`LibUsb_SendCommand_Config` over
    /// `CDppLibUsb::SendPacketUSB` (`DppLibUsb.cpp:102-138`): bulk OUT the
    /// header's length of `packet`, then one bulk IN. The reply, empty for
    /// C's `false`. C reads up to 32768 bytes into the 24648-byte
    /// `PacketIn`; the reply buffer here is the transfer size, so an
    /// oversize reply cannot overrun it.
    pub fn send_packet(&mut self, packet: &[u8]) -> Vec<u8> {
        let Some(handle) = self.handle.as_mut() else {
            return Vec::new();
        };
        let byte = |i: usize| packet.get(i).copied().unwrap_or(0);
        let timeout =
            if byte(2) == PID1_REQ_SCOPE_MISC_TO && byte(3) == PID2_SEND_DIAGNOSTIC_DATA_TO {
                DP5_DIAGDATA_TIMEOUT
            } else {
                DP5_USB_TIMEOUT
            };
        let length = usize::from(byte(4)) * 256 + usize::from(byte(5)) + 8;
        let out = &packet[..length.min(packet.len())];
        if let Err(e) = handle.bulk_out(BULK_OUT_ENDPOINT, out, timeout) {
            epics_rs::base::stderr_println!(
                "Error sending data via bulk transfer: {}",
                e.strerror()
            );
            return Vec::new();
        }
        let mut reply = vec![0u8; MAX_BULK_IN_TRANSFER_SIZE];
        match handle.bulk_in(BULK_IN_ENDPOINT, &mut reply, timeout) {
            Ok(0) => {
                epics_rs::base::stderr_println!("No data received in bulk transfer: Success");
                Vec::new()
            }
            Ok(n) => {
                reply.truncate(n);
                reply
            }
            Err(e) => {
                epics_rs::base::stderr_println!(
                    "Error receiving data via bulk transfer: {}",
                    e.strerror()
                );
                Vec::new()
            }
        }
    }
}

/// The host's USB stack as a [`UsbBus`], or `None` in a build without
/// one (the `usb` feature off, or a target nusb does not support).
#[cfg(all(
    feature = "usb",
    any(
        target_os = "linux",
        target_os = "android",
        target_os = "windows",
        target_os = "macos"
    )
))]
pub fn host_bus() -> Option<Box<dyn UsbBus>> {
    Some(Box::new(host::HostBus::default()))
}

#[cfg(not(all(
    feature = "usb",
    any(
        target_os = "linux",
        target_os = "android",
        target_os = "windows",
        target_os = "macos"
    )
)))]
pub fn host_bus() -> Option<Box<dyn UsbBus>> {
    None
}

/// [`UsbBus`] on nusb, standing in for libusb.
#[cfg(all(
    feature = "usb",
    any(
        target_os = "linux",
        target_os = "android",
        target_os = "windows",
        target_os = "macos"
    )
))]
mod host {
    use std::time::Duration;

    use nusb::transfer::{Buffer, Bulk, In, Out, TransferError};
    use nusb::{Endpoint, Interface, MaybeFuture};

    use super::{UsbBus, UsbError, UsbHandle};

    fn from_error(e: nusb::Error) -> UsbError {
        match e.kind() {
            nusb::ErrorKind::Disconnected => UsbError::NoDevice,
            nusb::ErrorKind::Busy => UsbError::Busy,
            nusb::ErrorKind::PermissionDenied => UsbError::Access,
            nusb::ErrorKind::NotFound => UsbError::NotFound,
            nusb::ErrorKind::Unsupported => UsbError::NotSupported,
            _ => UsbError::Other,
        }
    }

    /// libusb's code for a transfer's status; nusb reports a timed-out
    /// transfer as cancelled.
    fn from_transfer(e: TransferError) -> UsbError {
        match e {
            TransferError::Cancelled => UsbError::Timeout,
            TransferError::Stall => UsbError::Pipe,
            TransferError::Disconnected => UsbError::NoDevice,
            TransferError::Fault => UsbError::Io,
            TransferError::InvalidArgument => UsbError::InvalidParam,
            _ => UsbError::Other,
        }
    }

    #[derive(Default)]
    pub(super) struct HostBus {
        listed: Vec<nusb::DeviceInfo>,
    }

    impl UsbBus for HostBus {
        fn devices(&mut self) -> Result<Vec<(u16, u16)>, UsbError> {
            self.listed = nusb::list_devices().wait().map_err(from_error)?.collect();
            Ok(self
                .listed
                .iter()
                .map(|d| (d.vendor_id(), d.product_id()))
                .collect())
        }

        fn open(&mut self, index: usize) -> Result<Box<dyn UsbHandle>, UsbError> {
            let info = self.listed.get(index).ok_or(UsbError::NoDevice)?;
            let device = info.open().wait().map_err(from_error)?;
            Ok(Box::new(HostHandle {
                device,
                claimed: None,
            }))
        }
    }

    /// An open device; dropping it closes it.
    struct HostHandle {
        device: nusb::Device,
        claimed: Option<Claimed>,
    }

    /// A claimed interface and the bulk endpoints opened on it, each on
    /// its first transfer as libusb addresses them.
    struct Claimed {
        interface: Interface,
        bulk_out: Option<Endpoint<Bulk, Out>>,
        bulk_in: Option<Endpoint<Bulk, In>>,
    }

    impl HostHandle {
        fn claimed(&mut self) -> Result<&mut Claimed, UsbError> {
            self.claimed.as_mut().ok_or(UsbError::NotFound)
        }
    }

    impl UsbHandle for HostHandle {
        fn claim_interface(&mut self, interface: u8) -> Result<(), UsbError> {
            let interface = self
                .device
                .claim_interface(interface)
                .wait()
                .map_err(from_error)?;
            self.claimed = Some(Claimed {
                interface,
                bulk_out: None,
                bulk_in: None,
            });
            Ok(())
        }

        fn release_interface(&mut self, _interface: u8) -> Result<(), UsbError> {
            match self.claimed.take() {
                Some(claimed) => claimed.interface.release().wait().map_err(from_error),
                None => Err(UsbError::NotFound),
            }
        }

        fn bulk_out(
            &mut self,
            endpoint: u8,
            data: &[u8],
            timeout: Duration,
        ) -> Result<(), UsbError> {
            let claimed = self.claimed()?;
            if claimed.bulk_out.is_none() {
                claimed.bulk_out = Some(claimed.interface.endpoint(endpoint).map_err(from_error)?);
            }
            let ep = claimed.bulk_out.as_mut().ok_or(UsbError::NotFound)?;
            let done = ep.transfer_blocking(Buffer::from(data), timeout);
            done.status.map_err(from_transfer)
        }

        fn bulk_in(
            &mut self,
            endpoint: u8,
            buf: &mut [u8],
            timeout: Duration,
        ) -> Result<usize, UsbError> {
            let claimed = self.claimed()?;
            if claimed.bulk_in.is_none() {
                claimed.bulk_in = Some(claimed.interface.endpoint(endpoint).map_err(from_error)?);
            }
            let ep = claimed.bulk_in.as_mut().ok_or(UsbError::NotFound)?;
            let done = ep.transfer_blocking(Buffer::new(buf.len()), timeout);
            done.status.map_err(from_transfer)?;
            let n = done.buffer.len().min(buf.len());
            buf[..n].copy_from_slice(&done.buffer[..n]);
            Ok(n)
        }
    }
}

#[cfg(test)]
pub(crate) mod fake {
    //! A [`UsbBus`] over scripted devices.

    use super::*;
    use std::sync::{Arc, Mutex};

    /// What a fake bus records.
    #[derive(Debug, Default)]
    pub struct Log {
        pub opened: Vec<usize>,
        pub released: Vec<usize>,
        /// `(device index, endpoint, bytes, timeout)` per bulk OUT.
        pub sent: Vec<(usize, u8, Vec<u8>, Duration)>,
    }

    /// The IN endpoint's answer to an OUT packet.
    pub type Reply = Box<dyn FnMut(&[u8]) -> Result<Vec<u8>, UsbError> + Send>;

    /// One listed device: its ids, its claim result and its replies.
    pub struct FakeDevice {
        pub ids: (u16, u16),
        pub claim: Result<(), UsbError>,
        pub reply: Reply,
    }

    impl FakeDevice {
        pub fn dp5(reply: impl FnMut(&[u8]) -> Result<Vec<u8>, UsbError> + Send + 'static) -> Self {
            FakeDevice {
                ids: (AMPTEK_DP5_VENDOR_ID, AMPTEK_DP5_PRODUCT_ID),
                claim: Ok(()),
                reply: Box::new(reply),
            }
        }
    }

    type Devices = Arc<Mutex<Vec<FakeDevice>>>;

    pub struct FakeBus {
        devices: Devices,
        pub log: Arc<Mutex<Log>>,
    }

    impl FakeBus {
        pub fn new(devices: Vec<FakeDevice>) -> Self {
            FakeBus {
                devices: Arc::new(Mutex::new(devices)),
                log: Arc::default(),
            }
        }
    }

    impl UsbBus for FakeBus {
        fn devices(&mut self) -> Result<Vec<(u16, u16)>, UsbError> {
            Ok(self.devices.lock().unwrap().iter().map(|d| d.ids).collect())
        }

        fn open(&mut self, index: usize) -> Result<Box<dyn UsbHandle>, UsbError> {
            self.log.lock().unwrap().opened.push(index);
            Ok(Box::new(FakeHandle {
                index,
                devices: self.devices.clone(),
                log: self.log.clone(),
                pending: None,
            }))
        }
    }

    struct FakeHandle {
        index: usize,
        devices: Devices,
        log: Arc<Mutex<Log>>,
        pending: Option<Result<Vec<u8>, UsbError>>,
    }

    impl UsbHandle for FakeHandle {
        fn claim_interface(&mut self, interface: u8) -> Result<(), UsbError> {
            assert_eq!(interface, INTERFACE_NUMBER);
            self.devices.lock().unwrap()[self.index].claim
        }

        fn release_interface(&mut self, _interface: u8) -> Result<(), UsbError> {
            self.log.lock().unwrap().released.push(self.index);
            Ok(())
        }

        fn bulk_out(
            &mut self,
            endpoint: u8,
            data: &[u8],
            timeout: Duration,
        ) -> Result<(), UsbError> {
            self.log
                .lock()
                .unwrap()
                .sent
                .push((self.index, endpoint, data.to_vec(), timeout));
            self.pending = Some((self.devices.lock().unwrap()[self.index].reply)(data));
            Ok(())
        }

        fn bulk_in(
            &mut self,
            endpoint: u8,
            buf: &mut [u8],
            _timeout: Duration,
        ) -> Result<usize, UsbError> {
            assert_eq!(endpoint, BULK_IN_ENDPOINT);
            let reply = self.pending.take().unwrap_or(Err(UsbError::Timeout))?;
            buf[..reply.len()].copy_from_slice(&reply);
            Ok(reply.len())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::fake::*;
    use super::*;

    fn usb(devices: Vec<FakeDevice>) -> (DppLibUsb, std::sync::Arc<std::sync::Mutex<Log>>) {
        let bus = FakeBus::new(devices);
        let log = bus.log.clone();
        (DppLibUsb::new(Box::new(bus)), log)
    }

    fn other_device() -> FakeDevice {
        FakeDevice {
            ids: (0x1234, 0x5678),
            ..FakeDevice::dp5(|_| Ok(vec![1]))
        }
    }

    #[test]
    fn atoi_is_glibc_atoi() {
        assert_eq!(atoi(" 12ab"), 12);
        assert_eq!(atoi("-7"), -7);
        assert_eq!(atoi("abc"), 0);
        assert_eq!(atoi("99999999999999999999"), -1);
    }

    /// An empty address opens the first DP5, skipping other devices.
    #[test]
    fn an_empty_address_connects_the_first_dp5() {
        let (mut usb, log) = usb(vec![
            other_device(),
            FakeDevice::dp5(|_| Ok(vec![1])),
            FakeDevice::dp5(|_| Ok(vec![2])),
        ]);
        assert_eq!(usb.connect("", |_, _| unreachable!()), (true, 2));
        assert_eq!(log.lock().unwrap().opened, vec![1]);
    }

    #[test]
    fn no_dp5_connects_nothing() {
        let (mut usb, log) = usb(vec![other_device()]);
        assert_eq!(usb.connect("", |_, _| true), (false, 0));
        assert!(log.lock().unwrap().opened.is_empty());
    }

    /// A failed claim leaves the device closed (`DppLibUsb.cpp:71-79`).
    #[test]
    fn a_refused_claim_connects_nothing() {
        let mut dp5 = FakeDevice::dp5(|_| Ok(vec![1]));
        dp5.claim = Err(UsbError::Busy);
        let (mut usb, _) = usb(vec![dp5]);
        assert_eq!(usb.connect("", |_, _| true), (false, 0));
        assert!(!usb.is_connected());
    }

    /// A non-numeric or zero serial connects nothing (`ConsoleHelper.cpp:40-48`).
    #[test]
    fn a_zero_serial_connects_nothing() {
        let (mut usb, log) = usb(vec![FakeDevice::dp5(|_| Ok(vec![1]))]);
        assert_eq!(usb.connect("abc", |_, _| true), (false, 0));
        assert!(log.lock().unwrap().opened.is_empty());
    }

    /// Each DP5 is asked for its status; the mismatched one is released.
    #[test]
    fn a_serial_number_selects_the_device_that_reports_it() {
        let (mut usb, log) = usb(vec![
            FakeDevice::dp5(|_| Ok(vec![0x11])),
            FakeDevice::dp5(|_| Ok(vec![0x22])),
        ]);
        let found = usb.connect("4660", |reply, sn| {
            assert_eq!(sn, 4660);
            reply == [0x22]
        });
        assert_eq!(found, (true, 2));
        let log = log.lock().unwrap();
        assert_eq!(log.opened, vec![0, 1]);
        assert_eq!(log.released, vec![0]);
        let status = protocol::build_command(TransmitPacketType::SendStatus);
        assert_eq!(log.sent[0].1, BULK_OUT_ENDPOINT);
        assert_eq!(log.sent[0].2, status);
        assert_eq!(log.sent[0].3, DP5_USB_TIMEOUT);
    }

    #[test]
    fn an_unmatched_serial_connects_nothing() {
        let (mut usb, log) = usb(vec![FakeDevice::dp5(|_| Ok(vec![0x11]))]);
        assert_eq!(usb.connect("5", |_, _| false), (false, 0));
        assert!(!usb.is_connected());
        assert_eq!(log.lock().unwrap().released, vec![0]);
    }

    /// The diagnostic-data request gets the 2500 ms timeout
    /// (`DppLibUsb.cpp:108-112`).
    #[test]
    fn diagnostic_data_uses_the_long_timeout() {
        let (mut usb, log) = usb(vec![FakeDevice::dp5(|_| Ok(vec![1]))]);
        usb.connect("", |_, _| true);
        let mut diag = protocol::build_command(TransmitPacketType::SendStatus);
        diag[2] = PID1_REQ_SCOPE_MISC_TO;
        diag[3] = PID2_SEND_DIAGNOSTIC_DATA_TO;
        assert_eq!(usb.send_packet(&diag), vec![1]);
        assert_eq!(log.lock().unwrap().sent[0].3, DP5_DIAGDATA_TIMEOUT);
    }

    /// A failed or empty bulk IN is C's `false`; so is a send with no device.
    #[test]
    fn a_failed_transfer_is_an_empty_reply() {
        let packet = protocol::build_command(TransmitPacketType::SendStatus);
        let (mut usb, _) = usb(vec![FakeDevice::dp5(|_| Err(UsbError::Timeout))]);
        assert!(usb.send_packet(&packet).is_empty());
        usb.connect("", |_, _| true);
        assert!(usb.send_packet(&packet).is_empty());

        let (mut usb, _) = self::usb(vec![FakeDevice::dp5(|_| Ok(Vec::new()))]);
        usb.connect("", |_, _| true);
        assert!(usb.send_packet(&packet).is_empty());
    }

    #[test]
    fn close_connection_releases_the_interface() {
        let (mut usb, log) = usb(vec![FakeDevice::dp5(|_| Ok(vec![1]))]);
        usb.connect("", |_, _| true);
        usb.close_connection();
        assert!(!usb.is_connected());
        assert_eq!(log.lock().unwrap().released, vec![0]);
    }
}
