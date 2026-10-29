//! `CConsoleHelper`'s per-interface dispatch (`ConsoleHelper.cpp:25-125`):
//! one variant per `DppInterface_t`, each holding that interface's
//! connection state.

use std::io;
use std::net::Ipv4Addr;

use crate::transport::AmptekUdpTransport;
use crate::usb::DppLibUsb;

pub(crate) enum DppLink {
    /// `CDppSocket`. `direct_mode` is `drvAmptek::directMode_`, which C
    /// forces to 0 for the other interfaces; `dpp_addr` is `DppAddr`, set
    /// while `deviceConnected`.
    Ethernet {
        udp: AmptekUdpTransport,
        direct_mode: bool,
        dpp_addr: Option<Ipv4Addr>,
    },
    Usb(DppLibUsb),
    /// `ConnectDpp`'s and `SendCommand`'s Serial branches are empty.
    Serial,
}

impl DppLink {
    pub(crate) fn ethernet(udp: AmptekUdpTransport, direct_mode: bool) -> Self {
        DppLink::Ethernet {
            udp,
            direct_mode,
            dpp_addr: None,
        }
    }

    /// `CConsoleHelper::isConnected`.
    pub(crate) fn is_connected(&self) -> bool {
        match self {
            DppLink::Ethernet { dpp_addr, .. } => dpp_addr.is_some(),
            DppLink::Usb(usb) => usb.is_connected(),
            DppLink::Serial => false,
        }
    }

    /// `CConsoleHelper::SendCommand`/`SendCommand_Config`
    /// (`ConsoleHelper.cpp:85-125`): the reply to `packet`, empty for C's
    /// `false`. `expected_size` ends an Ethernet receive early.
    pub(crate) fn send(&mut self, packet: &[u8], expected_size: usize) -> io::Result<Vec<u8>> {
        match self {
            DppLink::Ethernet {
                udp,
                dpp_addr: Some(addr),
                ..
            } => udp.send_packet_inet(*addr, packet, expected_size),
            DppLink::Usb(usb) => Ok(usb.send_packet(packet)),
            DppLink::Ethernet { dpp_addr: None, .. } | DppLink::Serial => Ok(Vec::new()),
        }
    }

    /// `CConsoleHelper::Close_Connection` (`ConsoleHelper.cpp:65-81`).
    pub(crate) fn close(&mut self) {
        match self {
            DppLink::Ethernet { dpp_addr, .. } => *dpp_addr = None,
            DppLink::Usb(usb) => usb.close_connection(),
            DppLink::Serial => {}
        }
    }
}
