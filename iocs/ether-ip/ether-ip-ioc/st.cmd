#!../../bin/ether-ip-ioc
# EtherIP IOC startup script -*- shell-script -*-
#
# Port of epics-modules/ether_ip/iocBoot/iocether_ip/st.cmd.
#
# The Rust IOC needs no dbLoadDatabase / registerRecordDeviceDriver: the record
# types are built in and the "EtherIP" device support is registered from
# main.rs.

# Optional driver knobs, all effective before the scan tasks start.
# EIP_timeout(5000)
# EIP_buffer_limit(480)
# drvEtherIP_default_rate(1.0)

# Define the PLCs: name (used by the records), IP address (or DNS name), and
# the ControlLogix slot the CPU sits in (0 for a PLC-5 / CompactLogix).
drvEtherIP_define_PLC("plc1", "127.0.0.1", 0)

dbLoadRecords("iocs/ether-ip/ether-ip-ioc/db/test.db", "PLC=plc1,IOC=test")
dbLoadRecords("iocs/ether-ip/ether-ip-ioc/db/eip_stat.db", "PLC=plc1,IOC=test,TAG=REAL")

iocInit()

# drvEtherIP_report(10)
