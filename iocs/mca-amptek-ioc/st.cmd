#============================================================
# st.cmd -- Amptek DP5 MCA IOC
#
# Usage:
#   cargo run -p mca-amptek-ioc -- st.cmd
#
# Boots one Amptek DP5 asyn port over the Ethernet/UDP transport
# (drvAmptekConfigure) and loads one mca record (devMcaAsyn) plus the full
# Amptek.db/Amptek_SCAn.db control record set, ported from upstream
# iocBoot/iocAmptek/st_base.cmd.
#
# Deviation: dbLoadTemplate is not an iocsh command in epics-rs (same gap
# noted in syringepump-ioc's st.cmd). Amptek_SCAs.substitutions' 8 pattern
# rows (N=0..7) are mechanically expanded below into 8 explicit
# dbLoadRecords calls -- textually equivalent to what dbLoadTemplate would
# have produced.
#
# Omitted (matching every other IOC in this workspace): autosave/
# save_restore machinery (create_monitor_set/auto_settings.req) and the
# optional debug asynRecord instance upstream's st.cmd loads. Neither is
# protocol logic.
#
# Live boot test: point ADDRESS at 127.0.0.1 and run
# `cargo run -p mca-amptek --example stub_dp5` in another terminal first --
# see that example's doc comment. directMode=1 matches the stub, which
# only answers the direct (unicast) NetFinder query, not broadcast
# discovery.
#============================================================

# MCA_AMPTEK_IOC is set by main.rs (epics_rs::base::runtime::env::set_default)
# to this IOC crate's CARGO_MANIFEST_DIR.

# drvAmptekConfigure(portName, interface, addressInfo, directMode)
# interface: 0 = Ethernet (only supported interface -- USB is
# feasibility-gated, see the mca-amptek crate doc).
drvAmptekConfigure("Amptek1", 0, "127.0.0.1", 1)

# ---- records ----
dbLoadRecords("$(MCA_AMPTEK_IOC)/db/mca.db", "P=mcaTest:,R=spectrum1,PORT=Amptek1,ADDR=0,NCHAN=8192")
dbLoadRecords("$(MCA_AMPTEK_IOC)/db/Amptek.db", "P=mcaTest:,R=Amptek1:,PORT=Amptek1")

# Amptek_SCAs.substitutions, expanded (see header comment).
dbLoadRecords("$(MCA_AMPTEK_IOC)/db/Amptek_SCAn.db", "PORT=Amptek1,P=mcaTest:,R=Amptek1:,M=spectrum1,N=0")
dbLoadRecords("$(MCA_AMPTEK_IOC)/db/Amptek_SCAn.db", "PORT=Amptek1,P=mcaTest:,R=Amptek1:,M=spectrum1,N=1")
dbLoadRecords("$(MCA_AMPTEK_IOC)/db/Amptek_SCAn.db", "PORT=Amptek1,P=mcaTest:,R=Amptek1:,M=spectrum1,N=2")
dbLoadRecords("$(MCA_AMPTEK_IOC)/db/Amptek_SCAn.db", "PORT=Amptek1,P=mcaTest:,R=Amptek1:,M=spectrum1,N=3")
dbLoadRecords("$(MCA_AMPTEK_IOC)/db/Amptek_SCAn.db", "PORT=Amptek1,P=mcaTest:,R=Amptek1:,M=spectrum1,N=4")
dbLoadRecords("$(MCA_AMPTEK_IOC)/db/Amptek_SCAn.db", "PORT=Amptek1,P=mcaTest:,R=Amptek1:,M=spectrum1,N=5")
dbLoadRecords("$(MCA_AMPTEK_IOC)/db/Amptek_SCAn.db", "PORT=Amptek1,P=mcaTest:,R=Amptek1:,M=spectrum1,N=6")
dbLoadRecords("$(MCA_AMPTEK_IOC)/db/Amptek_SCAn.db", "PORT=Amptek1,P=mcaTest:,R=Amptek1:,M=spectrum1,N=7")

#------------------------------------------------------------------------------
iocInit()
