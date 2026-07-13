#============================================================
# st.cmd — Love PID controller IOC (RS-485 multi-drop)
#
# Usage:
#   cargo run -p love-ioc -- st.cmd
#
# Mirrors upstream epics-modules/love's iocs/loveExIOC st.cmd.linux: one
# serial port shared by every controller address on the RS-485 bus, one
# LoveInit call per port, and one LoveConfig call per address to set that
# address's model (1600 or 16A) before any records bind to it.
#
# Unlike the delaygen drivers, EOS is NOT set here: drvLoveInit itself
# hardcodes input EOS 0x06 (ACK) / output EOS 0x03 (ETX) on the serial port
# (C `setDefaultEos`, called unconditionally) -- neither upstream startup
# script (st.cmd.linux, love.iocsh) ever calls asynOctetSetInputEos/
# asynOctetSetOutputEos for Love, and this port doesn't either.
#============================================================

# LOVE is set by main.rs (epics_rs::base::runtime::env::set_default) to
# this IOC crate's CARGO_MANIFEST_DIR.

# ---- shared serial octet port ----
drvAsynSerialPortConfigure("S0", "/dev/ttyS0", 0, 0, 0)

asynSetOption(S0, 0, "baud",    "19200")
asynSetOption(S0, 0, "bits",    "8")
asynSetOption(S0, 0, "parity",  "none")
asynSetOption(S0, 0, "stop",    "1")
asynSetOption(S0, 0, "clocal",  "Y")
asynSetOption(S0, 0, "crtscts", "N")

# ---- Love driver + per-address model configuration ----
# LoveInit(lovPort,serPort,serAddr)
LoveInit("L0", "S0", 0)

# LoveConfig(lovPort,addr,model) -- "1600" or "16A", one call per controller
# address actually wired to the RS-485 bus.
LoveConfig("L0", 1, "1600")
LoveConfig("L0", 2, "1600")
LoveConfig("L0", 3, "1600")
LoveConfig("L0", 4, "16A")

# ---- records: one pair per controller address ----
dbLoadRecords("$(LOVE)/db/LoveController.template", "P=love:,Q=Love1:,PORT=L0,ADDR=0x01")
dbLoadRecords("$(LOVE)/db/LoveControllerControl.template", "P=love:,Q=Love1:,PORT=L0,ADDR=0x01")

dbLoadRecords("$(LOVE)/db/LoveController.template", "P=love:,Q=Love4:,PORT=L0,ADDR=0x04")
dbLoadRecords("$(LOVE)/db/LoveControllerControl.template", "P=love:,Q=Love4:,PORT=L0,ADDR=0x04")

## Addresses 2 and 3 (commented -- enable as needed):
# dbLoadRecords("$(LOVE)/db/LoveController.template", "P=love:,Q=Love2:,PORT=L0,ADDR=0x02")
# dbLoadRecords("$(LOVE)/db/LoveControllerControl.template", "P=love:,Q=Love2:,PORT=L0,ADDR=0x02")
# dbLoadRecords("$(LOVE)/db/LoveController.template", "P=love:,Q=Love3:,PORT=L0,ADDR=0x03")
# dbLoadRecords("$(LOVE)/db/LoveControllerControl.template", "P=love:,Q=Love3:,PORT=L0,ADDR=0x03")

#------------------------------------------------------------------------------
iocInit()
