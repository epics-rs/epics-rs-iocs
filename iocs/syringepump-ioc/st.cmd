#============================================================
# st.cmd -- SyringePump IOC startup script
#
# Usage:
#   cargo run -p syringepump-ioc -- st.cmd
#
# Wires all three pump families from epics-modules/SyringePump:
#   - Teledyne ISCO D/H-series (native asyn port driver -- drivers/syringepump)
#   - ISCO (Modbus/TCP, drvModbusAsynConfigure)
#   - Vindum (Modbus RTU-over-TCP, drvModbusAsynConfigure)
# No upstream st.cmd/iocBoot ships for either Teledyne family (confirmed
# absent from epics-modules/SyringePump/iocBoot) -- the wiring below is a
# demo instantiation of each, chosen by this port, not sourced from an
# upstream reference. ISCO/Vindum's drvModbusAsynConfigure arguments below
# ARE sourced verbatim from the real upstream
# iocBoot/iocISCO/st.cmd and iocBoot/iocVindum/st.cmd.
#
# ISCO*/Vindum* register maps load via dbLoadTemplate (epics-base-rs 0.24.3,
# registered as a builtin iocsh command) against db/*.substitutions -- ported
# byte-for-byte from the real upstream ISCO*/Vindum*.substitutions files,
# with each `file "$(MODBUS)/db/..."` path rewritten to a bare filename
# (dbLoadTemplate does not macro-expand `file` paths; resolved relative to
# the .substitutions file's own directory, i.e. db/). See each
# db/*.substitutions file's own header for its exact deviations from
# upstream.
#
# Omitted (matching drivers/love, drivers/scaler974, motor/amci precedent
# in this workspace): autosave/save_restore machinery
# (create_monitor_set/auto_settings.req) and the optional debug asynRecord
# instance upstream's st.cmd loads. Neither is protocol logic.
#============================================================

epicsEnvSet("P", "SP:")

# ------------------------------------------------------------------
# Teledyne D-series (demo instance -- no upstream st.cmd/iocBoot exists
# for either Teledyne family; see header comment).
# ------------------------------------------------------------------
# The driver port records bind to ($(TELD_PORT)) must differ from the serial
# transport it rides on ($(TELD_TTY)): TeledyneDInit registers its own port,
# so reusing the serial port's name collides ("port already registered").
epicsEnvSet("TELD_PORT", "TelD1")
epicsEnvSet("TELD_TTY", "TelD1_tty")
drvAsynSerialPortConfigure("$(TELD_TTY)", "/dev/ttyS0", 0, 0, 0)
asynSetTraceIOMask("$(TELD_TTY)", 0, HEX)
# TeledyneDInit(port, serPort, serAddr, unit) -- unit 6 matches
# teled_d.proto/teled_h.proto's shipped default ($(u=6), never overridden
# anywhere upstream).
TeledyneDInit("$(TELD_PORT)", "$(TELD_TTY)", 0, 6)

dbLoadRecords("db/teledynePumpD.template", "P=$(P),PUMP=D1:,s=SP,ta=Teledyne,ss=D1,PORT=$(TELD_PORT),ADDR=0")

# ------------------------------------------------------------------
# Teledyne H-series (demo instance -- see header comment).
# ------------------------------------------------------------------
epicsEnvSet("TELH_PORT", "TelH1")
epicsEnvSet("TELH_TTY", "TelH1_tty")
drvAsynSerialPortConfigure("$(TELH_TTY)", "/dev/ttyS1", 0, 0, 0)
asynSetTraceIOMask("$(TELH_TTY)", 0, HEX)
TeledyneHInit("$(TELH_PORT)", "$(TELH_TTY)", 0, 6)

dbLoadRecords("db/teledynePumpH.template", "s=SP,ta=Teledyne,ss=H1,PORT=$(TELH_PORT)")

# ------------------------------------------------------------------
# ISCO -- wiring and drvModbusAsynConfigure arguments taken verbatim from
# the real upstream epics-modules/SyringePump/iocBoot/iocISCO/st.cmd.
# ------------------------------------------------------------------
epicsEnvSet("PREFIX", "ISCO1:")
epicsEnvSet("ISCO_PORT", "SP1")
epicsEnvSet("POLL_MS", "1000")
epicsEnvSet("TIMEOUT_MS", "2000")

drvAsynIPPortConfigure("$(ISCO_PORT)", "gse-isco1:502", 0, 0, 0)
asynSetTraceIOMask("$(ISCO_PORT)", 0, HEX)
asynSetTraceIOTruncateSize("$(ISCO_PORT)", 0, 256)
modbusInterposeConfig("$(ISCO_PORT)", 0, $(TIMEOUT_MS), 0)

# Access 142 bits (0-141) as inputs. Function code=1.
drvModbusAsynConfigure("$(ISCO_PORT)_Bit_In",  "$(ISCO_PORT)", 1, 1,  0, 142, 0, $(POLL_MS), "Teledyne")
# Access 109 bits (0-108) as outputs. Function code=5.
drvModbusAsynConfigure("$(ISCO_PORT)_Bit_Out", "$(ISCO_PORT)", 1, 5,  0, 109, 0, 1, "Teledyne")
# Access first set of 100 16-bit holding registers starting at 0 as inputs. Function code=3. Data type=FLOAT32_BE
drvModbusAsynConfigure("$(ISCO_PORT)_Reg_In_1",  "$(ISCO_PORT)", 1,  3, 0, 100, FLOAT32_BE, $(POLL_MS), "Teledyne")
# Access second set of 62 16-bit holding registers starting at 100 as inputs. Function code=3. Data type=FLOAT32_BE
drvModbusAsynConfigure("$(ISCO_PORT)_Reg_In_2",  "$(ISCO_PORT)", 1,  3, 100, 62, FLOAT32_BE, $(POLL_MS), "Teledyne")
# Access third set of 46 16-bit holding registers starting at 200 as inputs. Function code=3. Data type=FLOAT32_BE
drvModbusAsynConfigure("$(ISCO_PORT)_Reg_In_3",  "$(ISCO_PORT)", 1,  3, 200, 46, FLOAT32_BE, $(POLL_MS), "Teledyne")
# Access first set of 100 16-bit holding registers starting at 0 as outputs. Function code=16. Data type=FLOAT32_BE
drvModbusAsynConfigure("$(ISCO_PORT)_Reg_Out_1",  "$(ISCO_PORT)", 1,  16, 0, 100, FLOAT32_BE, 1, "Teledyne")
# Access second set of 62 16-bit holding registers starting at 100 as outputs. Function code=16. Data type=FLOAT32_BE
drvModbusAsynConfigure("$(ISCO_PORT)_Reg_Out_2",  "$(ISCO_PORT)", 1,  16, 100, 62, FLOAT32_BE, 1, "Teledyne")

# Load the substitutions files for the records that use Modbus via
# dbLoadTemplate (see this file's header comment). Order matches upstream
# st.cmd exactly.

# Ported from epics-modules/SyringePump/SPApp/Db/ISCOBinaryIn.substitutions via dbLoadTemplate (142 rows).
dbLoadTemplate("db/ISCOBinaryIn.substitutions", "P=$(PREFIX)")

# Ported from epics-modules/SyringePump/SPApp/Db/ISCOBinaryOut.substitutions via dbLoadTemplate (98 rows).
dbLoadTemplate("db/ISCOBinaryOut.substitutions", "P=$(PREFIX)")

# Ported from epics-modules/SyringePump/SPApp/Db/ISCOAnalogIn.substitutions via dbLoadTemplate (104 rows).
# Upstream defect fixed in the local copy: row "D:RefillRateSP_RBV" is
# missing the comma between the R and PORT columns present on every
# sibling A/B/C row; see db/ISCOAnalogIn.substitutions's own header.
dbLoadTemplate("db/ISCOAnalogIn.substitutions", "P=$(PREFIX)")

# Ported from epics-modules/SyringePump/SPApp/Db/ISCOAnalogOut.substitutions via dbLoadTemplate (59 rows).
dbLoadTemplate("db/ISCOAnalogOut.substitutions", "P=$(PREFIX)")

# Load a database with other records for the controller
dbLoadRecords("db/ISCOController.template", "P=$(PREFIX)")

# Load a database with other records for each pump (A, B, AB only --
# matching upstream's own commented-out C/D/CD instantiations)
dbLoadRecords("db/ISCOPumpN.template", "P=$(PREFIX), PUMP=A:")
dbLoadRecords("db/ISCOPumpN.template", "P=$(PREFIX), PUMP=B:")
#dbLoadRecords("db/ISCOPumpN.template", "P=$(PREFIX), PUMP=C:")
#dbLoadRecords("db/ISCOPumpN.template", "P=$(PREFIX), PUMP=D:")
dbLoadRecords("db/ISCOPumpN.template", "P=$(PREFIX), PUMP=AB:")
#dbLoadRecords("db/ISCOPumpN.template", "P=$(PREFIX), PUMP=CD:")

# ------------------------------------------------------------------
# Vindum -- wiring and drvModbusAsynConfigure arguments taken verbatim
# from the real upstream epics-modules/SyringePump/iocBoot/iocVindum/st.cmd.
# modbusInterposeConfig linkType=1 (RTU) matches upstream: the VP pump
# is Modbus RTU framing carried over a TCP terminal-server link, not
# native Modbus/TCP (upstream's own st.cmd comments this distinction).
# VPREFIX/VINDUM_PORT/VPOLL_MS/VTIMEOUT_MS (rather than upstream's
# PREFIX/PORT/POLL_MS/TIMEOUT_MS) avoid clobbering the ISCO section's
# same-named macros above, since both families are wired in one st.cmd.
# ------------------------------------------------------------------
epicsEnvSet("VPREFIX", "VINDUM1:P1:")
epicsEnvSet("VINDUM_PORT", "SP2")
epicsEnvSet("VPOLL_MS", "100")
epicsEnvSet("VTIMEOUT_MS", "2000")

# This is for a Digi One SP in TCP sockets mode (matching upstream).
drvAsynIPPortConfigure("$(VINDUM_PORT)", "gsets22:4002", 0, 0, 0)
asynSetTraceIOMask("$(VINDUM_PORT)", 0, HEX)
asynSetTraceMask("$(VINDUM_PORT)", 0, ERROR|DRIVER)
asynSetTraceIOTruncateSize("$(VINDUM_PORT)", 0, 256)
modbusInterposeConfig("$(VINDUM_PORT)", 1, $(VTIMEOUT_MS), 0)

# Read 31 bits starting at address 0. Function code=1. Default data type=UINT16
drvModbusAsynConfigure("$(VINDUM_PORT)_ReadCoils", "$(VINDUM_PORT)", 1, 1, 0, 31, UINT16, $(VPOLL_MS), "Vindum")
# Ported from epics-modules/SyringePump/SPApp/Db/VindumReadCoils.substitutions via dbLoadTemplate (6 rows).
dbLoadTemplate("db/VindumReadCoils.substitutions", "P=$(VPREFIX),PORT=$(VINDUM_PORT)_ReadCoils")

# Write 31 bits starting at address 0. Function code=5. Default data type=UINT16
drvModbusAsynConfigure("$(VINDUM_PORT)_WriteCoils", "$(VINDUM_PORT)", 1, 5, 0, 31, UINT16, $(VPOLL_MS), "Vindum")
# Ported from epics-modules/SyringePump/SPApp/Db/VindumWriteCoils.substitutions via dbLoadTemplate (27 rows).
dbLoadTemplate("db/VindumWriteCoils.substitutions", "P=$(VPREFIX),PORT=$(VINDUM_PORT)_WriteCoils")

# Read 6 bits starting at address 0. Function code=2. Default data type=UINT16
drvModbusAsynConfigure("$(VINDUM_PORT)_ReadContacts", "$(VINDUM_PORT)", 1, 2, 0, 6, UINT16, $(VPOLL_MS), "Vindum")
# Ported from epics-modules/SyringePump/SPApp/Db/VindumReadContacts.substitutions via dbLoadTemplate (6 rows).
dbLoadTemplate("db/VindumReadContacts.substitutions", "P=$(VPREFIX),PORT=$(VINDUM_PORT)_ReadContacts")

# Read 42 16-bit analog input registers starting at 0. Function code=4. Default data type=UINT16
drvModbusAsynConfigure("$(VINDUM_PORT)_ReadInputRegs", "$(VINDUM_PORT)", 1, 4, 0, 42, UINT16, $(VPOLL_MS), "Vindum")
# Ported from epics-modules/SyringePump/SPApp/Db/VindumReadInputRegisters.substitutions via dbLoadTemplate (23 rows across 3 templates).
dbLoadTemplate("db/VindumReadInputRegisters.substitutions", "P=$(VPREFIX),PORT=$(VINDUM_PORT)_ReadInputRegs")

# Read 46 16-bit holding registers starting at 0. Function code=3. Default data type=UINT16
drvModbusAsynConfigure("$(VINDUM_PORT)_ReadHoldingRegs", "$(VINDUM_PORT)", 1, 3, 0, 46, UINT16, $(VPOLL_MS), "Vindum")
# Ported from epics-modules/SyringePump/SPApp/Db/VindumReadHoldingRegisters.substitutions via dbLoadTemplate (30 rows across 3 templates).
dbLoadTemplate("db/VindumReadHoldingRegisters.substitutions", "P=$(VPREFIX),PORT=$(VINDUM_PORT)_ReadHoldingRegs")

# Write 46 16-bit holding registers starting at 0. Function code=16. Default data type=UINT16
drvModbusAsynConfigure("$(VINDUM_PORT)_WriteHoldingRegs", "$(VINDUM_PORT)", 1, 16, 0, 46, UINT16, $(VPOLL_MS), "Vindum")
# Ported from epics-modules/SyringePump/SPApp/Db/VindumWriteHoldingRegisters.substitutions via dbLoadTemplate (30 rows across 3 templates).
dbLoadTemplate("db/VindumWriteHoldingRegisters.substitutions", "P=$(VPREFIX),PORT=$(VINDUM_PORT)_WriteHoldingRegs")

dbLoadRecords("db/VindumController.template", "P=$(VPREFIX), PORT=$(VINDUM_PORT)")
dbLoadRecords("db/VindumPumpN.template", "P=$(VPREFIX), PORT=$(VINDUM_PORT), PUMP=A:")
dbLoadRecords("db/VindumPumpN.template", "P=$(VPREFIX), PORT=$(VINDUM_PORT), PUMP=B:")

###############################################################################
iocInit()
###############################################################################
