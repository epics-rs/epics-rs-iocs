#============================================================
# st.cmd — Delta Tau Turbo PMAC / Geobrick motor IOC (tpmac)
#
# Usage:
#   cargo run -p pmac-ioc -- st.cmd
#
# Requires a Turbo PMAC or Geobrick reachable on TCP port 1025 (the PMAC
# ethernet protocol). The controller must have I3=2 and I6=1 so that replies are
# ACK-terminated and errors are reported — the ethernet framing layer relies on
# both.
#============================================================

epicsEnvSet("P",       "pmac:")
epicsEnvSet("PORT",    "PMACport")
epicsEnvSet("HOST",    "192.168.1.100:1025")
epicsEnvSet("CTRL",    "PMAC1")
epicsEnvSet("NUMAXES", "8")

# ---- octet port ----
# PMAC ethernet: creates the IP port and installs the PMAC packet framing
# (header + READREADY/GETBUFFER response pulls) plus the EOS layer above it,
# so the driver above sees a plain ASCII port. This is C pmacAsynIPConfigure.
pmacAsynIPConfigure("$(PORT)", "$(HOST)")

# A PMAC on a serial line (or a terminal server passing raw ASCII) instead:
#drvAsynSerialPortConfigure("$(PORT)", "/dev/ttyS0", 0, 0, 0)
#asynSetOption("$(PORT)", -1, "baud", "38400")

# ---- controller ----
# pmacCreateController(portName, lowLevelPortName, lowLevelPortAddress,
#                      numAxes, movingPollPeriod, idlePollPeriod), periods in ms
pmacCreateController("$(CTRL)", "$(PORT)", 0, $(NUMAXES), 100, 1000)

# Create axes 1..NUMAXES; records reach axis n as @asyn($(CTRL),n).
pmacCreateAxes("$(CTRL)", $(NUMAXES))

# Optional per-axis configuration:
#   an open-loop axis whose encoder is read back from another axis
#pmacSetOpenLoopEncoderAxis("$(CTRL)", 1, 5)
#   stop raising PROBLEM when the controller reports hardware limits disabled
#pmacDisableLimitsCheck("$(CTRL)", 1, 0)

# ---- coordinate-system groups (optional) ----
# pmacCreateCsGroup(controller, groupNo, groupName, axisCount)
# pmacCsGroupAddAxis(controller, groupNo, axisNo, mapping, coordinateSysNo)
#pmacCreateCsGroup("$(CTRL)", 1, "Direct", 3)
#pmacCsGroupAddAxis("$(CTRL)", 1, 1, "X", 1)
#pmacCsGroupAddAxis("$(CTRL)", 1, 2, "Y", 1)
#pmacCsGroupAddAxis("$(CTRL)", 1, 3, "Z", 1)

# ---- coordinate-system axes (optional) ----
# 9 kinematic axes A,B,C,U,V,W,X,Y,Z on Q71..Q79 (demand) / Q81..Q89
# (readback), moved by motion program `program`; CS axis A is address 0.
# pmacAsynCoordCreate(port, address, C.S. no, reference, program)
# drvAsynMotorConfigure(portName, "pmacAsynCoord", reference, 9)
#pmacAsynCoordCreate("$(PORT)", 0, 2, 0, 10)
#pmacSetCoordIdlePollPeriod(0, 500)
#pmacSetCoordMovingPollPeriod(0, 100)
#drvAsynMotorConfigure("CS2", "pmacAsynCoord", 0, 9)
#dbLoadRecords("$(PMAC_IOC)/db/pmac_cs.template", "P=$(P),M=cs2x,PORT=CS2,ADDR=6")

# ---- records ----
dbLoadRecords("$(PMAC_IOC)/db/pmac.template", "P=$(P),M=m1,PORT=$(CTRL),ADDR=1")
dbLoadRecords("$(PMAC_IOC)/db/pmac.template", "P=$(P),M=m2,PORT=$(CTRL),ADDR=2")

iocInit()

# Example:
#   dbl
#   camonitor pmac:m1 pmac:m1.RBV
#   caput pmac:m1 10000
