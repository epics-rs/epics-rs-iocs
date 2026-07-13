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
#asynOctetSetInputEos("$(PORT)", 0, "\006")
#asynOctetSetOutputEos("$(PORT)", 0, "\r")

# ---- controller ----
# pmacCreateController(controllerName, lowLevelPortName, lowLevelPortAddress,
#                      numAxes, [movingPollMs], [idlePollMs], [deferredMode],
#                      [feedRatePoll], [feedRateLimit])
#   deferredMode:  1 = fast (one jog line), 2 = coordinated (motion program 101)
#   feedRatePoll:  != 0 polls the global feed rate ("%") and raises the record's
#                  PROBLEM bit when it falls below feedRateLimit percent
pmacCreateController("$(CTRL)", "$(PORT)", 0, $(NUMAXES), 100, 1000, 1, 0, 100)

# Create axes 1..NUMAXES (DTYP PMAC_$(CTRL)_{1..NUMAXES}).
pmacCreateAxes("$(CTRL)", $(NUMAXES))

# Optional per-axis configuration:
#   an open-loop axis whose encoder is read back from another axis
#pmacSetOpenLoopEncoderAxis("$(CTRL)", 1, 5, 1.0)
#   stop raising PROBLEM when the controller reports hardware limits disabled
#pmacDisableLimitsCheck("$(CTRL)", 1, 0)

# ---- coordinate-system groups (optional) ----
# Define a grouping of real axes onto coordinate systems, then switch to it.
# pmacCreateCsGroup(controllerName, groupNumber, groupName, axisCount)
# pmacCsGroupAddAxis(controllerName, groupNumber, axis, axisDef, cs)
#pmacCreateCsGroup("$(CTRL)", 1, "Direct", 3)
#pmacCsGroupAddAxis("$(CTRL)", 1, 1, "X", 1)
#pmacCsGroupAddAxis("$(CTRL)", 1, 2, "Y", 1)
#pmacCsGroupAddAxis("$(CTRL)", 1, 3, "Z", 1)
#pmacCsGroupSwitch("$(CTRL)", 1)

# ---- coordinate-system axes (optional) ----
# One driver per coordinate system: 9 kinematic axes A,B,C,U,V,W,X,Y,Z on
# Q71..Q79 (demand) / Q81..Q89 (readback), moved by motion program `program`.
# pmacAsynCoordCreate(csName, lowLevelPortName, lowLevelPortAddress, cs, program,
#                     [movingPollMs], [idlePollMs])
#pmacAsynCoordCreate("CS2", "$(PORT)", 0, 2, 10, 100, 500)
#dbLoadRecords("db/pmac_cs.template", "P=$(P),M=cs2x,CS=CS2,AXIS=7")

# ---- records ----
dbLoadRecords("db/pmac.template", "P=$(P),M=m1,CTRL=$(CTRL),AXIS=1")
dbLoadRecords("db/pmac.template", "P=$(P),M=m2,CTRL=$(CTRL),AXIS=2")

iocInit()

# Example:
#   dbl
#   camonitor pmac:m1 pmac:m1.RBV
#   caput pmac:m1 10000
