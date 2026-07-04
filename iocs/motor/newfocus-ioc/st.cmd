#============================================================
# st.cmd — Newport New Focus PMNC 8750/8752 picomotor IOC startup script
#
# Usage:
#   cargo run -p newfocus-ioc -- st.cmd
#
# Requires a New Focus PMNC 8750/8752 network controller reachable at the
# HOST below. PMNC87xxCreateController identifies the controller (VER) and
# autodiscovers its driver modules and channels (MPV/DRT) at startup, so it
# must be reachable when the command runs.
#============================================================

epicsEnvSet("P",     "NF:")
epicsEnvSet("PORT",  "MOTOR1")
epicsEnvSet("ASYN",  "PMNC1")
epicsEnvSet("HOST",  "192.168.0.100:23")

# ---- TCP octet port ----
drvAsynIPPortConfigure("$(ASYN)", "$(HOST)", 0, 0, 1)

# PMNC prompt protocol: replies are terminated by the '>' prompt. The driver
# appends the CR command terminator itself, so only the input EOS is set.
asynOctetSetInputEos("$(ASYN)", 0, ">")

# ---- PMNC controller ----
# PMNC87xxCreateController(motorPort, asynPort, [movingPollMs], [idlePollMs])
# Autodiscovers driver modules and channels; one motor axis per channel.
PMNC87xxCreateController("$(PORT)", "$(ASYN)", 100, 1000)

# One motor record per discovered channel (DTYP PMNC_$(PORT)_0, _1, ...).
# Picomotor moves are integer steps: positions travel the driver boundary in
# EGU (= steps) directly, so MRES=1 and EGU=steps.
dbLoadRecords("db/newfocus.template", "P=$(P),M=m1,N=0,PORT=$(PORT),MRES=1,EGU=steps")

iocInit()

# Example:
#   dbl
#   camonitor NF:m1 NF:m1.RBV
#   caput NF:m1 1000
