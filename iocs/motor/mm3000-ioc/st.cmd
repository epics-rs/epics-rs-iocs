#============================================================
# st.cmd — Newport MM3000 motor IOC startup script
#
# Usage:
#   cargo run -p mm3000-ioc -- st.cmd
#
# Requires a Newport MM3000 controller on the serial line below.
# MM3000Config probes the controller (VE) and reads the axis configuration
# (RC), so it must be connected when the command runs.
#============================================================

epicsEnvSet("P",      "MM3000:")
epicsEnvSet("SERIAL", "SERIAL1")
epicsEnvSet("TTY",    "/dev/ttyUSB0")

# ---- serial octet port ----
drvAsynSerialPortConfigure("$(SERIAL)", "$(TTY)")

# MM3000 serial line: 9600 baud, 8 data bits, no parity, 1 stop bit
# (controller default; match the rear-panel setting).
asynSetOption("$(SERIAL)", 0, "baud",   "9600")
asynSetOption("$(SERIAL)", 0, "bits",   "8")
asynSetOption("$(SERIAL)", 0, "parity", "none")
asynSetOption("$(SERIAL)", 0, "stop",   "1")

# Port EOS as the C example sets it (MM3000.cmd:17-20);
# the driver sends bare commands.
asynOctetSetInputEos("$(SERIAL)", 0, "\r")
asynOctetSetOutputEos("$(SERIAL)", 0, "\r")

# ---- MM3000 controller (axis configuration read automatically) ----
# MM300Setup(num_cards, scan_rate in Hz): C registers the command under this
# name (NewportRegister.cc), not MM3000Setup.
MM300Setup(1, 10)
# MM3000Config(card, asyn port, GPIB address)
MM3000Config(0, "$(SERIAL)")

# One motor record per configured axis, signal S of card 0.
# The MM3000 wire is step-native, so MRES=1 (EGU = motor steps / encoder
# counts) — like the AG-UC driver.
dbLoadRecords("$(MM3000_IOC)/db/mm3000.template", "P=$(P),M=m1,C=0,S=0")

iocInit()

# Example:
#   dbl
#   camonitor MM3000:m1 MM3000:m1.RBV
#   caput MM3000:m1 5000
