#!../../bin/ur-robot-ioc
# Universal Robots IOC startup script -*- shell-script -*-
#
# Port of epics-modules/urRobot: iocs/urExample/iocBoot/iocurExample/st.cmd.Linux
# plus urRobotApp/iocsh/urRobot.iocsh, flattened into one script.
#
# The Rust IOC needs no dbLoadDatabase / registerRecordDeviceDriver: the record
# types are built in and the asyn device support is registered from main.rs.
#
# Order matters. RTDEControlConfig and URGripperConfig look their dependencies up
# by asyn port name, so the dashboard port must exist before either, and the
# receive port before the control port.

# Overridable from the process environment: export PREFIX/IP before
# launching to retarget without editing this file. URROBOT defaults to
# the crate directory (main.rs) so the db paths work from any cwd.
epicsEnvSet("PREFIX", "$(PREFIX=urExample:)")
epicsEnvSet("IP", "$(IP=192.168.101.42)")

# Dashboard server, TCP 29999. Supplies the robot IP and power state to the
# control and gripper ports. The surface is split so a monitoring IOC can
# load dashboard.db (status + Connect/Disconnect) without dashboard_ctrl.db
# (Play/Stop/PowerOff/... — everything that drives the robot).
URDashboardConfig("dash", "$(IP)", 0.1)
dbLoadRecords("$(URROBOT)/db/dashboard.db", "P=$(PREFIX),PORT=dash")
dbLoadRecords("$(URROBOT)/db/dashboard_ctrl.db", "P=$(PREFIX),PORT=dash")

# RTDE receive, TCP 30004. Supplies the safety word to the control port.
RTDEReceiveConfig("rtde_recv", "$(IP)", 0.02)
dbLoadRecords("$(URROBOT)/db/rtde_receive.db", "P=$(PREFIX),PORT=rtde_recv")

# RTDE inputs (digital / analog outputs, speed slider). Write-only: the poll
# period is accepted for command compatibility and never used.
RTDEInOutConfig("rtde_io", "$(IP)", 0.1)
dbLoadRecords("$(URROBOT)/db/rtde_io.db", "P=$(PREFIX),PORT=rtde_io")

# RTDE control: motion, teach mode, custom URScript.
RTDEControlConfig("rtde_ctrl", "dash", "rtde_recv", 0.02)
dbLoadRecords("$(URROBOT)/db/rtde_control.db", "P=$(PREFIX),PORT=rtde_ctrl")

# Jogging PVs on the same rtde_ctrl port. Upstream leaves these out of
# urRobot.iocsh (docs/usage.md:115); loading them here is safe because the
# jog parameters stage IOC-local state, so their PINI processing at boot
# needs no link. The 1 s watchdog records ship in the db itself.
dbLoadRecords("$(URROBOT)/db/rtde_control_jog.db", "P=$(PREFIX),PORT=rtde_ctrl")

# Robotiq gripper URCap, TCP 63352 on the robot's own IP.
URGripperConfig("gripper", "dash", 0.02)
dbLoadRecords("$(URROBOT)/db/robotiq_gripper.db", "P=$(PREFIX),MIN_POS=3,MAX_POS=248,AUTO_ACTIVATE=YES,PORT=gripper")

iocInit()
