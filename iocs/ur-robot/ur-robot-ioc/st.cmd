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

epicsEnvSet("PREFIX", "urExample:")
epicsEnvSet("IP", "192.168.101.42")

# Dashboard server, TCP 29999. Supplies the robot IP and power state to the
# control and gripper ports.
URDashboardConfig("dash", "$(IP)", 0.1)
dbLoadRecords("iocs/ur-robot/ur-robot-ioc/db/dashboard.db", "P=$(PREFIX),PORT=dash")

# RTDE receive, TCP 30004. Supplies the safety word to the control port.
RTDEReceiveConfig("rtde_recv", "$(IP)", 0.02)
dbLoadRecords("iocs/ur-robot/ur-robot-ioc/db/rtde_receive.db", "P=$(PREFIX),PORT=rtde_recv")

# RTDE inputs (digital / analog outputs, speed slider). Write-only: the poll
# period is accepted for command compatibility and never used.
RTDEInOutConfig("rtde_io", "$(IP)", 0.1)
dbLoadRecords("iocs/ur-robot/ur-robot-ioc/db/rtde_io.db", "P=$(PREFIX),PORT=rtde_io")

# RTDE control: motion, teach mode, custom URScript.
RTDEControlConfig("rtde_ctrl", "dash", "rtde_recv", 0.02)
dbLoadRecords("iocs/ur-robot/ur-robot-ioc/db/rtde_control.db", "P=$(PREFIX),PORT=rtde_ctrl")

# Optional: jogging PVs. Upstream does not load these from urRobot.iocsh either
# (docs/usage.md:115); they drive the same rtde_ctrl port.
# dbLoadRecords("iocs/ur-robot/ur-robot-ioc/db/rtde_control_jog.db", "P=$(PREFIX),PORT=rtde_ctrl")

# Robotiq gripper URCap, TCP 63352 on the robot's own IP.
URGripperConfig("gripper", "dash", 0.02)
dbLoadRecords("iocs/ur-robot/ur-robot-ioc/db/robotiq_gripper.db", "P=$(PREFIX),MIN_POS=3,MAX_POS=248,AUTO_ACTIVATE=YES,PORT=gripper")

iocInit()
