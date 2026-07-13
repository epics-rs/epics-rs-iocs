# OPC UA IOC startup script (exampleTop/iocBoot/iocopcua/st.cmd).
#
#   cargo run -p opcua-ioc -- iocs/opcua-ioc/st.cmd
#
# The example database talks to the Unified Automation UaServerCpp demo server;
# point OPCUA_URL at whatever server you have.

epicsEnvSet("P",         "OPC:")
epicsEnvSet("SESS",      "OPC1")
epicsEnvSet("SUBS",      "SUB1")
epicsEnvSet("OPCUA_URL", "opc.tcp://localhost:48010")
epicsEnvSet("OPCUA",     "iocs/opcua-ioc")

# Module defaults, before the sessions and the databases that pick them up.
# (The C reads these from its .dbd `variable()` entries; this IOC's `var`
# command is what sets them here — see the framework gap in src/main.rs.)
#var opcua_DefaultPublishInterval 100.0
#var opcua_DefaultServerQueueSize 1
#var opcua_ConnectTimeout 5.0

# One session, with a 200 ms subscription on top.
opcuaSession $(SESS) $(OPCUA_URL)
opcuaSubscription $(SUBS) $(SESS) 200

# No security. With security: sec-mode=SignAndEncrypt sec-policy=Basic256Sha256,
# a client certificate (opcuaClientCertificate) and a PKI store (opcuaSetupPKI).
opcuaOptions $(SESS) sec-mode=None

# The demo server >= v1.8 publishes the demo nodes in ns=3, the database uses
# ns=2; map the database's index onto the server's namespace URI.
#opcuaMapNamespace $(SESS) 2 "http://www.unifiedautomation.com/DemoServer/"

#opcuaClientCertificate("pki/own/certs/client.der", "pki/own/private/client.pem")
#opcuaSetupPKI("pki")

dbLoadRecords("$(OPCUA)/db/opcuaExample.db", "P=$(P),SESS=$(SESS),SUBS=$(SUBS)")

# What is configured, and what each session's items and records are (verbosity 1
# and 2). The items exist once the records have bound, which is after iocInit, so
# this has to be queued for then — a plain line below iocInit() would run before
# it, as the framework runs the whole script and then inits.
#afterIocRunning "opcuaShow * 2"

iocInit()
