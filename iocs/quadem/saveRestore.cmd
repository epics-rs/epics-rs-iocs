# Port of quadEM iocBoot/saveRestore.cmd: quadEMApp/Db is $(QUADEM)/db here.
# The rest of C's file repeats the autosave block of commonPlugins.cmd, which
# quadEM_Plugins.cmd has already run, so only the quadEM path is left. C's
# $(CALC) and $(SSCAN) search paths are left out: no request file the quadEM
# IOCs read is in either module, and epics-rs has no calc module to define
# $(CALC).
set_requestfile_path("$(QUADEM)/db")
