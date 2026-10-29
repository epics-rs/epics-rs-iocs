# Port of quadEM iocBoot/saveRestore.cmd: quadEMApp/Db and calcApp/Db are
# $(QUADEM)/db and $(CALC)/db here. The rest of C's file repeats the autosave
# block of commonPlugins.cmd, which quadEM_Plugins.cmd has already run. C's
# $(SSCAN) search path is left out: no request file the quadEM IOCs read is
# in sscan.
set_requestfile_path("$(QUADEM)/db")
set_requestfile_path("$(CALC)/db")
