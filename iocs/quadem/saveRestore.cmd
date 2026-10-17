# Port of quadEM iocBoot/saveRestore.cmd: quadEMApp/Db and ADApp/Db are
# $(QUADEM)/db and $(ADCORE)/db here. C's $(CALC) and $(SSCAN) search paths
# are left out: no request file the quadEM IOCs read is in either module,
# and epics-rs has no calc module to define $(CALC).
set_requestfile_path("./")
set_requestfile_path("$(QUADEM)/db")
set_requestfile_path("$(ADCORE)/db")
set_savefile_path("./autosave")
set_pass0_restoreFile("auto_settings.sav")
set_pass1_restoreFile("auto_settings.sav")
save_restoreSet_status_prefix("$(PREFIX)")
dbLoadRecords("$(AUTOSAVE)/db/save_restoreStatus.db", "P=$(PREFIX)")
