# Port of quadEM iocBoot/saveRestore.cmd: quadEMApp/Db and ADApp/Db are
# $(QUADEM)/db and $(ADCORE)/db here.
set_requestfile_path("./")
set_requestfile_path("$(QUADEM)/db")
set_requestfile_path("$(ADCORE)/db")
set_requestfile_path("$(CALC)/calcApp/Db")
set_requestfile_path("$(SSCAN)/sscanApp/Db")
set_savefile_path("./autosave")
set_pass0_restoreFile("auto_settings.sav")
set_pass1_restoreFile("auto_settings.sav")
save_restoreSet_status_prefix("$(PREFIX)")
dbLoadRecords("$(AUTOSAVE)/asApp/Db/save_restoreStatus.db", "P=$(PREFIX)")
