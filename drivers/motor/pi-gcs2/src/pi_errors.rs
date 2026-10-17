//! C `TranslatePIError` (TranslatePIError.cpp), generated from it and
//! `picontrollererrors.h`: the message for a GCS error code.

/// The message C's `TranslatePIError` gives `error`; `None` is its
/// `default`, "Unknown Error - number {error}".
pub(crate) fn message(error: i32) -> Option<&'static str> {
    Some(match error {
        -10016 => {
            "Parameters could not be set on controller because the corresponding firmware feature is missing"
        }
        -10015 => {
            "One or more parameter definitions are not present in stage database. Please update PISTAGES3.DB via PIUpdateFinder. Missing parameters were ignored."
        }
        -10014 => "One or more parameters could not be set correctly on the controller.",
        -10013 => {
            "Mismatch between number of parameters present in stage database and available in controller interface. Some parameters were ignored."
        }
        -10012 => {
            "Mismatch between number of parameters present in stage database and available in controller interface. No parameters were set."
        }
        -10011 => {
            "The version of PISTAGES3.DB stage database is out of date. Please update via PIUpdateFinder. No parameters were set."
        }
        -10010 => {
            "Controller does not support all stage parameters stored in PI stage database. No parameters were set."
        }
        -10009 => "PI stage database: Stage has not been removed.",
        -10008 => "PI stage database: Stage has not been added.",
        -10007 => "PI stage database: Inconsistency between database and response to HPA?.",
        -10006 => "PI stage database: Response to HPA? contains unknown parameter IDs.",
        -10005 => "PI stage database: System already exists. Rename stage and try again.",
        -10004 => "PI stage database: Querying data failed.",
        -10003 => {
            "PI stage database: Communication was interrupted (e.g. because database was deleted)."
        }
        -10002 => "PI stage database: Establishing the connection has failed.",
        -10001 => {
            "PI stage database: Database does not contain the selected stage type for the connected controller."
        }
        -10000 => {
            "PI stage database: String containing stage type and description has invalid format."
        }
        -1118 => {
            "Critical error indicated for system, check error log with \"LOG?\" to find more details."
        }
        -1117 => {
            "Runtime error indicated for axis, check error log with \"LOG?\" to find more details."
        }
        -1116 => "Not enough memory",
        -1115 => "Sending Controller or controller version does not exist in PIStages database.",
        -1114 => "Sending MAC_DEF or receiving the response has failed.",
        -1113 => "Sending IDN? or receiving the response has failed.",
        -1112 => "Cannot determine actual end of travel range while platform is moving.",
        -1111 => "File has invalid signature",
        -1110 => "File has no signature",
        -1109 => "PAM file has duplicate entry with different values.",
        -1108 => "Error while performing CST: One or more parameters were not set correctly.",
        -1107 => "Response to SPA? could not be received.",
        -1106 => "Response to SEP? could not be received.",
        -1105 => "Unknown parameter",
        -1104 => "Invalid value",
        -1103 => "No PAM file is open",
        -1102 => "No value for parameter available",
        -1101 => "Information does not contain all required data",
        -1100 => "PAM file does not contain required data in PAM-file format",
        -1099 => "Version of PAM file cannot be handled (too old or too new)",
        -1098 => "Response to SPA? could not be received.",
        -1097 => "HPA? response does not comply with GCS2 syntax.",
        -1096 => "Sending HPA? or receiving the response has failed.",
        -1095 => "Connection could not be reestablished after reboot.",
        -1094 => {
            "A parameter could not be set while performing CST: CST was not performed (parameters remain unchanged)."
        }
        -1093 => "Switching off the servo mode has failed.",
        -1092 => "Change of command level has failed.",
        -1091 => "Wrong number of input arguments.",
        -1090 => "Communication error while reading GCS data",
        -1089 => "Controller sent too many GCS data sets",
        -1088 => "Controller sent too few GCS data sets",
        -1087 => "command cancelled by user input.",
        -1086 => "Macro recording is not allowed in this mode of operation.",
        -1085 => "Parameter value not found in parameter DAT file.",
        -1084 => "Length of data arrays is different.",
        -1083 => "A function argument has an unexpected datatype.",
        -1082 => "Timeout Error. Some lengthy operation did not finish within expected time.",
        -1081 => "User stages DAT file does not have correct revision.",
        -1080 => "Parameter DAT file does not have correct revision.",
        -1079 => "Cannot create parameter DAT file to store user defined stage type.",
        -1078 => "Cannot write to parameter DAT file to store user defined stage type.",
        -1077 => {
            "Version of parameter DAT file does not match the required version. Current files are available at www.pi.ws."
        }
        -1076 => "The interface is currently locked by another function. Please try again later.",
        -1075 => {
            "The library used doesn't match the required version. Please see the documentation to determine the required library version."
        }
        -1074 => {
            "The installed device driver doesn't match the required version. Please see the documentation to determine the required device driver version."
        }
        -1073 => "Unknown cluster identifier",
        -1072 => "User Profile Mode: Cluster is not assigned to this axis",
        -1071 => "User Profile Mode: Out of memory",
        -1070 => "ProfileGenerator: Parameter out of allowed range",
        -1069 => "ProfileGenerator: Unknown parameter ID in Set/Get Parameter command",
        -1068 => "ProfileGenerator: Generated profile exceeds limits of one or both axes",
        -1067 => "ProfileGenerator: No profile has been created yet",
        -1066 => "User Profile Mode: Block or Data Set index out of allowed range",
        -1065 => "Controller is (already) in User Profile Mode",
        -1064 => {
            "User Profile Mode: First target position in User Profile is too far from current position"
        }
        -1063 => {
            "User Profile Mode: Command is not allowed, check for required preparatory commands"
        }
        -1062 => "WaveEditor: Graph display component not installed",
        -1061 => "WaveEditor: Error during wave creation, could not calculate value",
        -1060 => {
            "WaveEditor: Error during wave creation, incorrect index for floating point parameter"
        }
        -1059 => "WaveEditor: Error during wave creation, incorrect index for integer parameter",
        -1058 => "WaveEditor: Frequency out of range",
        -1057 => "WaveEditor: Error during wave creation, incorrect number of parameters",
        -1056 => {
            "Error during communication with OPM (Optical Power Meter), maybe no OPM connected"
        }
        -1055 => "Invalid password",
        -1054 => "No wave being output to specified axis",
        -1053 => {
            "The PiStages.dat database could not be found. This file is required to connect a stage with the CST command"
        }
        -1052 => "Checksum error when transfering a file to/from the controller",
        -1051 => "Could not create/read file on host PC",
        -1050 => {
            "A uuencoded file transfered did not start with \"begin\" followed by the proper filename"
        }
        -1049 => "A stage name given to CST is not unique",
        -1048 => "Command ID given to SPP or SPP? is not valid",
        -1047 => "Could not find parameter set required by frequency relation",
        -1046 => "Not all axes could be referenced",
        -1045 => "Wave output to axis already stopped/started",
        -1044 => "No wave defined for specified axis",
        -1043 => "The maximum number of wave generators has been exceeded",
        -1042 => "The maximum number of wave definitions has been exceeded",
        -1041 => "Parameter could not be set with SPA--parameter not defined for this controller!",
        -1040 => "The version of the connected controller is invalid",
        -1039 => "Could not open parameter file",
        -1038 => "There is already a parameter file open. Close it before opening a new file",
        -1037 => "Selected stage does not exist",
        -1036 => "There is no parameter file open",
        -1035 => "The open parameter file is protected and cannot be edited",
        -1034 => "DLL necessary to call function not loaded, or function not found in DLL",
        -1033 => "Error from C-844 Controller",
        -1032 => "The user cancelled a dialog",
        -1031 => "The WGWaveEditor DLL function was not found at startup",
        -1030 => "Could not find description of wave form. Maybe WG.INI is missing?",
        -1029 => "Error while reading/writing wave generator parameter file",
        -1028 => "Unknown channel identifier",
        -1027 => "Current settings are not valid for desired configuration",
        -1026 => "Configuration of PZT stage or amplifier failed",
        -1025 => "Controller is (already) running a macro",
        -1024 => "Motion error: position error too large, servo is switched off automatically",
        -1023 => "Generation of wave failed",
        -1022 => "Incorrect number of samples given to WAV",
        -1021 => "Number for AVG out of range--must be >0",
        -1020 => "Invalid parameter ID given to SPA or SPA?",
        -1019 => "Range of array does not fit the number of parameters",
        -1018 => "Could not access array data in COM server",
        -1017 => "Invalid axis identifier",
        -1016 => "Axis identifier is already in use by a connected stage",
        -1015 => {
            "One or more arguments given to function is invalid (empty string, index out of range, ...)"
        }
        -1014 => "Internal error in macro editor",
        -1013 => "No macro with given name on controller, or macro is empty",
        -1012 => "Could not open file to write or read macro",
        -1011 => "Controller not in macro mode--command not valid unless macro mode active",
        -1010 => "Controller is (already) in macro mode--command not valid in macro mode",
        -1009 => "Internal error--could not start thread",
        -1008 => {
            "Controller is busy with some lengthy operation (e.g. reference move, fast scan algorithm)"
        }
        -1007 => "Axis not currently controlled by a manual control pad",
        -1006 => "Invalid number for manual control pad knob",
        -1005 => {
            "No manual control pad installed, calls to SMA and related commands are not allowed"
        }
        -1004 => "Controller sent unexpected response",
        -1003 => "Invalid value for SGA--must be one of {1, 10, 100, 1000}",
        -1002 => "Number for NAV out of range--must be in [1,10000]",
        -1001 => "Unknown axis identifier",
        -63 => "Device already connected",
        -62 => "Host not found",
        -61 => "Driver could not be opened (on Vista: run as administrator!)",
        -60 => "No Device with this ID is present",
        -59 => "Operation was interrupted by a nonblocked signal",
        -58 => "Can't connect to peer",
        -57 => "Connection terminated (by peer)",
        -56 => "Socket not connected (or not valid)",
        -55 => "Port is used by another socket",
        -54 => "Socket not ready, you should call the function again",
        -53 => "Checksum error in received data from COM port",
        -52 => "Error while opening the COM port: was already open",
        -51 => "FTDIUSB: other error",
        -50 => "FTDIUSB: not supported",
        -49 => "FTDIUSB: invalid arguments",
        -48 => "FTDIUSB: EEPROM not programmed",
        -47 => "FTDIUSB: EEPROM not present",
        -46 => "FTDIUSB: EEPROM erase failed",
        -45 => "FTDIUSB: EEPROM write failed",
        -44 => "FTDIUSB: EEPROM read failed",
        -43 => "FTDIUSB: failed to write device",
        -42 => "FTDIUSB: device not opened for write",
        -41 => "FTDIUSB: device not opened for erase",
        -40 => "FTDIUSB: invalid baud rate",
        -39 => "FTDIUSB: invalid parameter",
        -38 => "FTDIUSB: insufficient resources",
        -37 => "FTDIUSB: IO error",
        -36 => "FTDIUSB: device not opened",
        -35 => "FTDIUSB: device not found",
        -34 => "FTDIUSB: invalid handle",
        -33 => "A DLL or one of the required functions could not be loaded",
        -32 => "Error dealing with internal system resources (events, threads, ...)",
        -31 => "RS-232: Error configuring the COM port",
        -30 => {
            "RS-232: 5 data bits with 2 stop bits is an invalid combination, as is 6, 7, or 8 data bits with 1.5 stop bits"
        }
        -29 => "IEEE488: Address or board locked",
        -28 => "IEEE488: Return buffer full",
        -27 => "IEEE488: SRQ remains asserted",
        -26 => "IEEE488: Serial poll-status byte lost",
        -25 => "IEEE488: Command error during device call",
        -24 => "IEEE488: File system operation error",
        -23 => "IEEE488: No capability for intended operation",
        -22 => "IEEE488: I/O operation started before previous operation completed",
        -21 => "IEEE488: Error performing DMA",
        -20 => "IEEE488: Interface board not found",
        -19 => "IEEE488: I/O operation aborted",
        -18 => "IEEE488: Function requires GPIB board to be SAC",
        -17 => "IEEE488: Invalid argument to function call",
        -16 => "IEEE488: Interface board not addressed correctly",
        -15 => "IEEE488: Write function detected no listeners",
        -14 => "IEEE488: Function requires GPIB board to be CIC",
        -13 => "IEEE488: System error",
        -12 => "Error while sending \"echoed\" data",
        -11 => "Function not supported by this interface type",
        -10 => "Event/message for notification could not be opened",
        -9 => "There is no interface or DLL handle with the given ID",
        -8 => "There are more lines waiting in buffer",
        -7 => "Timeout error",
        -6 => "Error while opening port",
        -5 => "Buffer overflow",
        -4 => "Not connected (no port with given ID open)",
        -3 => "Error while receiving data",
        -2 => "Error while sending data",
        -1 => "Error during com operation (could not be specified)",
        0 => "No error",
        1 => "Parameter syntax error",
        2 => "Unknown command",
        3 => "Command length out of limits or command buffer overrun",
        4 => "Error while scanning",
        5 => "Unallowable move attempted on unreferenced axis, or move attempted with servo off",
        6 => "Parameter for SGA not valid",
        7 => "Position out of limits",
        8 => "Velocity out of limits",
        9 => "Attempt to set pivot point while U,V and W not all 0",
        10 => "Controller was stopped by command",
        11 => "Parameter for SST or for one of the embedded scan algorithms out of range",
        12 => "Invalid axis combination for fast scan",
        13 => "Parameter for NAV out of range",
        14 => "Invalid analog channel",
        15 => "Invalid axis identifier",
        16 => "Invalid stage name",
        17 => "Parameter out of range",
        18 => "Invalid macro name",
        19 => "Error while recording macro",
        20 => "Macro not found",
        21 => "Axis has no brake",
        22 => "Axis identifier specified more than once",
        23 => "Illegal axis or channel",
        24 => "Incorrect number of parameters",
        25 => "Invalid floating point number",
        26 => "Parameter missing",
        27 => "Soft limit out of range",
        28 => "No manual pad found",
        29 => "No more step-response values",
        30 => "No step-response values recorded",
        31 => "Axis has no reference sensor",
        32 => "Axis has no limit switch",
        33 => "No relay card installed",
        34 => "Command not allowed for selected stage(s)",
        35 => "No digital input installed",
        36 => "No digital output configured",
        37 => "No more MCM responses",
        38 => "No MCM values recorded",
        39 => "Controller number invalid",
        40 => "No joystick configured",
        41 => "Invalid axis for electronic gearing, axis can not be slave",
        42 => "Position of slave axis is out of range",
        43 => "Slave axis cannot be commanded directly when electronic gearing is enabled",
        44 => "Calibration of joystick failed",
        45 => "Referencing failed",
        46 => "OPM (Optical Power Meter) missing",
        47 => "OPM (Optical Power Meter) not initialized or cannot be initialized",
        48 => "OPM (Optical Power Meter) Communication Error",
        49 => "Move to limit switch failed",
        50 => "Attempt to reference axis with referencing disabled",
        51 => "Selected axis is controlled by joystick",
        52 => "Controller detected communication error",
        53 => "Command is not allowed while the affected axis is in motion.",
        54 => "Unknown parameter",
        55 => "No commands were recorded with REP",
        56 => "Password invalid",
        57 => "Data recorder table does not exist",
        58 => "Source option does not exist; number too low or too high",
        59 => "Source ID (channel or axis) too low or too high",
        60 => "Protected Param: current Command Level (CCL) too low",
        61 => "Command execution not possible while Autozero is running",
        62 => "Autozero requires at least one linear axis",
        63 => "Initialization still in progress",
        64 => "Parameter is read-only",
        65 => "Parameter not found in non-volatile memory",
        66 => "Voltage out of limits",
        67 => "Not enough memory available for requested wave curve",
        68 => "Not enough memory available for DDL table; DDL can not be started",
        69 => "Time delay larger than DDL table; DDL can not be started",
        70 => "The requested arrays have different lengths; query them separately",
        71 => "Attempt to restart the generator while it is running in single step mode",
        72 => {
            "Motion commands and wave generator activation are not allowed when analog target is active"
        }
        73 => {
            "Motion commands are not allowed when wave generator output is active; use WGO to disable generator output"
        }
        74 => {
            "No sensor channel or no piezo channel connected to selected axis (sensor and piezo matrix)"
        }
        75 => "Generator started (WGO) without having selected a wave table (WSL).",
        76 => "Interface buffer did overrun and command couldn't be received correctly",
        77 => "Data Record Table does not hold enough recorded data",
        78 => "Data Record Table is not configured for recording",
        79 => "Open-loop commands are not allowed when servo is on",
        80 => "Hardware error affecting RAM",
        81 => "Not macro command",
        82 => "Macro counter out of range",
        83 => "Joystick is active",
        84 => "Motor is off",
        85 => "Macro-only command",
        86 => "Invalid joystick axis",
        87 => "Joystick unknown",
        88 => "Move without referenced stage",
        89 => "Command not allowed in current motion mode",
        90 => {
            "No tracing possible while digital IOs are used on this HW revision. Reconnect to switch operation mode."
        }
        91 => "Move not possible, would cause collision",
        92 => "Stage is not capable of following the master. Check the gear ratio(SRA).",
        93 => "This command is not allowed while the affected axis or its master is in motion.",
        94 => "Servo cannot be switched on when open-loop joystick control is enabled.",
        95 => "This parameter cannot be changed in current servo mode.",
        96 => "Unknown stage name",
        97 => "Invalid length of value (too much characters)",
        98 => "AutoZero procedure was not successful",
        99 => "Sensor voltage is off",
        100 => "PI LabVIEW driver reports error. See source control for details.",
        200 => "No stage connected to axis",
        201 => "File with axis parameters not found",
        202 => "Invalid axis parameter file",
        203 => "Backup file with axis parameters not found",
        204 => "PI internal error code 204",
        205 => "SMO with servo on",
        206 => "uudecode: incomplete header",
        207 => "uudecode: nothing to decode",
        208 => "uudecode: illegal UUE format",
        209 => "CRC32 error",
        210 => "Illegal file name (must be 8-0 format)",
        211 => "File not found on controller",
        212 => "Error writing file on controller",
        213 => "VEL command not allowed in DTR Command Mode",
        214 => "Position calculations failed",
        215 => "The connection between controller and stage may be broken",
        216 => {
            "The connected stage has driven into a limit switch, some controllers need CLR to resume operation"
        }
        217 => "Strut test command failed because of an unexpected strut stop",
        218 => "While MOV! is running position can only be estimated!",
        219 => "Position was calculated during MOV motion",
        220 => "FIFO buffer underrun during interpolation",
        221 => "FIFO buffer overflow during interpolation",
        230 => "Invalid handle",
        231 => "No bios found",
        232 => "Save system configuration failed",
        233 => "Load system configuration failed",
        301 => "Send buffer overflow",
        302 => "Voltage out of limits",
        303 => "Open-loop motion attempted when servo ON",
        304 => "Received command is too long",
        305 => "Error while reading/writing EEPROM",
        306 => "Error on I2C bus",
        307 => "Timeout while receiving command",
        308 => "A lengthy operation has not finished in the expected time",
        309 => "Insufficient space to store macro",
        310 => "Configuration data has old version number",
        311 => "Invalid configuration data",
        333 => "Internal hardware error",
        400 => "Wave generator index error",
        401 => "Wave table not defined",
        402 => "Wave type not supported",
        403 => "Wave length exceeds limit",
        404 => "Wave parameter number error",
        405 => "Wave parameter out of range",
        406 => "WGO command bit not supported",
        500 => "The \"red knob\" is still set and disables system",
        501 => "The \"red knob\" was activated and still disables system - reanimation required",
        502 => "Position consistency check failed",
        503 => "Hardware collision sensor(s) are activated",
        504 => "Strut following error occurred, e.g. caused by overload or encoder failure",
        505 => "One sensor signal is not valid",
        506 => {
            "Servo loop was unstable due to wrong parameter setting and switched off to avoid damage."
        }
        507 => "digital connection to external spi slave device is lost",
        508 => "Move attempt not permitted due to customer or limit settings",
        509 => "Emergency stop caused by trigger input",
        530 => "A command refers to a coordinate system that does not exist",
        531 => "A command refers to a coordinate system that has no parent node",
        532 => "Attempt to delete or change a coordinate system that is in use",
        533 => "Definition of a coordinate system is cyclic",
        536 => "Coordinate system cannot be defined as long as Hexapod is in motion",
        537 => "Coordinate system type is not intended for manual enabling",
        539 => "A coordinate system cannot be linked to itself",
        540 => "Coordinate system definition is erroneous or not complete (replace or delete it)",
        542 => "The coordinate systems are not part of the same chain",
        543 => {
            "Unused coordinate system must be deleted before new coordinate system can be stored"
        }
        544 => "With this coordinate system type SPI usage is not supported",
        545 => "Soft limits invalid due to changes in coordinate system",
        546 => "Coordinate system is write protected",
        547 => {
            "Coordinate system cannot be changed because its content is loaded from a configuration file"
        }
        548 => "Coordinate system may not be linked",
        549 => "A KSB-type coordinate system can only be rotated by multiples of 90 degrees",
        551 => "This query is not supported for this coordinate system type",
        552 => "This combination of work and tool coordinate systems does not exist",
        553 => "The combination must consist of one work and one tool coordinate system",
        554 => "This coordinate system type does not exist",
        555 => "BasMac: unknown controller error",
        556 => "No coordinate system of this type is enabled",
        557 => "Name of coordinate system is invalid",
        558 => "File with stored CS systems is missing or erroneous",
        559 => "File with leveling CS is missing or erroneous",
        601 => "Not enough memory",
        602 => "Hardware voltage error",
        603 => "Hardware temperature out of range",
        604 => "Position error of any axis in the system is too high",
        606 => "Maximum value of input signal has been exceeded",
        607 => "Value is not integer",
        608 => "Fast alignment process cannot be paused because it is not running",
        609 => "Fast alignment process cannot be restarted/resumed because it is not paused",
        650 => "Parameter could not be set with SPA - SEP needed?",
        651 => "Phase finding error",
        652 => "Sensor setup error",
        653 => "Sensor communication error",
        654 => "Motor amplifier error",
        655 => "Overcurrent protection triggered by I2T-module",
        656 => "Overcurrent protection triggered by amplifier module",
        657 => "Safety stop triggered",
        658 => "Sensor off?",
        659 => "Parameter could not be set. Conflict with another parameter.",
        700 => "Command not allowed in external mode",
        710 => "External mode communication error",
        715 => "Invalid mode of operation",
        716 => "Firmware stopped by command (#27)",
        717 => "External mode driver missing",
        718 => "Missing or incorrect configuration of external mode",
        719 => "External mode cycletime invalid",
        720 => "Brake is activated",
        725 => "Drive state transition error",
        731 => "Command not allowed while surface detection is running",
        732 => "Last surface detection failed",
        733 => "Fieldbus is active and is blocking GCS control commands",
        1000 => "Too many nested macros",
        1001 => "Macro already defined",
        1002 => "Macro recording not activated",
        1003 => "Invalid parameter for MAC",
        1004 => "Deleting macro failed",
        1005 => {
            "Controller is busy with some lengthy operation (e.g. reference move, fast scan algorithm)"
        }
        1006 => "Invalid identifier (invalid special characters, ...)",
        1007 => "Variable or argument not defined",
        1008 => "Controller is (already) running a macro",
        1009 => {
            "Invalid or missing operator for condition. Check necessary spaces around operator."
        }
        1010 => "No answer was received while executing WAC/MEX/JRC/...",
        1011 => "Command not valid during macro execution",
        1012 => "Error occured during macro execution",
        1013 => "No macro with given name on controller, or macro is empty",
        1015 => {
            "One or more arguments given to function is invalid (empty string, index out of range, ...)"
        }
        1024 => "Motion error: position error too large, servo is switched off automatically",
        1025 => "Maximum motor output reached",
        1028 => "Unknown channel identifier",
        1063 => {
            "User Profile Mode: Command is not allowed, check for required preparatory commands"
        }
        1064 => {
            "User Profile Mode: First target position in User Profile is too far from current position"
        }
        1065 => "Controller is (already) in User Profile Mode",
        1066 => "User Profile Mode: Block or Data Set index out of allowed range",
        1071 => "User Profile Mode: Out of memory",
        1072 => "User Profile Mode: Cluster is not assigned to this axis",
        1073 => "Unknown cluster identifier",
        1090 => "There are too many open tcpip connections",
        2000 => "Controller already has a serial number",
        2100 => "Entered license is invalid",
        4000 => "Sector erase failed",
        4001 => "Flash program failed",
        4002 => "Flash read failed",
        4003 => "HW match code missing/invalid",
        4004 => "FW match code missing/invalid",
        4005 => "HW version missing/invalid",
        4006 => "FW version missing/invalid",
        4007 => "FW update failed",
        4008 => "FW Parameter CRC wrong",
        4009 => "FW CRC wrong",
        5000 => "PicoCompensation scan data is not valid",
        5001 => {
            "PicoCompensation is running, some actions can not be executed during scanning/recording"
        }
        5002 => "Given axis can not be defined as PPC axis",
        5003 => "Defined scan area is larger than the travel range",
        5004 => "Given PicoCompensation type is not defined",
        5005 => "PicoCompensation parameter error",
        5006 => "PicoCompensation table is larger than maximum table length",
        5100 => "Common error in Nexline firmware module",
        5101 => "Output channel for Nexline can not be redefined for other usage",
        5102 => "Memory for Nexline signals is too small",
        5103 => "RNP can not be executed if axis is in closed loop",
        5104 => "relax procedure (RNP) needed",
        5200 => "Axis must be configured for this action",
        5300 => "Frequency analysis failed",
        5301 => "Another frequency analysis is running",
        6000 => "Invalid preset value of absolute sensor",
        6001 => "Error while writing to sensor",
        6002 => "Error while reading from sensor",
        6003 => "Checksum error of absolute sensor",
        6004 => "General error of absolute sensor",
        6005 => "Overflow of absolute sensor position",
        8961 => "Wrong number of arguments",
        8962 => "Unknown command",
        8965 => "User command level too low for command access",
        8970 => "Invalid password",
        16385 => "Wrong data type",
        17154 => "Unknown parameter ID",
        17157 => "User command level too low for parameter access",
        17162 => "Argument value not valid",
        17163 => "Wrong parameter type",
        17192 => "Argument value out of range",
        21255 => "Motion not possible due to active limit switch",
        21258 => "Invalid mode of operation",
        21259 => "Axis not referenced",
        21288 => "Target out of range",
        21308 => "Axis is disabled",
        21767 => "Axis has activated a limit switch",
        21800 => "Motor current is too high, servo is switched off automatically",
        21801 => "Position error too large, servo is switched off automatically",
        24583 => "The max number of storable data recorders has been reached",
        24596 => "The given data recorder has already been registered",
        25354 => "Data recorder is not in configuration mode",
        25355 => "Data recorder is already running",
        25384 => "Specified data recorder trace does not exist",
        25414 => "Not enough recorded data in at least one data recorder trace",
        25415 => "Data recorder traces are not configured",
        33380 => "Controller detected communication error",
        33538 => "Firmware index unknown",
        33796 => "Timeout occurred",
        33803 => "Invalid Socket",
        36867 => "Unit not initialized",
        36871 => "The max number of storable connections has been reached",
        36874 => "Wrong arguments when setting a connection output",
        36875 => "Wrong arguments when setting a connection input",
        36876 => "Wrong unit ID format given",
        36877 => "Wrong device ID",
        36878 => "Wrong function ID",
        36879 => "Wrong proxy ID",
        36904 => "Connection output index is out of range",
        36914 => "Failed to register an interface in the bus gateway",
        36915 => "Failed to register a device in the bus gateway",
        36916 => "Failed to register a proxy in the bus gateway",
        37140 => "The given input port has already been registered in another connection",
        37141 => "The given unit has already been registered to the broker",
        37180 => "The connection has no input",
        37181 => "The connection has no output",
        37183 => "The sought connection is not registered",
        37184 => "The input port is not connected",
        37446 => "Data corrupted",
        37682 => "Firmware update failed",
        37692 => "Unit could not be found",
        37889 => "Unit type not supported",
        37918 => "Not enough memory",
        37919 => "No data available",
        37938 => "Flash read failed",
        37988 => "Fatal system error occurred",
        _ => return None,
    })
}
