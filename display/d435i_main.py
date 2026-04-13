"""
D435i RealSense Main Detector Control Display

Launch with: pydm d435i_main.py -m '{"P":"RS1:"}'
"""

from os import path

from pydm import Display
from pydm.widgets import (
    PyDMByteIndicator,
    PyDMEnumComboBox,
    PyDMLabel,
    PyDMLineEdit,
    PyDMPushButton,
    PyDMRelatedDisplayButton,
)
from qtpy.QtCore import Qt
from qtpy.QtWidgets import (
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)


# Plugin records loaded by the three ioc/*Plugins.cmd scripts.
# Each entry: (record_prefix, friendly_name). record_prefix is appended to P.
# These match the R=... suffixes used in the plugin cmd files.
PLUGIN_MANIFEST = {
    "Color (RS1)": [
        ("image1:", "StdArrays"),
        ("ROI1:", "ROI"),
        ("ROIStat1:", "ROIStat"),
        ("Stats1:", "Stats"),
        ("Trans1:", "Trans"),
        ("Proc1:", "Process"),
        ("Over1:", "Overlay"),
        ("CB1:", "CircularBuff"),
        ("Attr1:", "Attribute"),
        ("FFT1:", "FFT"),
        ("Codec1:", "Codec1"),
        ("Codec2:", "Codec2"),
        ("CC1:", "ColorConvert1"),
        ("CC2:", "ColorConvert2"),
        ("BadPix1:", "BadPixel"),
        ("TIFF1:", "TIFF (save)"),
        ("JPEG1:", "JPEG (save)"),
        ("HDF1:", "HDF5 (save)"),
        ("netCDF1:", "netCDF (save)"),
        ("Nexus1:", "Nexus (save)"),
        ("Pva1:", "PVAccess"),
    ],
    "Depth (RS1_DEPTH)": [
        ("image2:", "StdArrays"),
        ("depthROI1:", "ROI"),
        ("depthROIStat1:", "ROIStat"),
        ("depthStats1:", "Stats"),
        ("depthTIFF1:", "TIFF (save)"),
        ("depthHDF1:", "HDF5 (save)"),
    ],
    "Pointcloud (RS1_PC)": [
        ("image3:", "StdArrays"),
        ("pcHDF1:", "HDF5 (save)"),
    ],
}


# Save plugins that get "Capture" quick-trigger buttons
SAVE_PLUGINS = [
    ("TIFF1:", "Color TIFF"),
    ("HDF1:", "Color HDF5"),
    ("depthTIFF1:", "Depth TIFF"),
    ("depthHDF1:", "Depth HDF5"),
    ("pcHDF1:", "PC HDF5"),
]


class D435iMainDisplay(Display):
    def __init__(self, parent=None, args=None, macros=None):
        super().__init__(parent=parent, args=args, macros=macros)
        self.setWindowTitle("D435i RealSense Control")
        if macros is None:
            macros = {}
        self.p = macros.get("P", "RS1:")
        self._setup_ui()

    def ui_filename(self):
        return None

    def _pv(self, suffix):
        return f"ca://{self.p}{suffix}"

    def _setup_ui(self):
        layout = QVBoxLayout()
        self.setLayout(layout)

        title = QLabel(f"D435i RealSense — {self.p}")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(title)

        tabs = QTabWidget()
        layout.addWidget(tabs, stretch=1)

        # Tab 1: Detector — Device Info, Acquire, Stream, Sensor
        det_tab = QWidget()
        det_layout = QVBoxLayout()
        det_tab.setLayout(det_layout)
        top = QHBoxLayout()
        top.addWidget(self._device_info_group())
        top.addWidget(self._acquire_group(), stretch=1)
        det_layout.addLayout(top)
        mid = QHBoxLayout()
        mid.addWidget(self._stream_config_group())
        mid.addWidget(self._sensor_controls_group())
        det_layout.addLayout(mid)
        det_layout.addStretch()
        tabs.addTab(det_tab, "Detector")

        # Tab 2: Processing — Depth Info, IMU, Diagnostics, Post-Processing, Alignment
        proc_tab = QWidget()
        proc_layout = QVBoxLayout()
        proc_tab.setLayout(proc_layout)
        bot = QHBoxLayout()
        bot.addWidget(self._depth_info_group())
        bot.addWidget(self._imu_group())
        bot.addWidget(self._diagnostics_group())
        proc_layout.addLayout(bot)
        proc_row = QHBoxLayout()
        proc_row.addWidget(self._postprocessing_group())
        proc_row.addWidget(self._alignment_group())
        proc_layout.addLayout(proc_row)
        proc_layout.addStretch()
        tabs.addTab(proc_tab, "Processing")

        # Tab 3: Plugins — Enable/Status, File Capture
        plug_tab = QWidget()
        plug_layout = QVBoxLayout()
        plug_tab.setLayout(plug_layout)
        plug_layout.addWidget(self._plugins_group(), stretch=1)
        plug_layout.addWidget(self._capture_group())
        tabs.addTab(plug_tab, "Plugins")

        # Image viewer launcher (always visible)
        layout.addWidget(self._image_plugins_group())

    # ------------------------------------------------------------------ groups
    def _device_info_group(self):
        grp = QGroupBox("Device Info")
        form = QFormLayout()
        grp.setLayout(form)
        for label, suf in [
            ("Model", "Model_RBV"),
            ("Serial", "SerialNumber_RBV"),
            ("Firmware", "FirmwareVersion_RBV"),
            ("SDK Version", "SDKVersion_RBV"),
            ("Connected", "RSConnected_RBV"),
        ]:
            w = PyDMLabel(init_channel=self._pv(f"cam1:{suf}"))
            form.addRow(f"{label}:", w)
        return grp

    def _acquire_group(self):
        grp = QGroupBox("Acquire / Trigger")
        outer = QVBoxLayout()
        grp.setLayout(outer)

        # Trigger buttons row
        btn_row = QHBoxLayout()

        start = PyDMPushButton(
            label="Start",
            init_channel=self._pv("cam1:Acquire"),
            pressValue=1,
        )
        start.setStyleSheet("padding: 6px 16px;")
        btn_row.addWidget(start)

        stop = PyDMPushButton(
            label="Stop",
            init_channel=self._pv("cam1:Acquire"),
            pressValue=0,
        )
        stop.setStyleSheet("padding: 6px 16px;")
        btn_row.addWidget(stop)

        # Single-frame trigger: sequential write of ImageMode=Single, NumImages=1, Acquire=1
        single = QPushButton("Trigger Single")
        single.setStyleSheet("padding: 6px 16px;")
        single.clicked.connect(self._trigger_single)
        btn_row.addWidget(single)

        # N-frame trigger: uses current NumImages, sets ImageMode=Multiple, Acquire=1
        burst = QPushButton("Trigger Burst")
        burst.setStyleSheet("padding: 6px 16px;")
        burst.clicked.connect(self._trigger_burst)
        btn_row.addWidget(burst)

        btn_row.addStretch()
        outer.addLayout(btn_row)

        # Mode / counters form
        form = QFormLayout()
        outer.addLayout(form)

        form.addRow("Image Mode:", PyDMEnumComboBox(init_channel=self._pv("cam1:ImageMode")))
        form.addRow("Num Images:", PyDMLineEdit(init_channel=self._pv("cam1:NumImages")))
        form.addRow("State:", PyDMLabel(init_channel=self._pv("cam1:DetectorState_RBV")))
        form.addRow("Array Counter:", PyDMLabel(init_channel=self._pv("cam1:ArrayCounter_RBV")))
        form.addRow("Images Counter:", PyDMLabel(init_channel=self._pv("cam1:NumImagesCounter_RBV")))
        form.addRow("Status:", PyDMLabel(init_channel=self._pv("cam1:StatusMessage_RBV")))

        return grp

    def _stream_config_group(self):
        grp = QGroupBox("Stream Config")
        form = QFormLayout()
        grp.setLayout(form)

        mode = PyDMEnumComboBox(init_channel=self._pv("cam1:RSStreamMode"))
        form.addRow("Stream Mode:", mode)

        for label, suf in [
            ("Width", "cam1:RSResX_RBV"),
            ("Height", "cam1:RSResY_RBV"),
            ("Frame Rate", "cam1:RSFrameRate_RBV"),
        ]:
            w = PyDMLabel(init_channel=self._pv(suf))
            form.addRow(f"{label}:", w)

        return grp

    def _sensor_controls_group(self):
        grp = QGroupBox("Sensor Controls")
        form = QFormLayout()
        grp.setLayout(form)

        for label, suf in [
            ("Exposure", "cam1:AcquireTime"),
            ("Gain", "cam1:Gain"),
        ]:
            row = QHBoxLayout()
            sp = PyDMLineEdit(init_channel=self._pv(suf))
            rb = PyDMLabel(init_channel=self._pv(f"{suf}_RBV"))
            row.addWidget(sp)
            row.addWidget(rb)
            w = QWidget()
            w.setLayout(row)
            form.addRow(f"{label}:", w)

        ae = PyDMEnumComboBox(init_channel=self._pv("cam1:RSAutoExposure"))
        form.addRow("Auto Exposure:", ae)

        lp = PyDMLineEdit(init_channel=self._pv("cam1:RSLaserPower"))
        form.addRow("Laser Power:", lp)

        em = PyDMEnumComboBox(init_channel=self._pv("cam1:RSEmitterEnabled"))
        form.addRow("Emitter:", em)

        return grp

    def _depth_info_group(self):
        grp = QGroupBox("Depth Info")
        form = QFormLayout()
        grp.setLayout(form)
        du = PyDMLabel(init_channel=self._pv("cam1:RSDepthUnits_RBV"))
        form.addRow("Depth Units:", du)
        return grp

    def _imu_group(self):
        grp = QGroupBox("IMU Readback")
        form = QFormLayout()
        grp.setLayout(form)
        for axis in ("X", "Y", "Z"):
            a = PyDMLabel(init_channel=self._pv(f"cam1:RSAccel{axis}_RBV"))
            form.addRow(f"Accel {axis}:", a)
        for axis in ("X", "Y", "Z"):
            g = PyDMLabel(init_channel=self._pv(f"cam1:RSGyro{axis}_RBV"))
            form.addRow(f"Gyro {axis}:", g)
        return grp

    def _diagnostics_group(self):
        grp = QGroupBox("Diagnostics")
        form = QFormLayout()
        grp.setLayout(form)
        for label, suf in [
            ("Frames Dropped", "cam1:RSFramesDropped_RBV"),
            ("Error Count", "cam1:RSErrorCount_RBV"),
            ("Last Error", "cam1:RSLastError_RBV"),
            ("Connected", "cam1:RSConnected_RBV"),
        ]:
            w = PyDMLabel(init_channel=self._pv(suf))
            form.addRow(f"{label}:", w)
        return grp

    def _postprocessing_group(self):
        grp = QGroupBox("Depth Post-Processing")
        form = QFormLayout()
        grp.setLayout(form)

        form.addRow("Decimation:", PyDMEnumComboBox(init_channel=self._pv("cam1:RSDecimationEnable")))
        form.addRow("  Magnitude:", PyDMLineEdit(init_channel=self._pv("cam1:RSDecimationMag")))

        form.addRow("Spatial:", PyDMEnumComboBox(init_channel=self._pv("cam1:RSSpatialEnable")))
        form.addRow("  Alpha:", PyDMLineEdit(init_channel=self._pv("cam1:RSSpatialAlpha")))
        form.addRow("  Delta:", PyDMLineEdit(init_channel=self._pv("cam1:RSSpatialDelta")))

        form.addRow("Temporal:", PyDMEnumComboBox(init_channel=self._pv("cam1:RSTemporalEnable")))
        form.addRow("  Alpha:", PyDMLineEdit(init_channel=self._pv("cam1:RSTemporalAlpha")))
        form.addRow("  Delta:", PyDMLineEdit(init_channel=self._pv("cam1:RSTemporalDelta")))

        form.addRow("Hole Fill:", PyDMEnumComboBox(init_channel=self._pv("cam1:RSHoleFillEnable")))
        form.addRow("  Mode:", PyDMEnumComboBox(init_channel=self._pv("cam1:RSHoleFillMode")))

        return grp

    def _alignment_group(self):
        grp = QGroupBox("Alignment & Pointcloud")
        form = QFormLayout()
        grp.setLayout(form)
        form.addRow("Align D->C:", PyDMEnumComboBox(init_channel=self._pv("cam1:RSAlignEnable")))
        form.addRow("Pointcloud:", PyDMEnumComboBox(init_channel=self._pv("cam1:RSPointcloudEnable")))
        return grp

    def _plugins_group(self):
        """Tabbed per-port plugin enable panel.

        Each row: [Enable combo] | status LED | plugin name | queue use | array rate.
        """
        grp = QGroupBox("Plugins — Enable & Status")
        outer = QVBoxLayout()
        grp.setLayout(outer)

        tabs = QTabWidget()
        outer.addWidget(tabs)

        for port_label, entries in PLUGIN_MANIFEST.items():
            tab = QWidget()
            grid = QGridLayout()

            # Header row
            for col, text in enumerate(["On", "State", "Plugin", "Queue", "Dropped"]):
                h = QLabel(text)
                h.setStyleSheet("font-weight: bold;")
                grid.addWidget(h, 0, col)

            for row, (r_prefix, name) in enumerate(entries, start=1):
                enable = PyDMEnumComboBox(
                    init_channel=self._pv(f"{r_prefix}EnableCallbacks")
                )
                enable.setMinimumWidth(70)
                grid.addWidget(enable, row, 0)

                led = PyDMByteIndicator(
                    init_channel=self._pv(f"{r_prefix}EnableCallbacks_RBV")
                )
                led.showLabels = False
                led.numBits = 1
                grid.addWidget(led, row, 1)

                grid.addWidget(QLabel(name), row, 2)

                queue = PyDMLabel(init_channel=self._pv(f"{r_prefix}QueueUse_RBV"))
                grid.addWidget(queue, row, 3)

                dropped = PyDMLabel(init_channel=self._pv(f"{r_prefix}DroppedArrays_RBV"))
                grid.addWidget(dropped, row, 4)

            grid.setColumnStretch(2, 1)

            wrapper = QVBoxLayout()
            wrapper.addLayout(grid)
            wrapper.addStretch()
            tab.setLayout(wrapper)
            tabs.addTab(tab, port_label)

        return grp

    def _capture_group(self):
        """Quick file-capture triggers for save plugins.

        One-click = set NumCapture=1, AutoSave=1, Capture=1 on the plugin.
        """
        grp = QGroupBox("File Capture (one shot)")
        h = QHBoxLayout()
        grp.setLayout(h)

        h.addWidget(QLabel("Snapshot:"))
        for prefix, label in SAVE_PLUGINS:
            btn = QPushButton(label)
            btn.setStyleSheet("padding: 4px 10px;")
            btn.clicked.connect(lambda _=False, p=prefix: self._trigger_capture(p))
            h.addWidget(btn)
        h.addStretch()
        return grp

    def _image_plugins_group(self):
        grp = QGroupBox("Image Viewers")
        h = QHBoxLayout()
        grp.setLayout(h)

        btn_color = PyDMRelatedDisplayButton(
            filename=path.join(
                path.dirname(path.abspath(__file__)), "d435i_dual_view.py"
            )
        )
        btn_color.setText("Open Dual Viewer (Color + Depth)")
        btn_color.macros = f'{{"P":"{self.p}"}}'
        h.addWidget(btn_color)
        return grp

    # ------------------------------------------------------------------ action helpers
    def _caput(self, suffix, value):
        """Synchronous CA put using pyepics (bundled with pydm)."""
        try:
            from epics import caput
        except ImportError as e:
            print(f"pyepics not available, cannot write {suffix}: {e}")
            return
        try:
            caput(f"{self.p}{suffix}", value, wait=False)
        except Exception as e:
            print(f"caput failed for {self.p}{suffix}: {e}")

    def _trigger_single(self):
        # ImageMode: 0=Single, 1=Multiple, 2=Continuous
        self._caput("cam1:ImageMode", 0)
        self._caput("cam1:NumImages", 1)
        self._caput("cam1:Acquire", 1)

    def _trigger_burst(self):
        self._caput("cam1:ImageMode", 1)  # Multiple (uses NumImages as-is)
        self._caput("cam1:Acquire", 1)

    def _trigger_capture(self, plugin_prefix):
        self._caput(f"{plugin_prefix}EnableCallbacks", 1)
        self._caput(f"{plugin_prefix}AutoSave", 1)
        self._caput(f"{plugin_prefix}NumCapture", 1)
        self._caput(f"{plugin_prefix}Capture", 1)
