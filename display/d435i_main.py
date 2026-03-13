"""
D435i RealSense Main Detector Control Display

Launch with: pydm d435i_main.py -m '{"P":"RS1:"}'
"""

from os import path

from pydm import Display
from pydm.widgets import (
    PyDMEnumComboBox,
    PyDMLabel,
    PyDMLineEdit,
    PyDMPushButton,
    PyDMRelatedDisplayButton,
)
from qtpy.QtCore import Qt
from qtpy.QtWidgets import (
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QVBoxLayout,
    QWidget,
)


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

        # Title
        title = QLabel(f"D435i RealSense — {self.p}")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(title)

        # Top row: Device Info + Acquire
        top = QHBoxLayout()
        top.addWidget(self._device_info_group())
        top.addWidget(self._acquire_group())
        layout.addLayout(top)

        # Middle row: Stream Config + Sensor Controls
        mid = QHBoxLayout()
        mid.addWidget(self._stream_config_group())
        mid.addWidget(self._sensor_controls_group())
        layout.addLayout(mid)

        # Bottom row: Depth Info + IMU
        bot = QHBoxLayout()
        bot.addWidget(self._depth_info_group())
        bot.addWidget(self._imu_group())
        layout.addLayout(bot)

        # Image plugin buttons
        layout.addWidget(self._image_plugins_group())

        # Array info
        layout.addWidget(self._array_info_group())

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
            ("Connected", "IsConnected_RBV"),
        ]:
            w = PyDMLabel(init_channel=self._pv(f"cam1:{suf}"))
            form.addRow(f"{label}:", w)
        return grp

    def _acquire_group(self):
        grp = QGroupBox("Acquire")
        form = QFormLayout()
        grp.setLayout(form)

        # Acquire button
        acq = PyDMEnumComboBox(init_channel=self._pv("cam1:Acquire"))
        form.addRow("Acquire:", acq)

        # Image mode
        im = PyDMEnumComboBox(init_channel=self._pv("cam1:ImageMode"))
        form.addRow("Image Mode:", im)

        # Detector state
        state = PyDMLabel(init_channel=self._pv("cam1:DetectorState_RBV"))
        form.addRow("State:", state)

        # Array counter
        cnt = PyDMLabel(init_channel=self._pv("cam1:ArrayCounter_RBV"))
        form.addRow("Array Counter:", cnt)

        # Status message
        msg = PyDMLabel(init_channel=self._pv("cam1:StatusMessage_RBV"))
        form.addRow("Status:", msg)

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

    def _image_plugins_group(self):
        grp = QGroupBox("Image Viewers")
        h = QHBoxLayout()
        grp.setLayout(h)

        btn_color = PyDMRelatedDisplayButton(filename=path.join(
            path.dirname(path.abspath(__file__)), "d435i_dual_view.py"
        ))
        btn_color.setText("Open Dual Viewer (Color + Depth)")
        btn_color.macros = f'{{"P":"{self.p}"}}'
        h.addWidget(btn_color)

        return grp

    def _array_info_group(self):
        grp = QGroupBox("Array Info")
        h = QHBoxLayout()
        grp.setLayout(h)

        for title, prefix in [("Color (image1)", "image1"), ("Depth (image2)", "image2")]:
            sub = QGroupBox(title)
            form = QFormLayout()
            sub.setLayout(form)
            for label, suf in [
                ("Size X", f"{prefix}:ArraySize0_RBV"),
                ("Size Y", f"{prefix}:ArraySize1_RBV"),
                ("Size Z", f"{prefix}:ArraySize2_RBV"),
                ("Callbacks", f"{prefix}:EnableCallbacks"),
            ]:
                if suf.endswith("EnableCallbacks"):
                    w = PyDMEnumComboBox(init_channel=self._pv(suf))
                else:
                    w = PyDMLabel(init_channel=self._pv(suf))
                form.addRow(f"{label}:", w)
            h.addWidget(sub)

        return grp
