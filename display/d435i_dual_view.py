"""
D435i Dual Image Viewer — Color + Depth side by side

Launch with: pydm d435i_dual_view.py -m '{"P":"RS1:"}'
"""

from pydm import Display
from pydm.widgets import PyDMEnumComboBox, PyDMImageView, PyDMLabel, PyDMPushButton
from qtpy.QtCore import Qt
from qtpy.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
)


class D435iDualViewDisplay(Display):
    def __init__(self, parent=None, args=None, macros=None):
        super().__init__(parent=parent, args=args, macros=macros)
        self.setWindowTitle("D435i Dual View")
        if macros is None:
            macros = {}
        self.p = macros.get("P", "RS1:")
        self._setup_ui()

    def ui_filename(self):
        return None

    def _pv(self, suffix):
        return f"ca://{self.p}{suffix}"

    def _caput(self, suffix, value):
        try:
            from epics import caput
        except ImportError:
            return
        try:
            caput(f"{self.p}{suffix}", value, wait=False)
        except Exception as e:
            print(f"caput failed for {self.p}{suffix}: {e}")

    def _setup_ui(self):
        layout = QVBoxLayout()
        self.setLayout(layout)

        title = QLabel(f"D435i Dual View — {self.p}")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(title)

        # Required plugin enables (image1/image2 StdArrays must be on to see frames)
        plugins = QHBoxLayout()
        plugins.addWidget(QLabel("Viewer plugins:"))
        for prefix, label in [("image1:", "image1 (color)"), ("image2:", "image2 (depth)")]:
            cb = QCheckBox(label)
            cb.setChecked(True)
            cb.stateChanged.connect(
                lambda state, p=prefix: self._caput(
                    f"{p}EnableCallbacks", 1 if state else 0
                )
            )
            plugins.addWidget(cb)
        plugins.addStretch()
        layout.addLayout(plugins)

        # Image viewers side by side
        images = QHBoxLayout()

        color_box = QGroupBox("Color (RGB)")
        color_layout = QVBoxLayout()
        color_box.setLayout(color_layout)
        self.color_image = PyDMImageView(
            image_channel=self._pv("image1:ArrayData"),
            width_channel=self._pv("image1:ArraySize0_RBV"),
        )
        self.color_image.setMinimumSize(320, 240)
        self.color_image.readingOrder = PyDMImageView.ReadingOrder.Clike
        color_layout.addWidget(self.color_image)
        images.addWidget(color_box)

        depth_box = QGroupBox("Depth (Z16)")
        depth_layout = QVBoxLayout()
        depth_box.setLayout(depth_layout)
        self.depth_image = PyDMImageView(
            image_channel=self._pv("image2:ArrayData"),
            width_channel=self._pv("image2:ArraySize0_RBV"),
        )
        self.depth_image.setMinimumSize(320, 240)
        self.depth_image.readingOrder = PyDMImageView.ReadingOrder.Clike
        depth_layout.addWidget(self.depth_image)
        images.addWidget(depth_box)

        layout.addLayout(images, stretch=1)

        # Bottom toolbar: colormap + trigger buttons + state
        toolbar = QHBoxLayout()

        toolbar.addWidget(QLabel("Depth Colormap:"))
        self.cmap_combo = QComboBox()
        self.cmap_combo.addItems([
            "inferno", "viridis", "plasma", "magma", "hot", "jet", "gray",
        ])
        self.cmap_combo.currentTextChanged.connect(self._set_depth_colormap)
        toolbar.addWidget(self.cmap_combo)

        toolbar.addStretch()

        toolbar.addWidget(QLabel("Mode:"))
        toolbar.addWidget(PyDMEnumComboBox(init_channel=self._pv("cam1:ImageMode")))

        start = PyDMPushButton(label="Start", init_channel=self._pv("cam1:Acquire"), pressValue=1)
        stop = PyDMPushButton(label="Stop", init_channel=self._pv("cam1:Acquire"), pressValue=0)
        for b in (start, stop):
            b.setStyleSheet("padding: 4px 12px;")
            toolbar.addWidget(b)

        single = QPushButton("Single")
        single.setStyleSheet("padding: 4px 12px;")
        single.clicked.connect(self._trigger_single)
        toolbar.addWidget(single)

        toolbar.addWidget(QLabel("State:"))
        toolbar.addWidget(PyDMLabel(init_channel=self._pv("cam1:DetectorState_RBV")))

        layout.addLayout(toolbar)

        self._set_depth_colormap("inferno")

    def _trigger_single(self):
        self._caput("cam1:ImageMode", 0)
        self._caput("cam1:NumImages", 1)
        self._caput("cam1:Acquire", 1)

    def _set_depth_colormap(self, name):
        try:
            self.depth_image.colorMap = name
        except Exception:
            try:
                import numpy as np
                from matplotlib import colormaps

                cmap = colormaps[name]
                lut = (cmap(np.linspace(0, 1, 256)) * 255).astype(np.uint8)
                self.depth_image.colorMapLUT = lut
            except Exception:
                pass
