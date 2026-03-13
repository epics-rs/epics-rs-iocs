"""
D435i Dual Image Viewer — Color + Depth side by side

Launch with: pydm d435i_dual_view.py -m '{"P":"RS1:"}'
"""

from pydm import Display
from pydm.widgets import PyDMEnumComboBox, PyDMImageView, PyDMLabel
from qtpy.QtCore import Qt
from qtpy.QtWidgets import (
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
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

    def _setup_ui(self):
        layout = QVBoxLayout()
        self.setLayout(layout)

        # Title
        title = QLabel(f"D435i Dual View — {self.p}")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        layout.addWidget(title)

        # Image viewers side by side
        images = QHBoxLayout()

        # Color image
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

        # Depth image
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

        # Bottom toolbar
        toolbar = QHBoxLayout()

        # Depth colormap selector
        toolbar.addWidget(QLabel("Depth Colormap:"))
        self.cmap_combo = QComboBox()
        self.cmap_combo.addItems([
            "inferno", "viridis", "plasma", "magma", "hot", "jet", "gray",
        ])
        self.cmap_combo.currentTextChanged.connect(self._set_depth_colormap)
        toolbar.addWidget(self.cmap_combo)

        toolbar.addStretch()

        # Acquire control
        toolbar.addWidget(QLabel("Acquire:"))
        acq = PyDMEnumComboBox(init_channel=self._pv("cam1:Acquire"))
        toolbar.addWidget(acq)

        # Detector state
        toolbar.addWidget(QLabel("State:"))
        state = PyDMLabel(init_channel=self._pv("cam1:DetectorState_RBV"))
        toolbar.addWidget(state)

        layout.addLayout(toolbar)

        # Apply default colormap
        self._set_depth_colormap("inferno")

    def _set_depth_colormap(self, name):
        try:
            self.depth_image.colorMap = name
        except Exception:
            # Fallback for PyDM versions with different colormap API
            try:
                import numpy as np
                from matplotlib import colormaps

                cmap = colormaps[name]
                lut = (cmap(np.linspace(0, 1, 256)) * 255).astype(np.uint8)
                self.depth_image.colorMapLUT = lut
            except Exception:
                pass
