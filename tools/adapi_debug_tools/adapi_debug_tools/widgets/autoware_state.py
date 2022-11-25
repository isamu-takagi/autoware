from python_qt_binding import QtCore, QtWidgets

from adapi_debug_tools.api import Adapi
from adapi_debug_tools.widgets.interface import InterfaceWidgets
from adapi_debug_tools.widgets.operation_mode import OperationModeWidgets


class AutowareStateWidget(QtWidgets.QWidget):

    def __init__(self, adapi: Adapi):
        super().__init__()
        self.widgets = [
            InterfaceWidgets(adapi),
            OperationModeWidgets(adapi),
        ]
        layout = QtWidgets.QFormLayout()
        for widgets in self.widgets:
            for row in widgets.layout:
                layout.addRow(*row)
        self.setLayout(layout)
