from python_qt_binding import QtCore, QtWidgets

from adapi_debug_tools.api import Adapi


class UpdateAndCancel(QtWidgets.QHBoxLayout):

    def __init__(self, dialog):
        super().__init__()
        self.update = QtWidgets.QPushButton("update")
        self.cancel = QtWidgets.QPushButton("cancel")
        self.update.clicked.connect(dialog.on_update)
        self.cancel.clicked.connect(dialog.on_cancel)
        self.addWidget(self.update)
        self.addWidget(self.cancel)


class PoseCovDialog(QtWidgets.QDialog):

    def __init__(self, adapi: Adapi, parent: QtWidgets.QWidget):
        super().__init__(parent)
        self.adapi = adapi
        self.footer = UpdateAndCancel(self)
        self.editor = QtWidgets.QTextEdit()
        self.editor.setPlainText(adapi.settings.get_yaml("initial-pose"))
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.editor)
        layout.addLayout(self.footer)
        self.setLayout(layout)

    def on_update(self):
        data = self.editor.toPlainText()
        self.adapi.settings.set_yaml("initial-pose", data)
        self.close()

    def on_cancel(self):
        self.close()
