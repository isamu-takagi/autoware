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


class PoseInputLayout(QtWidgets.QGridLayout):

    def __init__(self):
        super().__init__()
        self.fields = ["x", "y", "z", "roll", "pitch", "yaw"]
        self.inputs = {field: QtWidgets.QLineEdit() for field in self.fields}

        for index, field in enumerate(self.fields):
            row = int((index * 2) / 6)
            col = int((index * 2) % 6)
            self.addWidget(QtWidgets.QLabel(field), row, col)
            self.addWidget(self.inputs[field], row, col + 1)



class PoseCovDialog(QtWidgets.QDialog):

    def __init__(self, adapi: Adapi, parent: QtWidgets.QWidget):
        super().__init__(parent)
        self.adapi = adapi
        self.footer = UpdateAndCancel(self)
        self.editor = QtWidgets.QTextEdit()
        self.editor.setPlainText(adapi.settings.get_yaml("initial-pose"))
        self.pose = PoseInputLayout()
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.editor)
        layout.addLayout(self.pose)
        layout.addLayout(self.footer)
        self.setLayout(layout)

    def on_update(self):
        data = self.editor.toPlainText()
        self.adapi.settings.set_yaml("initial-pose", data)
        self.close()

    def on_cancel(self):
        self.close()
