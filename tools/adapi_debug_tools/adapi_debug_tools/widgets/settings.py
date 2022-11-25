from python_qt_binding import QtCore, QtWidgets


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

    def __init__(self, parent):
        super().__init__(parent)
        self.textarea = QtWidgets.QTextEdit()
        self.footer = UpdateAndCancel(self)
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.textarea)
        layout.addLayout(self.footer)
        self.setLayout(layout)

    def on_update(self):
        self.close()

    def on_cancel(self):
        self.close()
