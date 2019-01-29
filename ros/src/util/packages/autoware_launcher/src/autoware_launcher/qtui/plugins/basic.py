from python_qt_binding import QtCore, QtWidgets
from autoware_launcher.core import myutils
from autoware_launcher.qtui import widgets



def plugin_widgets():
    return \
    {
        "text" : AwTextEdit,
        "file" : AwFileSelect,
    }



class AwTextEdit(widgets.AwAbstructFrame):

    def __init__(self, guimgr, node, view):
        super(AwTextEdit, self).__init__(guimgr, node, view)

        super(AwTextEdit, self).setup_widget()
        self.lineedit = QtWidgets.QLineEdit()
        self.lineedit.setText(self.node.get_config(self.view.target))
        self.lineedit.editingFinished.connect(self.edited)
        self.add_widget(self.lineedit)
        if self.view.title:
            self.set_title(self.view.title)
        else:
            self.set_title("Text ({})".format(self.view.target))

    def edited(self):
        print self.__class__.__name__ + ".edited"
        prev_data = self.node.get_config(self.view.target)
        curr_data = self.lineedit.text()
        if prev_data != curr_data:
            self.node.update({"config": {self.view.target: curr_data}})



class AwFileSelectButton(QtWidgets.QPushButton):

    def __init__(self, lineedit, text = "Browse"):
        super(AwFileSelectButton, self).__init__(text)
        self.lineedit = lineedit
        self.clicked.connect(self.browsed)

    def browsed(self):
        filepath, filetype = QtWidgets.QFileDialog.getOpenFileName(self, "Select File", myutils.userhome())
        filepath = myutils.envpath(filepath)
        if filepath:
            self.lineedit.setText(filepath)
            self.lineedit.editingFinished.emit()

class AwFileSelect(AwTextEdit):

    def __init__(self, guimgr, node, view):
        super(AwFileSelect, self).__init__(guimgr, node, view)
        self.add_button(AwFileSelectButton(self.lineedit))