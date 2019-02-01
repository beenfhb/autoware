from python_qt_binding import QtCore, QtWidgets
from autoware_launcher.core import myutils
from autoware_launcher.gui  import widgets



def plugin_widgets():
    return \
    {
        "text"      : AwTextFrame,
        "textlist"  : AwTextListFrame,
        "file"      : AwFileFrame,
        "filelist"  : AwFileListFrame,
        "transform" : AwTransformFrame,
    }

def frame_title(view, text):
    if view.title:
        return view.title
    else:
        target = view.target
        if type(target) is list: target = ", ".join(target)
        return "{} ({})".format(text, target)



class AwTextFrame(widgets.AwAbstructFrame):

    def __init__(self, guimgr, node, view):
        super(AwTextFrame, self).__init__(guimgr, node, view)
        super(AwTextFrame, self).setup_widget()
        self.set_title(frame_title(self.view, "Text"))

        self.field = AwTextField(self.node.get_config(self.view.target))
        self.field.value_updated.connect(self.apply)
        self.add_widget(self.field)

    def apply(self, value):
        self.node.update({"config": {self.view.target: value}})



class AwTextListFrame(widgets.AwAbstructFrame):

    def __init__(self, guimgr, node, view):
        super(AwTextListFrame, self).__init__(guimgr, node, view)
        super(AwTextListFrame, self).setup_widget()
        self.set_title(frame_title(self.view, "TextList"))

        self.field = AwTextListField(self.node.get_config(self.view.target))
        self.field.value_updated.connect(self.apply)
        self.add_widget(self.field)

    def apply(self, value):
        self.node.update({"config": {self.view.target: value}})



class AwFileFrame(AwTextFrame):

    def __init__(self, guimgr, node, view):
        super(AwFileFrame, self).__init__(guimgr, node, view)
        self.add_button(AwFileBrowseButton(self.field))



class AwFileListFrame(AwTextListFrame):

    def __init__(self, guimgr, node, view):
        super(AwFileListFrame, self).__init__(guimgr, node, view)
        self.add_button(AwFileListBrowseButton(self.field))



class AwIntegerFrame(widgets.AwAbstructFrame):

    def __init__(self, guimgr, node, view):
        super(AwIntegerFrame, self).__init__(guimgr, node, view)
        super(AwIntegerFrame, self).setup_widget()
        self.set_title(frame_title(self.view, "Integer"))

        self.field = AwIntegerField(self.node.get_config(self.view.target))
        self.field.value_updated.connect(self.apply)
        self.add_widget(self.field)

    def apply(self, value):
        self.node.update({"config": {self.view.target: value}})



class AwTransformFrame(widgets.AwAbstructFrame):

    def __init__(self, guimgr, node, view):
        super(AwTransformFrame, self).__init__(guimgr, node, view)
        super(AwTransformFrame, self).setup_widget()
        self.set_title(frame_title(self.view, "Transform"))

        widget = QtWidgets.QWidget()
        widget.setLayout(QtWidgets.QHBoxLayout())
        for idx, txt in enumerate(["Tx", "Ty", "Tz", "Rx", "Ry", "Rz"]):
            field = AwTextField(self.node.get_config(self.view.target[idx]))
            field.target = self.view.target[idx]
            field.value_updated.connect(self.apply)
            widget.layout().addWidget(QtWidgets.QLabel(txt + ":"))
            widget.layout().addWidget(field)
        self.add_widget(widget)

    def apply(self, value):
        field = self.sender()
        self.node.update({"config": {field.target: value}})






class AwTextField(QtWidgets.QLineEdit):

    value_updated = QtCore.Signal(str)

    def __init__(self, value):
        super(AwTextField, self).__init__()
        self.__value = value
        self.setText(str(self.__value))

    def update_value(self, value):
        if self.__value != value:
            self.__value = value
            self.value_updated.emit(value)

    def focusOutEvent(self, event):
        self.update_value(self.text())
        super(AwTextField, self).focusOutEvent(event)



class AwTextListField(QtWidgets.QPlainTextEdit):

    value_updated = QtCore.Signal(list)

    def __init__(self, value):
        super(AwTextListField, self).__init__()
        self.__value = [v for v in value if v]
        self.setPlainText("\n".join(self.__value))

    def update_value(self, value):
        value = [v for v in value if v]
        if self.__value != value:
            self.__value = value
            self.value_updated.emit(value)

    def focusOutEvent(self, event):
        self.update_value(self.toPlainText().split("\n"))
        super(AwTextListField, self).focusOutEvent(event)



class AwIntegerField(QtWidgets.QLineEdit):

    value_updated = QtCore.Signal(str)

    def __init__(self, value):
        super(AwIntegerField, self).__init__()
        self.__value = value
        self.setText(str(self.__value))

    def update_value(self, value):
        if self.__value != value:
            self.__value = value
            self.value_updated.emit(value)

    def focusOutEvent(self, event):
        self.update_value(int(self.text()))
        super(AwIntegerField, self).focusOutEvent(event)






class AwFileBrowseButton(QtWidgets.QPushButton):

    def __init__(self, field, text = "Browse"):
        super(AwFileBrowseButton, self).__init__(text)
        self.__field = field
        self.clicked.connect(self.browsed)

    def browsed(self):
        filepath, filetype = QtWidgets.QFileDialog.getOpenFileName(self, "Select File", myutils.userhome())
        if filepath:
            filepath = myutils.envpath(filepath)
            self.__field.update_value(filepath)



class AwFileListBrowseButton(QtWidgets.QPushButton):

    def __init__(self, field, text = "Browse"):
        super(AwFileListBrowseButton, self).__init__(text)
        self.__field = field
        self.clicked.connect(self.browsed)

    def browsed(self):
        filepaths, filetype = QtWidgets.QFileDialog.getOpenFileNames(self, "Select Files", myutils.userhome())
        if filepaths:
            filepaths = [myutils.envpath(filepath) for filepath in filepaths]
            self.__field.update_value(filepaths)
