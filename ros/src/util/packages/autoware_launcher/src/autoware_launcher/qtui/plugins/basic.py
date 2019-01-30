from python_qt_binding import QtCore, QtWidgets
from autoware_launcher.core import myutils
from autoware_launcher.qtui import widgets



def plugin_widgets():
    return \
    {
        "text"     : AwTextFrame,
        "textlist" : AwTextListFrame,
        "file"     : AwFileFrame,
        "filelist" : AwFileListFrame,
    }

def frame_title(view, text):
    if view.title:
        return view.title
    else:
        return "{} ({})".format(text, view.target)



class AwTextFrame(widgets.AwAbstructFrame):

    def __init__(self, guimgr, node, view):
        super(AwTextFrame, self).__init__(guimgr, node, view)
        super(AwTextFrame, self).setup_widget()
        self.set_title(frame_title(self.view, "Text"))

        self.input = AwTextInput(self.node.get_config(self.view.target))
        self.input.value_updated.connect(self.apply)
        self.add_widget(self.input)

    def apply(self, value):
        self.node.update({"config": {self.view.target: value}})



class AwTextListFrame(widgets.AwAbstructFrame):

    def __init__(self, guimgr, node, view):
        super(AwTextListFrame, self).__init__(guimgr, node, view)
        super(AwTextListFrame, self).setup_widget()
        self.set_title(frame_title(self.view, "TextList"))

        self.input = AwTextListInput(self.node.get_config(self.view.target))
        self.input.value_updated.connect(self.apply)
        self.add_widget(self.input)

    def apply(self, value):
        self.node.update({"config": {self.view.target: value}})



class AwFileFrame(AwTextFrame):

    def __init__(self, guimgr, node, view):
        super(AwFileFrame, self).__init__(guimgr, node, view)
        self.add_button(AwFileBrowseButton(self.input))



class AwFileListFrame(AwTextListFrame):

    def __init__(self, guimgr, node, view):
        super(AwFileListFrame, self).__init__(guimgr, node, view)
        self.add_button(AwFileListBrowseButton(self.input))



class AwIntegerFrame(widgets.AwAbstructFrame):

    def __init__(self, guimgr, node, view):
        super(AwIntegerFrame, self).__init__(guimgr, node, view)
        super(AwIntegerFrame, self).setup_widget()
        self.set_title(frame_title(self.view, "Integer"))

        self.input = AwIntegerInput(self.node.get_config(self.view.target))
        self.input.value_updated.connect(self.apply)
        self.add_widget(self.input)

    def apply(self, value):
        self.node.update({"config": {self.view.target: value}})


"""
class AwTransformFrame(widgets.AwAbstructFrame):

    def __init__(self, guimgr, node, view):
        super(AwTransformFrame, self).__init__(guimgr, node, view)
        super(AwTransformFrame, self).setup_widget()
        self.set_title(frame_title(self.view, "Transform"))

        widget = QtWidgets.QWidget()
        widget.setLayout(QtWidgets.QHBoxLayout())

        self.inputs = []

        mapper = QtCore.QSignalMapper(widget)
        for idx, txt in enumerate(["Tx", "Ty", "Tz", "Rx", "Ry", "Rz"]):
            field = QtWidgets.QLineEdit()
            field.setText(self.node.get_config("args." + self.opts["defs"][idx]["name"]))
            field.editingFinished.connect(mapper.map)
            mapper.setMapping(field, idx)
            widget.layout().addWidget(QtWidgets.QLabel(txt + ":"))
            widget.layout().addWidget(field)
            self.fields.append(field)

        mapper.mapped.connect(self.edited)
        self.add_widget(widget)

    def edited(self, idx):
        cfgkey = "args." + self.opts["defs"][idx]["name"]
        self.node.update({"config": {cfgkey: self.fields[idx].text()}})

    @staticmethod
    def tostring(node, opts):
        result = opts["title"] + ": "
        for idx, txt in enumerate(["Tx", "Ty", "Tz", "Rx", "Ry", "Rz"]):
            result += txt + "=" + node.get_config("args." + opts["defs"][idx]["name"]) + ", "
        return result
"""





class AwTextInput(QtWidgets.QLineEdit):

    value_updated = QtCore.Signal(str)

    def __init__(self, value):
        super(AwTextInput, self).__init__()
        self.__value = value
        self.setText(str(self.__value))

    def update_value(self, value):
        if self.__value != value:
            self.__value = value
            self.value_updated.emit(value)

    def focusOutEvent(self, event):
        self.update_value(self.text())
        super(AwTextInput, self).focusOutEvent(event)



class AwTextListInput(QtWidgets.QPlainTextEdit):

    value_updated = QtCore.Signal(list)

    def __init__(self, value):
        super(AwTextListInput, self).__init__()
        self.__value = [v for v in value if v]
        self.setPlainText("\n".join(self.__value))

    def update_value(self, value):
        value = [v for v in value if v]
        if self.__value != value:
            self.__value = value
            self.value_updated.emit(value)

    def focusOutEvent(self, event):
        self.update_value(self.toPlainText().split("\n"))
        super(AwTextListInput, self).focusOutEvent(event)



class AwIntegerInput(QtWidgets.QLineEdit):

    value_updated = QtCore.Signal(str)

    def __init__(self, value):
        super(AwIntegerInput, self).__init__()
        self.__value = value
        self.setText(str(self.__value))

    def update_value(self, value):
        if self.__value != value:
            self.__value = value
            self.value_updated.emit(value)

    def focusOutEvent(self, event):
        self.update_value(int(self.text()))
        super(AwIntegerInput, self).focusOutEvent(event)






class AwFileBrowseButton(QtWidgets.QPushButton):

    def __init__(self, input, text = "Browse"):
        super(AwFileBrowseButton, self).__init__(text)
        self.__input = input
        self.clicked.connect(self.browsed)

    def browsed(self):
        filepath, filetype = QtWidgets.QFileDialog.getOpenFileName(self, "Select File", myutils.userhome())
        if filepath:
            filepath = myutils.envpath(filepath)
            self.__input.update_value(filepath)



class AwFileListBrowseButton(QtWidgets.QPushButton):

    def __init__(self, input, text = "Browse"):
        super(AwFileListBrowseButton, self).__init__(text)
        self.__input = input
        self.clicked.connect(self.browsed)

    def browsed(self):
        filepaths, filetype = QtWidgets.QFileDialog.getOpenFileNames(self, "Select Files", myutils.userhome())
        if filepaths:
            filepaths = [myutils.envpath(filepath) for filepath in filepaths]
            self.__input.update_value(filepaths)
