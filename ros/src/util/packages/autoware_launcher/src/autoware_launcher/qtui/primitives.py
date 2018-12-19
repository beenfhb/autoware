#from python_qt_binding import QtCore
from python_qt_binding import QtWidgets

from .abstruct import AwAbstructWindow
from .abstruct import AwAbstructPanel
from .abstruct import AwAbstructFrame



class AwStrTypeFrame(AwAbstructFrame):

    def __init__(self, guimgr, holder, argdef):
        super(AwStrTypeFrame, self).__init__(guimgr, None)
        self.holder = holder
        self.argdef = argdef

        super(AwStrTypeFrame, self).setup_widget()
        self.edit = QtWidgets.QLineEdit()
        self.edit.setText(self.holder.config.get("args." + self.argdef["name"]))
        self.edit.editingFinished.connect(self.edited)
        self.add_widget(self.edit)
        self.set_title(argdef["name"])

    def edited(self):
        self.holder.config["args." + self.argdef["name"]] = self.edit.text()

class AwIntTypeFrame(AwAbstructFrame):

    def __init__(self, guimgr, holder, argdef):
        super(AwIntTypeFrame, self).__init__(guimgr, None)
        self.holder = holder
        self.argdef = argdef

        super(AwIntTypeFrame, self).setup_widget()
        self.edit = QtWidgets.QLineEdit()
        self.edit.setText(self.holder.config.get("args." + self.argdef["name"]))
        self.edit.editingFinished.connect(self.edited)
        self.add_widget(self.edit)
        self.set_title(argdef["name"])

    def edited(self):
        self.holder.config["args." + self.argdef["name"]] = self.edit.text()

class AwRealTypeFrame(AwAbstructFrame):

    def __init__(self, guimgr, holder, argdef):
        super(AwRealTypeFrame, self).__init__(guimgr, None)
        self.holder = holder
        self.argdef = argdef

        super(AwRealTypeFrame, self).setup_widget()
        self.edit = QtWidgets.QLineEdit()
        self.edit.setText(self.holder.config.get("args." + self.argdef["name"]))
        self.edit.editingFinished.connect(self.edited)
        self.add_widget(self.edit)
        self.set_title(argdef["name"])

    def edited(self):
        self.holder.config["args." + self.argdef["name"]] = self.edit.text()



class AwFileSelectFrame(AwAbstructFrame):

    def __init__(self, guimgr, mirror):
        super(AwFileSelectFrame, self).__init__(guimgr, mirror)

        button = QtWidgets.QPushButton("Browse")
        button.clicked.connect(self.browse)
        self.add_button(button)
        self.set_title(mirror.nodename().capitalize())

        if self.mirror.plugin().gui.get("list") is not True:
            self.widget = QtWidgets.QLineEdit()
            self.widget.setReadOnly(True)
            self.add_widget(self.widget)
        else:
            self.widget = QtWidgets.QTextEdit()
            self.widget.setReadOnly(True)
            self.add_widget(self.widget)

        self.mirror.bind_listener(self)
        self.destroyed.connect(lambda: self.mirror.unbind_listener(self))
        self.config_updated()

    def browse(self):

        import os, re
        if self.mirror.plugin().gui.get("list") is not True:
            filename, filetype = QtWidgets.QFileDialog.getOpenFileName(self, "Select File", os.path.expanduser("~"))
            filename = re.sub("^" + os.environ['HOME'], "$(env HOME)", filename)
        else:
            filename, filetype = QtWidgets.QFileDialog.getOpenFileNames(self, "Select Files", os.path.expanduser("~"))
            filename = map(lambda v: re.sub("^" + os.environ['HOME'], "$(env HOME)", v), filename)

        if filename:
            name = self.mirror.plugin().gui.get("data")
            self.mirror.set_config(name, filename)
            self.config_updated()

    def config_updated(self):
        name = self.mirror.plugin().gui.get("data")
        data = self.mirror.getconfig(name, "")
        if self.mirror.plugin().gui.get("list"):
            data = "\n".join(data)
        self.widget.setText(data)


