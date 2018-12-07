from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets

from autoware_launcher.core import AwLaunchNodeListenerIF



class AwAbstructPanel(QtWidgets.QWidget):

    def __init__(self, guimgr, launch, window):
        super(AwAbstructPanel, self).__init__()
        self.guimgr = guimgr
        self.launch = launch
        self.window = window

        # Panel Footer
        layout = QtWidgets.QHBoxLayout()
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)
        layout.addStretch()
        self.footer = QtWidgets.QWidget()
        self.footer.setLayout(layout)

        # Panel Layout
        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)
        layout.addStretch()
        layout.addWidget(self.footer)
        self.setLayout(layout)


    def add_frame(self, launch):
        index = self.layout().count() - 2
        self.layout().insertWidget(index, self.guimgr.create_frame(launch))

    def add_node_button(self):
        button = QtWidgets.QPushButton("Add Node")
        self.footer.layout().addWidget(button)
        def temp():
            window = AwPluginSelectWindow(self.guimgr, self.launch, self)
            window.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
            window.setWindowModality(QtCore.Qt.ApplicationModal)
            window.show()
        button.clicked.connect(temp)



#temporary
class AwPluginSelectWindow(QtWidgets.QMainWindow):

    def __init__(self, guimgr, launch, parent):
        super(AwPluginSelectWindow, self).__init__(parent)
        self.guimgr = guimgr
        self.launch = launch

        settings = QtCore.QSettings("Autoware", "AutowareLauncher")
        if settings.contains("geometry"):
            self.restoreGeometry(settings.value("geometry"))

        widget = QtWidgets.QWidget()
        self.group   = QtWidgets.QListWidget()
        self.plugins = QtWidgets.QStackedWidget()
        self.info   = QtWidgets.QLabel()
        layout = QtWidgets.QGridLayout()
        layout.addWidget(self.group,   0, 0, 2, 1)
        layout.addWidget(self.plugins, 0, 1, 2, 1)
        layout.addWidget(self.info ,   2, 0, 1, 2)

        layout2 = QtWidgets.QHBoxLayout()
        layout2.addWidget(QtWidgets.QLabel("Node Name"))

        self.edit = QtWidgets.QLineEdit()
        layout2.addWidget(self.edit)

        button2 = QtWidgets.QPushButton("Cancel")
        button2.clicked.connect(self.close)
        layout2.addWidget(button2)

        button1 = QtWidgets.QPushButton("OK")
        button1.clicked.connect(self.create_launch_node)
        layout2.addWidget(button1)

        vlayout = QtWidgets.QVBoxLayout()
        vlayout.addLayout(layout)
        vlayout.addLayout(layout2)
        widget.setLayout(vlayout)
        
        self.setCentralWidget(widget)
        self.setWindowTitle("Select Plugin")
        self.group.currentRowChanged.connect(lambda index: self.plugins.setCurrentIndex(index))

        for group_name, group_data in self.guimgr.loader._AwPluginLoader__plugins.items():
            plugin = QtWidgets.QListWidget()
            self.group.addItem(group_name)
            self.plugins.addWidget(plugin)
            for plugin_name in group_data:
                plugin.addItem(plugin_name)

    def create_launch_node(self):
        plugin = None
        group_index = self.group.currentRow()
        if group_index != -1:
            plugin_index = self.plugins.widget(group_index).currentRow()
            if plugin_index != -1:
                group_name  =  self.group.item(group_index).text()
                plugin_name =  self.plugins.widget(group_index).item(plugin_index).text()
                plugin = self.guimgr.loader.find(group_name + "/" + plugin_name)

        print plugin

        result = self.launch.create_child(self.edit.text(), plugin)
        if type(result) is not str:
            self.close()
        else:
            print result



class AwQuickStartPanel(AwAbstructPanel):

    def __init__(self, guimgr, launch, window):
        super(AwQuickStartPanel, self).__init__(guimgr, launch, window)
        self.add_node_button()
        self.add_frame(self.launch)
        for child in self.launch.children():
            self.add_frame(child)




class AwDefaultNodePanel(AwAbstructPanel):

    def __init__(self, guimgr, launch, window):
        super(AwDefaultNodePanel, self).__init__(guimgr, launch, window)
        for child in self.launch.children():
            self.add_frame(child)



class AwAbstructFrame(QtWidgets.QWidget):

    def __init__(self, guimgr, launch):
        super(AwAbstructFrame, self).__init__()
        self.guimgr = guimgr
        self.launch = launch

        # Frame Header
        self.title = QtWidgets.QLabel("No Title")
        self.title.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        layout = QtWidgets.QHBoxLayout()
        layout.setContentsMargins(5, 2, 2, 2)
        layout.setSpacing(0)
        layout.addWidget(self.title)
        self.header = QtWidgets.QWidget()
        self.header.setObjectName("FrameHeader")
        self.header.setLayout(layout)

        # Frame Layout
        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self.header)
        self.setLayout(layout)

    def set_title(self, title):
        self.title.setText(title + " : " + self.__class__.__name__)

    def add_widget(self, widget):
        widget.setObjectName("FrameWidget")
        self.layout().addWidget(widget)

    def add_button(self, button):
        self.header.layout().addWidget(button)

    def add_launch_button(self):
        self.add_button(AwLaunchButton(self.launch))

    def add_config_button(self):

        button = QtWidgets.QPushButton("Config")
        button.clicked.connect(self.guimgr.create_window_open_event(self))
        self.add_button(button)

    def create_dummy_widget(self, text):

        layout = QtWidgets.QHBoxLayout()
        layout.setContentsMargins(5, 2, 2, 2)
        layout.setSpacing(0)
        layout.addWidget(QtWidgets.QLabel(text))

        widget = QtWidgets.QWidget()
        widget.setLayout(layout)
        return widget



class AwDefaultNodeFrame(AwAbstructFrame):

    def __init__(self, guimgr, launch):
        super(AwDefaultNodeFrame, self).__init__(guimgr, launch)
        self.set_title(launch.nodename().capitalize())
        self.add_config_button()
        self.add_launch_button()
        self.add_widget(self.create_dummy_widget(launch.get_data("info", "title")))



class AwFileSelectFrame(AwAbstructFrame, AwLaunchNodeListenerIF):

    def __init__(self, guimgr, launch):
        super(AwFileSelectFrame, self).__init__(guimgr, launch)

        button = QtWidgets.QPushButton("Browse")
        button.clicked.connect(self.browse)
        self.add_button(button)
        self.set_title(launch.nodename().capitalize())

        if self.launch.plugin.gui.get("list") is not True:
            self.widget = QtWidgets.QLineEdit()
            self.widget.setReadOnly(True)
            self.add_widget(self.widget)
        else:
            self.widget = QtWidgets.QTextEdit()
            self.widget.setReadOnly(True)
            self.add_widget(self.widget)

        self.config_updated()
        self.launch.bind_listener(self)
        self.destroyed.connect(lambda: self.launch.unbind_listener(self))

    def browse(self):

        import os, re
        if self.launch.plugin.gui.get("list") is not True:
            filename, filetype = QtWidgets.QFileDialog.getOpenFileName(self, "Select File", os.path.expanduser("~"))
            filename = re.sub("^" + os.environ['HOME'], "$(env HOME)", filename)
        else:
            filename, filetype = QtWidgets.QFileDialog.getOpenFileNames(self, "Select Files", os.path.expanduser("~"))
            filename = map(lambda v: re.sub("^" + os.environ['HOME'], "$(env HOME)", v), filename)

        if filename:
            grp, key = self.launch.plugin.gui.get("data").split(".")
            self.launch.set_data(grp, key, filename)

    def config_updated(self):
        grp, key = self.launch.plugin.gui.get("data").split(".")
        data = self.launch.get_data(grp, key, "")
        if self.launch.plugin.gui.get("list") is True:
            data = "\n".join(data)
        self.widget.setText(data)



class AwLaunchButton(QtWidgets.QPushButton, AwLaunchNodeListenerIF):

    def __init__(self, node):
        super(AwLaunchButton, self).__init__()
        self.node = node
        self.setText("Launch")
        self.clicked.connect(self.on_clicked)

        self.node.bind_listener(self)
        self.destroyed.connect(lambda: self.node.unbind_listener(self))

    def exec_requested(self):
        self.setText("Terminate")

    def term_requested(self):
        self.setEnabled(False)

    def term_completed(self):
        self.setText("Launch")
        self.setEnabled(True)

    # QtCore.Slot
    def on_clicked(self):
        state_text = self.text()
        if state_text == "Launch":
            self.node.request_exec()
        if state_text == "Terminate":
            self.node.request_term()

