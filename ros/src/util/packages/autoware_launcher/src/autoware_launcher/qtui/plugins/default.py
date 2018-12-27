from python_qt_binding import QtCore, QtWidgets
from autoware_launcher.qtui import widgets



def plugin_widgets():
    return \
    {
        "node_panel" : AwDefaultNodePanel,
        "node_frame" : AwDefaultNodeFrame,
        "leaf_panel" : AwDefaultLeafPanel,
        "leaf_frame" : AwDefaultLeafFrame
    }



class AwDefaultNodePanel(widgets.AwAbstructPanel):

    def __init__(self, guimgr, mirror):
        super(AwDefaultNodePanel, self).__init__(guimgr, mirror)

    def setup_widget(self):
        super(AwDefaultNodePanel, self).setup_widget()
        #ToDo: node info edit frame
        for child in self.mirror.children():
            self.add_frame(self.guimgr.create_frame(child))
        
        optional_plugins = self.mirror.plugin().optional_children()
        if optional_plugins:
            self.add_button(AwPluginSelectButton(self.guimgr, self.mirror, self))



class AwDefaultNodeFrame(widgets.AwAbstructFrame):

    def __init__(self, guimgr, mirror):
        super(AwDefaultNodeFrame, self).__init__(guimgr, mirror)

        super(AwDefaultNodeFrame, self).setup_widget()
        self.set_title(self.mirror.name().capitalize() + " : " + self.mirror.get_config("info.title", "No Title"))
        self.add_text_widget(self.mirror.get_config("info.description", "No Description"))
        self.add_button(AwConfigButton(self.guimgr.client(), self.mirror.path()))



class AwDefaultLeafPanel(widgets.AwAbstructPanel):

    def __init__(self, guimgr, mirror):
        super(AwDefaultLeafPanel, self).__init__(guimgr, mirror)
        self.config = self.mirror.config().copy()

    def setup_widget(self):
        super(AwDefaultLeafPanel, self).setup_widget()
        for view in self.mirror.plugin().views():
            self.add_frame(self.guimgr.create_arg_frame(self, view))

        cancel_button = QtWidgets.QPushButton("Cancel")
        update_button = QtWidgets.QPushButton("Update")
        self.add_button(cancel_button)
        self.add_button(update_button)
        cancel_button.clicked.connect(self.cancel_clicked)
        update_button.clicked.connect(self.update_clicked)

    def cancel_clicked(self):
        self.config = self.mirror.config().copy()
        self.setup_widget()
        
    def update_clicked(self):
        response = self.mirror.update({"config": self.config})
        print response



class AwDefaultLeafFrame(widgets.AwAbstructFrame):

    def __init__(self, guimgr, mirror):
        super(AwDefaultLeafFrame, self).__init__(guimgr, mirror)
        self.setup_widget()

    def setup_widget(self):
        super(AwDefaultLeafFrame, self).setup_widget()
        config = self.mirror.config()
        plugin = self.mirror.plugin()

        summary = []
        for view in plugin.views():
            summary.append(self.guimgr.widget(view).summary(view, config))
        self.set_title(self.mirror.name().capitalize() + " : " + self.mirror.get_config("info.title", "No Title"))
        self.add_text_widget("\n".join(summary))
        self.add_button(AwConfigButton(self.guimgr.client(), self.mirror.path()))

    def config_updated(self):
        self.setup_widget()



class AwConfigButton(QtWidgets.QPushButton):

    def __init__(self, client, lpath):
        super(AwConfigButton, self).__init__("Config")
        self.clicked.connect(lambda: client.select_config(lpath))



class AwPluginSelectButton(QtWidgets.QPushButton):

    def __init__(self, guimgr, mirror, parent):
        super(AwPluginSelectButton, self).__init__("Select Plugin")
        def clicked():
            widget = guimgr.create_widget(mirror, parent, guicls = AwPluginSelectWidget)
            window = QtWidgets.QMainWindow(self)
            window.setCentralWidget(widget)
            window.centralWidget().setup_widget()
            window.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
            window.setWindowModality(QtCore.Qt.ApplicationModal)
            window.setGeometry(parent.window().geometry())
            window.show()
        self.clicked.connect(clicked)



class AwPluginSelectWidget(QtWidgets.QWidget):

    def __init__(self, guimgr, mirror, parent):
        super(AwPluginSelectWidget, self).__init__()
        self.guimgr = guimgr
        self.mirror = mirror
        self.parent = parent

    def setup_widget(self):

        cancel_button = QtWidgets.QPushButton("Cancel")
        select_button = QtWidgets.QPushButton("Select")
        self.nodename = QtWidgets.QLineEdit()
        self.nodetype = QtWidgets.QListWidget()

        footer = QtWidgets.QHBoxLayout()
        footer.addStretch()
        footer.addWidget(cancel_button)
        footer.addWidget(select_button)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(QtWidgets.QLabel("Node Name"))
        layout.addWidget(self.nodename)
        layout.addWidget(QtWidgets.QLabel("Node Type"))
        layout.addWidget(self.nodetype)
        layout.addLayout(footer)

        self.setLayout(layout)
        self.setWindowTitle("Select Plugin")

        cancel_button.clicked.connect(self.window().close)
        select_button.clicked.connect(self.create_launch_node)
        #self.group.currentRowChanged.connect(lambda index: self.plugins.setCurrentIndex(index))

        for name, plugins in self.mirror.plugin().optional_children().items():
            for plugin in plugins:
                self.nodetype.addItem(plugin)

    def create_launch_node(self):
        items = self.nodetype.selectedItems()
        if len(items) != 1:
            print "Type is not selected"
            return
        nodetype = items[0].text()
        nodename = self.nodename.text()

        response = self.mirror.addchild(nodename, nodetype)
        if response is None:
            self.parent.setup_widget()
            self.window().close()
        else:
            print response



#experimental
class AwPluginRemoveWindow(QtWidgets.QMainWindow):

    def __init__(self, guimgr, launch, parent):
        super(AwPluginRemoveWindow, self).__init__(parent)
        self.guimgr = guimgr
        self.mirror = launch

        settings = QtCore.QSettings("Autoware", "AutowareLauncher")
        if settings.contains("geometry"):
            self.restoreGeometry(settings.value("geometry"))

        # select
        self.nodelist = QtWidgets.QListWidget()
        for child in self.mirror.children():
            self.nodelist.addItem(child.name())

        # footer
        cancel = QtWidgets.QPushButton("Cancel")
        cancel.clicked.connect(self.close)
        remove = QtWidgets.QPushButton("Remove")
        remove.clicked.connect(self.remove_launch_node)
        footer = QtWidgets.QHBoxLayout()
        footer.addStretch()
        footer.addWidget(cancel)
        footer.addWidget(remove)

        # widget
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.nodelist)
        layout.addLayout(footer)
        widget = QtWidgets.QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)
        self.setWindowTitle("Remove Launch Node")

    def remove_launch_node(self):
        items = self.nodelist.selectedItems()
        if len(items) != 1:
            print "node is not selected"
        else:
            error = self.mirror.remove_child(items[0].text())
            if error:
                print error
            else:
                self.close()