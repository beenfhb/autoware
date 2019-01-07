from python_qt_binding import QtCore, QtWidgets
from autoware_launcher.qtui import widgets



def plugin_widgets():
    return \
    {
        "panel" : AwDefaultNodePanel,
        "frame" : AwDefaultNodeFrame,
    }



class AwDefaultNodePanel(widgets.AwAbstructPanel):

    def __init__(self, guimgr, node, view):
        super(AwDefaultNodePanel, self).__init__(guimgr, node, view)
        self.config = self.node.config().copy()

    def setup_widget(self):
        super(AwDefaultNodePanel, self).setup_widget()
        self.debug = QtWidgets.QLabel(self.node.tostring())
        self.debug.setVisible(False)
        self.add_frame(self.debug)
        self.setFocusPolicy(QtCore.Qt.ClickFocus)

        cancel_button = QtWidgets.QPushButton("Cancel")
        update_button = QtWidgets.QPushButton("Update")
        self.add_button(cancel_button)
        self.add_button(update_button)
        cancel_button.clicked.connect(self.cancel_clicked)
        update_button.clicked.connect(self.update_clicked)

        for arg in self.node.plugin().args():
            self.add_frame(self.guimgr.create_arg_frame(self, arg))

        for child in self.node.children():
            frame = child.plugin().frame()
            self.add_frame(self.guimgr.create_widget(child, frame))

        for rule in self.node.plugin().rules():
            if rule["type"] == "unit":
                if not self.node.haschild(rule["name"]):
                    if rule.get("scan"):
                        self.add_frame(AwPluginSelectButton(self.guimgr, self.node, self))
                    else:
                        self.add_frame(AwNodeCreateButton(self.node, rule, "Create " + rule["name"]))
            if rule["type"] == "list":
                if rule.get("scan"):
                    self.add_frame(AwPluginSelectButton(self.guimgr, self.node, self))
                else:
                    self.add_frame(AwNodeCreateButton(self.node, rule, "Create " + rule["name"]))

    def cancel_clicked(self):
        self.config = self.node.config().copy()
        self.setup_widget()
        
    def update_clicked(self):
        response = self.node.update({"config": self.config})
        print response

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_D:
            self.debug.setVisible(not self.debug.isVisible())
            event.accept()
        else:
            super(AwDefaultNodePanel, self).keyPressEvent(event)
        


class AwDefaultNodeFrame(widgets.AwAbstructFrame):

    def __init__(self, guimgr, node, view):
        super(AwDefaultNodeFrame, self).__init__(guimgr, node, view)

        super(AwDefaultNodeFrame, self).setup_widget()
        self.set_title(self.node.name().capitalize() + " : " + self.node.get_config("info.title", "No Title"))
        self.add_text_widget(self.node.get_config("info.description", "No Description"))
        self.add_button(AwConfigButton(self.guimgr.client(), self.node.path()))



class AwDefaultLeafPanel(widgets.AwAbstructPanel):

    def __init__(self, guimgr, node):
        super(AwDefaultLeafPanel, self).__init__(guimgr, node)
        self.config = self.node.config().copy()

    def setup_widget(self):
        super(AwDefaultLeafPanel, self).setup_widget()
        for view in self.node.plugin().views():
            self.add_frame(self.guimgr.create_arg_frame(self, view))

        cancel_button = QtWidgets.QPushButton("Cancel")
        update_button = QtWidgets.QPushButton("Update")
        self.add_button(cancel_button)
        self.add_button(update_button)
        cancel_button.clicked.connect(self.cancel_clicked)
        update_button.clicked.connect(self.update_clicked)

    def cancel_clicked(self):
        self.config = self.node.config().copy()
        self.setup_widget()
        
    def update_clicked(self):
        response = self.node.update({"config": self.config})
        print response



class AwDefaultLeafFrame(widgets.AwAbstructFrame):

    def __init__(self, guimgr, node):
        super(AwDefaultLeafFrame, self).__init__(guimgr, node)
        self.setup_widget()

    def setup_widget(self):
        super(AwDefaultLeafFrame, self).setup_widget()
        config = self.node.config()
        plugin = self.node.plugin()

        summary = []
        for view in plugin.views():
            summary.append(self.guimgr.widget(view).summary(view, config))
        self.set_title(self.node.name().capitalize() + " : " + self.node.get_config("info.title", "No Title"))
        self.add_text_widget("\n".join(summary))
        self.add_button(AwConfigButton(self.guimgr.client(), self.node.path()))

    def config_updated(self):
        self.setup_widget()



class AwConfigButton(QtWidgets.QPushButton):

    def __init__(self, client, lpath):
        super(AwConfigButton, self).__init__("Config")
        self.clicked.connect(lambda: client.select_config(lpath))



class AwNodeCreateButton(QtWidgets.QPushButton):

    def __init__(self, node, rule, text):
        super(AwNodeCreateButton, self).__init__(text)
        self.node = node
        self.rule = rule
        self.clicked.connect(self.onclicked)

    def onclicked(self):
        self.node.addchild(self.rule["name"], self.rule["plugin"])



class AwPluginSelectButton(QtWidgets.QPushButton):

    def __init__(self, guimgr, node, parent):
        super(AwPluginSelectButton, self).__init__("Create")
        def clicked():
            widget = AwPluginSelectWidget(guimgr, node, parent)
            window = QtWidgets.QMainWindow(self)
            window.setCentralWidget(widget)
            window.centralWidget().setup_widget()
            window.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
            window.setWindowModality(QtCore.Qt.ApplicationModal)
            window.setGeometry(parent.window().geometry())
            window.show()
        self.clicked.connect(clicked)



class AwPluginSelectWidget(QtWidgets.QWidget):

    def __init__(self, guimgr, node, parent):
        super(AwPluginSelectWidget, self).__init__()
        self.guimgr = guimgr
        self.node = node
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

        for rule in self.node.plugin().rules():
            if rule.get("scan"):
                for name in self.node.plugin()._AwPluginNode__tree.scan(rule["plugin"]):
                    self.nodetype.addItem(name)

    def create_launch_node(self):
        items = self.nodetype.selectedItems()
        if len(items) != 1:
            print "Type is not selected"
            return
        nodetype = items[0].text()
        nodename = self.nodename.text()

        response = self.node.addchild(nodename, nodetype)
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
        self.node = launch

        settings = QtCore.QSettings("Autoware", "AutowareLauncher")
        if settings.contains("geometry"):
            self.restoreGeometry(settings.value("geometry"))

        # select
        self.nodelist = QtWidgets.QListWidget()
        for child in self.node.children():
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
            error = self.node.remove_child(items[0].text())
            if error:
                print error
            else:
                self.close()