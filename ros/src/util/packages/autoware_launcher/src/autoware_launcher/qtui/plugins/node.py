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


        for data in self.node.plugin().info():
            self.add_frame(self.guimgr.create_info_frame(self, data))

        for data in self.node.plugin().args():
            self.add_frame(self.guimgr.create_arg_frame(self, data))

        childnodes = {child.name(): child for child in self.node.children()}
        for rule in self.node.plugin().rules():
            if rule["type"] == "unit":
                if self.node.haschild(rule["name"]):
                    child = self.node.getchild(rule["name"])
                    frame = child.plugin().frame()
                    self.add_frame(self.guimgr.create_widget(child, frame))
                    childnodes.pop(rule["name"])
                else:
                    self.add_frame(AwNodeCreateButton(self.node, rule, "Create " + rule["name"]))
                    
        for child in childnodes.values():
            frame = child.plugin().frame()
            self.add_frame(self.guimgr.create_widget(child, frame))
        for rule in self.node.plugin().rules():
            if rule["type"] == "list":
                self.add_frame(AwNodeCreateButton(self.node, rule, "Add " + rule["name"]))


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

        if (self.rule["type"] == "unit") and (len(self.rule["plugin"]) == 1):
            self.node.addchild(self.rule["name"], self.rule["plugin"][0])
        else:
            self.open_window()

    def onselected(self):

        lname = self.ui_lname.text()
        if not lname:
            self.ui_error.setText("Node name is empty")
            return
        items = self.ui_pname.selectedItems()
        if not items:
            self.ui_error.setText("Node type is not selected")
            return

        error = self.node.addchild(lname, items[0].text())
        if error:
            self.ui_error.setText(error)
            return

        self.ui_error.setText("")
        self.ui_window.close()

    def open_window(self):

        window = QtWidgets.QMainWindow(self)
        window.setCentralWidget(QtWidgets.QWidget())
        window.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        window.setWindowModality(QtCore.Qt.ApplicationModal)
        window.setGeometry(self.window().geometry())

        window.setWindowTitle("Create Node")
        widget = window.centralWidget()
        widget.setLayout(QtWidgets.QVBoxLayout())

        error_label = QtWidgets.QLabel()
        error_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        cancel_button = QtWidgets.QPushButton("Cancel")
        select_button = QtWidgets.QPushButton("Select")
        cancel_button.clicked.connect(window.close)
        select_button.clicked.connect(self.onselected)

        lname_edit = QtWidgets.QLineEdit()
        if self.rule["type"] == "unit":
            lname_edit.setText(self.rule["name"])
            lname_edit.setReadOnly(True) 
        else:
            lname_edit.setText(self.rule["name"])
            index = 0
            while self.node.haschild(self.rule["name"] + str(index)):
                index = index + 1
            lname_edit.setText(self.rule["name"] + str(index))

        widget.layout().addWidget(QtWidgets.QLabel("Node Name"))
        widget.layout().addWidget(lname_edit)

        pname_edit = QtWidgets.QListWidget()
        for name in self.rule["plugin"]:
            pname_edit.addItem(name)
        if len(self.rule["plugin"]) == 1:
            pname_edit.setCurrentRow(0)
        widget.layout().addWidget(QtWidgets.QLabel("Node Type"))
        widget.layout().addWidget(pname_edit)

        footer = QtWidgets.QHBoxLayout()
        footer.addWidget(error_label)
        footer.addWidget(cancel_button)
        footer.addWidget(select_button)
        widget.layout().addLayout(footer)

        self.ui_window = window
        self.ui_error  = error_label
        self.ui_lname  = lname_edit
        self.ui_pname  = pname_edit
        window.show()



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