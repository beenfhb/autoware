from python_qt_binding import QtCore
from python_qt_binding import QtWidgets

from autoware_launcher.core import fsys
from autoware_launcher.core import AwLaunchNodeListenerIF

from .abstruct import AwAbstructWindow
from .abstruct import AwAbstructPanel
from .abstruct import AwAbstructFrame
from .procmgr  import AwProcessMonitorPanel



class AwMainWindow(AwAbstructWindow):

    def __init__(self, guimgr, launch_path):

        super(AwMainWindow, self).__init__(guimgr, launch_path)
        self.load_geomerty()
        self.setWindowTitle("Autoware Launcher")

        self.__init_menu()
        self.update_widget()

    def closeEvent(self, event):

        self.save_geometry()
        super(AwMainWindow, self).closeEvent(event)

    def update_widget(self):

        profile = AwStandardHomePanel(self.guimgr, launch_path)
        #process = AwProcessMonitorPanel(self.guimgr, None)

        widget = QtWidgets.QTabWidget()
        widget.addTab(profile, "Profile")
        #widget.addTab(process, "Process")
        self.setCentralWidget(widget)

    def __init_menu(self):

        load_action = QtWidgets.QAction("Load Profile", self)
        load_action.setShortcut("Ctrl+L")
        load_action.triggered.connect(self.load_profile)

        save_action = QtWidgets.QAction("Save Profile", self)
        save_action.setShortcut("Ctrl+S")

        save_as_action = QtWidgets.QAction("Save Profile As", self)
        save_as_action.setShortcut("Ctrl+A")
        save_as_action.triggered.connect(self.save_profile_as)

        mainmenu = self.menuBar()
        filemenu = mainmenu.addMenu("File")
        filemenu.addAction(load_action)
        filemenu.addAction(save_action)
        filemenu.addAction(save_as_action)

    def load_profile(self):
        import os
        filename, filetype = QtWidgets.QFileDialog.getOpenFileName(self, "Load Profile", fsys.profile_path(), "Launch Profile (*.launch)")
        filename, filetype = os.path.splitext(filename)
        if filename:
            self.guimgr.client.load_launch_tree(filename)
            self.update_widget()


    def save_profile_as(self):
        import os
        filename, filetype = QtWidgets.QFileDialog.getSaveFileName(self, "Save Profile As", os.path.expanduser("~"), "Launch Profile (*.launch)")
        filename, filetype = os.path.splitext(filename)
        if filetype != ".launch":
            filename = filename + filetype



class AwStandardHomePanel(AwAbstructPanel):

    def __init__(self, guimgr, launch_path):
        super(AwStandardHomePanel, self).__init__(guimgr, launch_path)
        self.update_widget()

    def update_widget(self):
        launch = self.guimgr.client.find_launch_node(self.launch_path)
        self.add_frame(AwProfileFrame(self.guimgr, self.launch_path))
        for child in launch.children():
            if child.nodename() in ["map", "vehicle", "sensing", "rviz"]:
                #self.add_frame(self.guimgr.create_frame(child))
                self.add_frame(AwLaunchFrame(self.guimgr, child.nodepath()))
        #self.add_button(AwLaunchButton(self.launch.getchild("rviz")))



class AwProfileFrame(AwAbstructFrame):

    def __init__(self, guimgr, launch_path):
        super(AwProfileFrame, self).__init__(guimgr, launch_path)
        self.update_widget()

    def update_widget(self):
        launch = self.guimgr.client.find_launch_node(self.launch_path)
        self.set_title("Profile : " + launch.config.get("info.title"))
        self.add_text_widget(launch.config.get("info.description"))



class AwLaunchFrame(AwAbstructFrame):

    def __init__(self, guimgr, launch_path):
        super(AwLaunchFrame, self).__init__(guimgr, launch_path)
        self.update_widget()

    def update_widget(self):
        launch = self.guimgr.client.find_launch_node(self.launch_path)
        self.set_title("__NAME__ : " + launch.config.get("info.title"))
        self.add_text_widget(launch.config.get("info.description"))

        # ToDo
        self.add_button(QtWidgets.QPushButton("Launch"))



#experimental
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

        result = self.launch.create_child(self.edit.text(), plugin)
        if type(result) is not str:
            self.close()
        else:
            print result



#experimental
class AwPluginRemoveWindow(QtWidgets.QMainWindow):

    def __init__(self, guimgr, launch, parent):
        super(AwPluginRemoveWindow, self).__init__(parent)
        self.guimgr = guimgr
        self.launch = launch

        settings = QtCore.QSettings("Autoware", "AutowareLauncher")
        if settings.contains("geometry"):
            self.restoreGeometry(settings.value("geometry"))

        # select
        self.nodelist = QtWidgets.QListWidget()
        for child in self.launch.children():
            self.nodelist.addItem(child.nodename())

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
            error = self.launch.remove_child(items[0].text())
            if error:
                print error
            else:
                self.close()






class AwDefaultRootFrame(AwAbstructFrame):

    def __init__(self, guimgr, launch):
        super(AwDefaultRootFrame, self).__init__(guimgr, launch)
        self.set_title("Profile")
        self.add_widget(self.create_dummy_widget(launch.get_data("info", "title")))



class AwDefaultNodePanel(AwAbstructPanel, AwLaunchNodeListenerIF):

    def __init__(self, guimgr, launch, window):
        super(AwDefaultNodePanel, self).__init__(guimgr, launch, window)
        self.add_node_button()
        for child in self.launch.children():
            self.add_frame(child)

        #self.config_updated()
        self.launch.bind_listener(self)
        self.destroyed.connect(lambda: self.launch.unbind_listener(self))

        remove_button = QtWidgets.QPushButton("Remove")
        self.add_button(remove_button)
        def temp():
            window = AwPluginRemoveWindow(self.guimgr, self.launch, self)
            window.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
            window.setWindowModality(QtCore.Qt.ApplicationModal)
            window.show()
        remove_button.clicked.connect(temp)

    def config_created(self, child):
        self.add_frame(child)

    def config_removed(self, name):
        for i in range(self.layout().count()):
            frame = self.layout().itemAt(i).widget()
            if isinstance(frame, AwAbstructFrame):
                if frame.launch.nodename() == name:
                    self.layout().takeAt(i).widget().deleteLater()
                    return


class AwDefaultNodeFrame(AwAbstructFrame):

    def __init__(self, guimgr, launch):
        super(AwDefaultNodeFrame, self).__init__(guimgr, launch)
        self.set_title(launch.nodename().capitalize())
        self.add_config_button()
        self.add_launch_button()
        self.add_text_widget("No Description")



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

