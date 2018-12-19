from python_qt_binding import QtCore
from python_qt_binding import QtWidgets

from autoware_launcher.core import fspath
from autoware_launcher.core import AwLaunchNodeListenerIF

from .abstruct import AwAbstructWindow
from .abstruct import AwAbstructPanel
from .abstruct import AwAbstructFrame
from .procmgr  import AwProcessMonitorPanel



class AwMainWindow(AwAbstructWindow):

    def __init__(self, guimgr, mirror):

        super(AwMainWindow, self).__init__(None)
        self.guimgr = guimgr
        self.mirror = mirror

        self.load_geomerty()
        self.setWindowTitle("Autoware Launcher")

        self.__init_menu()
        self.setup_widget()

    def closeEvent(self, event):

        self.save_geometry()
        super(AwMainWindow, self).closeEvent(event)

    def setup_widget(self):

        self.profile = AwStandardLaunchPanel(self.guimgr, self.mirror)
        self.process = AwProcessMonitorPanel(self.guimgr, self.mirror)

        widget = QtWidgets.QTabWidget()
        widget.addTab(self.profile, "Profile")
        widget.addTab(self.process, "Process")
        self.setCentralWidget(widget)

    def __init_menu(self):

        load_action = QtWidgets.QAction("Load Profile", self)
        load_action.setShortcut("Ctrl+L")
        load_action.triggered.connect(self.load_profile)

        save_action = QtWidgets.QAction("Save Profile", self)
        save_action.setShortcut("Ctrl+S")
        save_action.triggered.connect(self.save_profile)

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
        filename, filetype = QtWidgets.QFileDialog.getOpenFileName(self, "Load Profile", fspath.profile(), "Launch Profile (*.launch)")
        filename, filetype = os.path.splitext(filename)
        if filename:
            self.guimgr.load_profile(filename)
            self.profile.setup_widget()

    def save_profile(self):
        pass

    def save_profile_as(self):
        import os
        filename, filetype = QtWidgets.QFileDialog.getSaveFileName(self, "Save Profile As", fspath.profile(), "Launch Profile (*.launch)")
        filename, filetype = os.path.splitext(filename)
        if filename:
            if filetype != ".launch":
                filename = filename + filetype
            self.guimgr.save_profile(filename)



class AwStandardLaunchPanel(AwAbstructPanel):

    def __init__(self, guimgr, mirror):
        super(AwStandardLaunchPanel, self).__init__(guimgr, mirror)
        self.setup_widget()

    def setup_widget(self):
        super(AwStandardLaunchPanel, self).setup_widget()
        self.add_frame(AwProfileFrame(self.guimgr, self.mirror))
        for child in self.mirror.children():
            if child.nodename() in ["map", "vehicle", "sensing", "rviz"]:
                self.add_frame(self.guimgr.create_frame(child, guicls = AwLaunchFrame))
        self.add_button(AwConfigButton(self.guimgr, self.mirror))
        #self.add_button(AwLaunchButton(self.guimgr, self.mirror.getchild("rviz"), ("Start", "Stop")))

class AwProfileFrame(AwAbstructFrame):

    def __init__(self, guimgr, mirror):
        super(AwProfileFrame, self).__init__(guimgr, mirror)
        self.setup_widget()

    def setup_widget(self):
        super(AwProfileFrame, self).setup_widget()
        self.set_title("Profile : " + self.mirror.get_config("info.title", "No Title"))
        self.add_text_widget(self.mirror.get_config("info.description", "No Description"))

class AwLaunchFrame(AwAbstructFrame):

    def __init__(self, guimgr, mirror):
        super(AwLaunchFrame, self).__init__(guimgr, mirror)
        self.setup_widget()

    def setup_widget(self):
        super(AwLaunchFrame, self).setup_widget()
        self.set_title(self.mirror.nodename().capitalize() + " : " + self.mirror.get_config("info.title", "No Title"))
        self.add_text_widget(self.mirror.get_config("info.description", "No Description"))
        self.add_button(AwLaunchButton(self.guimgr, self.mirror))






class AwDefaultWindow(AwAbstructWindow):

    def __init__(self, parent):

        super(AwDefaultWindow, self).__init__(parent)
        self.load_geomerty()

class AwDefaultNodePanel(AwAbstructPanel):

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

class AwDefaultNodeFrame(AwAbstructFrame):

    def __init__(self, guimgr, mirror):
        super(AwDefaultNodeFrame, self).__init__(guimgr, mirror)
        self.set_title(self.mirror.nodename().capitalize() + " : " + self.mirror.getconfig("info.title", "No Title"))
        self.add_text_widget(self.mirror.getconfig("info.description", "No Description"))
        self.add_button(AwConfigButton(self.guimgr, self.mirror))

class AwDefaultLeafPanel(AwAbstructPanel):

    def __init__(self, guimgr, mirror):
        super(AwDefaultLeafPanel, self).__init__(guimgr, mirror)
        self.config = mirror.config().copy()

    def setup_widget(self):
        super(AwDefaultLeafPanel, self).setup_widget()
        for argdef in self.mirror.plugin().args():
            self.add_frame(self.guimgr.create_arg_frame(self, argdef))

        cancel_button = QtWidgets.QPushButton("Cancel")
        update_button = QtWidgets.QPushButton("Update")
        self.add_button(cancel_button)
        self.add_button(update_button)
        cancel_button.clicked.connect(self.cancel_clicked)
        update_button.clicked.connect(self.update_clicked)

    def cancel_clicked(self):
        self.window().close()

    def update_clicked(self):
        response = self.mirror.update({"config": self.config})
        if not response["error"]:
            self.window().close()


class AwDefaultLeafFrame(AwAbstructFrame):

    def __init__(self, guimgr, mirror):
        super(AwDefaultLeafFrame, self).__init__(guimgr, mirror)
        config = mirror.config()
        self.mirror.connect(self)
        self.setup_widget()

    def setup_widget(self):
        super(AwDefaultLeafFrame, self).setup_widget()
        argstrs = []
        for argdef in self.mirror.plugin().args():
            argstrs.append("{}: {}".format(argdef["name"], self.mirror.get_config("args."+argdef["name"], "")))
        self.set_title(self.mirror.nodename().capitalize() + " : " + self.mirror.get_config("info.title", "No Title"))
        self.add_text_widget("\n".join(argstrs))
        self.add_button(AwConfigButton(self.guimgr, self.mirror))

    def mirror_updated(self):
        self.setup_widget()


class AwConfigButton(QtWidgets.QPushButton):

    def __init__(self, guimgr, mirror):
        super(AwConfigButton, self).__init__("Config")
        def clicked():
            widget = guimgr.create_panel(mirror)
            window = AwDefaultWindow(self)
            window.setCentralWidget(widget)
            window.centralWidget().setup_widget()
            window.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
            window.setWindowModality(QtCore.Qt.ApplicationModal)
            window.show()
        self.clicked.connect(clicked)

class AwLaunchButton(QtWidgets.QPushButton, AwLaunchNodeListenerIF):

    def __init__(self, guimgr, launch, states = None):
        super(AwLaunchButton, self).__init__()
        self.guimgr = guimgr
        self.mirror = launch
        self.states = states or ("Launch", "Terminate")

        self.mirror.bind_listener(self)
        self.destroyed.connect(lambda: self.mirror.unbind_listener(self))
        self.setup_widget()

    def setup_widget(self):
        self.setText(self.states[0])
        self.clicked.connect(self.on_clicked)

    def exec_requested(self):
        self.setText(self.states[1])

    def term_requested(self):
        self.setEnabled(False)

    def term_completed(self):
        self.setText(self.states[0])
        self.setEnabled(True)

    # QtCore.Slot
    def on_clicked(self):
        state_text = self.text()
        if state_text == self.states[0]: self.mirror.request_exec()
        if state_text == self.states[1]: self.mirror.request_term()
        #if state_text == self.states[0]: self.guimgr.server.request_launch_exec(self.lpath)
        #if state_text == self.states[1]: self.guimgr.server.request_launch_term(self.lpath)



class AwPluginSelectButton(QtWidgets.QPushButton):

    def __init__(self, guimgr, mirror, parent):
        super(AwPluginSelectButton, self).__init__("Select Plugin")
        def clicked():
            widget = guimgr.create_widget(mirror, parent, guicls = AwPluginSelectWidget)
            window = AwDefaultWindow(self)
            window.setCentralWidget(widget)
            window.centralWidget().setup_widget()
            window.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
            window.setWindowModality(QtCore.Qt.ApplicationModal)
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
            print result



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
            error = self.mirror.remove_child(items[0].text())
            if error:
                print error
            else:
                self.close()






class AwDefaultRootFrame(AwAbstructFrame):

    def __init__(self, guimgr, mirror):
        super(AwDefaultRootFrame, self).__init__(guimgr, mirror)
        self.set_title("Profile")
        self.add_widget(self.create_dummy_widget(launch.get_data("info", "title")))



class AwDefaultNodePanelOld(AwAbstructPanel, AwLaunchNodeListenerIF):

    def __init__(self, guimgr, launch, window):
        super(AwDefaultNodePanel, self).__init__(guimgr, launch, window)
        self.add_node_button()
        for child in self.mirror.children():
            self.add_frame(child)

        #self.config_updated()
        self.mirror.bind_listener(self)
        self.destroyed.connect(lambda: self.mirror.unbind_listener(self))

        remove_button = QtWidgets.QPushButton("Remove")
        self.add_button(remove_button)
        def temp():
            window = AwPluginRemoveWindow(self.guimgr, self.mirror, self)
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



