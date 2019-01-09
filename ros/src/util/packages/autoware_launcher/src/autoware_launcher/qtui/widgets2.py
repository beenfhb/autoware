from python_qt_binding import QtCore
from python_qt_binding import QtWidgets

from autoware_launcher.core import console
from autoware_launcher.core import fspath

from .widgets import AwAbstructWindow
from .widgets import AwAbstructPanel
from .widgets import AwAbstructFrame



class AwMainWindow(AwAbstructWindow):

    def __init__(self, client):

        super(AwMainWindow, self).__init__(None)
        self.client = client

        self.load_geomerty()
        self.setWindowTitle("Autoware Launcher")

        self.__init_menu()

    def closeEvent(self, event):

        self.save_geometry()
        super(AwMainWindow, self).closeEvent(event)

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
            self.client.load_profile(filename)

    def save_profile(self):
        pass

    def save_profile_as(self):
        import os
        filename, filetype = QtWidgets.QFileDialog.getSaveFileName(self, "Save Profile As", fspath.profile(), "Launch Profile (*.launch)")
        filename, filetype = os.path.splitext(filename)
        if filename:
            if filetype != ".launch":
                filename = filename + filetype
            self.client.save_profile(filename)



class AwQuickStartPanel(AwAbstructPanel):

    def __init__(self, guimgr, target, option):
        super(AwQuickStartPanel, self).__init__(guimgr, mirror)
        self.setup_widget()

    def setup_widget(self):
        super(AwQuickStartPanel, self).setup_widget()
        self.add_frame(AwProfileFrame(self.guimgr, self.mirror))
        for child in self.mirror.children():
            if child.name() in ["map", "vehicle", "sensing", "rviz"]:
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
        self.set_title(self.mirror.name().capitalize() + " : " + self.mirror.get_config("info.title", "No Title"))
        self.add_text_widget(self.mirror.get_config("info.description", "No Description"))
        self.add_button(AwLaunchButton(self.guimgr, self.mirror))






class AwDefaultWindow(AwAbstructWindow):

    def __init__(self, parent):

        super(AwDefaultWindow, self).__init__(parent)
        self.load_geomerty()


class AwLaunchButton(QtWidgets.QPushButton):

    def __init__(self, guimgr, launch, states = None):
        super(AwLaunchButton, self).__init__()
        self.guimgr = guimgr
        self.mirror = launch
        self.states = states or ("Launch", "Terminate")

        self.mirror.bind(self)
        self.destroyed.connect(lambda: self.mirror.unbind(self))
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
        if state_text == self.states[0]: self.mirror.launch(True)
        if state_text == self.states[1]: self.mirror.launch(False)










class AwDefaultRootFrame(AwAbstructFrame):

    def __init__(self, guimgr, mirror):
        super(AwDefaultRootFrame, self).__init__(guimgr, mirror)
        self.set_title("Profile")
        self.add_widget(self.create_dummy_widget(launch.get_data("info", "title")))



class AwDefaultNodePanelOld(AwAbstructPanel):

    def __init__(self, guimgr, launch, window):
        super(AwDefaultNodePanel, self).__init__(guimgr, launch, window)
        self.add_node_button()
        for child in self.mirror.children():
            self.add_frame(child)

        #self.node_updated()
        self.mirror.bind(self)
        self.destroyed.connect(lambda: self.mirror.unbind(self))

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
                if frame.launch.name() == name:
                    self.layout().takeAt(i).widget().deleteLater()
                    return



