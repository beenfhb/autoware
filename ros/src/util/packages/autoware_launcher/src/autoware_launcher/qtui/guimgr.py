from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets

from . import widgets
from . import procmgr



class AwGuiManager(object):

    def __init__(self, sys_argv):
        self.sys_argv = sys_argv
        self.panels = {}
        self.frames = {}

        self.panels["default"] = widgets.AwDefaultNodePanel
        self.frames["default"] = widgets.AwDefaultNodeFrame

        self.panels["file_select"] = None
        self.frames["file_select"] = widgets.AwFileSelectFrame

    def start_autoware_launcher(self, tree):

        application = QtWidgets.QApplication(self.sys_argv)
        resolution = application.desktop().screenGeometry()
        resolution = min(resolution.width(), resolution.height())

        stylesheet = []
        stylesheet.append("#FrameHeader { border-top: 1px solid; } #FrameHeader, #FrameWidget { padding: 5px; border-bottom: 1px solid; border-left: 1px solid; border-right: 1px solid; }")
        stylesheet.append("* { font-size: " + str(resolution/100) + "px; }")
        application.setStyleSheet(" ".join(stylesheet))

        window = AwMainWindow()
        window.set_panel(self.create_main_panel(window, tree))
        window.show()
        return application.exec_()
    
    def create_frame(self, launch):

        gui_type = launch.plugin.gui.get("type", "default")
        print "Create Frame: " + launch.nodename() + " " + gui_type
        return self.frames[gui_type](self, launch)

    def create_node_panel(self, window, launch):

        gui_type = launch.plugin.gui.get("type", "default")
        print "Create Panel: " + launch.nodename() + " " + gui_type
        return widgets.AwDefaultNodePanel(self, window, launch)

    def create_main_panel(self, window, tree):

        profile = widgets.AwQuickStartPanel(self, window, tree.root)
        process = procmgr.AwLaunchWidget(self, tree)
        widget = QtWidgets.QTabWidget()
        widget.addTab(profile, "Profile")
        widget.addTab(process, "Process")
        return widget

    def create_window_open_event(self, frame):
        def config_event(): # @QtCore.Slot
            window = AwNodeWindow(frame)
            window.load_geomerty()
            window.set_panel(self.create_node_panel(window, frame.launch))
            window.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
            window.setWindowModality(QtCore.Qt.ApplicationModal)
            window.show()
        return config_event


class AwNodeWindow(QtWidgets.QMainWindow):

    def load_geomerty(self):
        settings = QtCore.QSettings("Autoware", "AutowareLauncher")
        if settings.contains("geometry"):
            self.restoreGeometry(settings.value("geometry"))

    def save_geometry(self):
        settings = QtCore.QSettings("Autoware", "AutowareLauncher")
        settings.setValue("geometry", self.saveGeometry())

    def set_panel(self, panel):
        self.setCentralWidget(panel)
        self.setWindowTitle(panel.__class__.__name__)



class AwMainWindow(AwNodeWindow):

    def __init__(self):
        super(AwMainWindow, self).__init__()
        self.load_geomerty()
