from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets

from autoware_launcher.core import console
from autoware_launcher.core import AwLaunchClientIF

from .widgets import AwMainWindow
from . import widgets
from . import procmgr



class AwQtGuiClient(AwLaunchClientIF):

    def __init__(self, sysarg, server):
        self.sysarg = sysarg
        self.server = server
        self.elements = {}

        #self.server.register_client()
        #self.server.request_launch_load("default")
        self.server.request_launch_init("node/root")

        self.panels = {}
        self.frames = {}
        self.panels["default"] = widgets.AwDefaultNodePanel
        self.frames["default"] = widgets.AwDefaultNodeFrame
        self.panels["file_select"] = None
        self.frames["file_select"] = widgets.AwFileSelectFrame

    def element_created(self, path, listener):
        console.info("element_created {} {}".format(path, listener))
        if not self.elements.get(path): self.elements[path] = []
        self.elements[path].append(listener)

    def element_deleted(self, path, listener):
        console.info("element_deleted {} {}".format(path, listener))
        self.elements[path].remove(listener)
        if not self.elements.get(path): self.elements.pop(path)

    def start(self):

        application = QtWidgets.QApplication(self.sysarg)
        resolution = application.desktop().screenGeometry()
        resolution = min(resolution.width(), resolution.height())

        stylesheet = []
        stylesheet.append("#FrameHeader { border-top: 1px solid; } #FrameHeader, #FrameWidget { padding: 5px; border-bottom: 1px solid; border-left: 1px solid; border-right: 1px solid; }")
        stylesheet.append("* { font-size: " + str(resolution/100) + "px; }")
        application.setStyleSheet(" ".join(stylesheet))

        launch = self.server.request_launch_find("root")
        window = QtWidgets.QMainWindow()
        window = AwMainWindow(self, launch)
        window.show()
        return application.exec_()

    def create_frame(self, launch, guikey = None, guicls = None):
        print "Create Frame: {:<7} Key: {} Class: {}".format(launch.nodename(), guikey, guicls)
        if not guicls:
            guikey = guikey or launch.plugin.gui.get("type", "default")
            guicls = self.frames[guikey]
        return guicls(self, launch)

    def create_panel(self, launch, guikey = None, guicls = None):
        print "Create Panel: {:<7} Key: {} Class: {}".format(launch.nodename(), guikey, guicls)
        if not guicls:
            guikey = guikey or launch.plugin.gui.get("type", "default")
            guicls = self.panels[guikey]
        return guicls(self, launch)

    def create_frame_entire_vlayout(self):
        layout = QtWidgets.QVBoxLayout()
        layout.setSpacing(0)
        layout.setContentsMargins(0, 0, 0, 0)
        return layout

    def create_frame_header_hlayout(self):
        layout = QtWidgets.QHBoxLayout()
        layout.setSpacing(0)
        layout.setContentsMargins(5, 2, 2, 2)
        return layout
