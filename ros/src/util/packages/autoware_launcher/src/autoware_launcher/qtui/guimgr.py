from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets

from .widgets import AwMainWindow
from . import widgets
from . import procmgr



class AwGuiManager(object):

    def __init__(self, sysarg, client):
        self.sysarg = sysarg
        self.client = client
        self.client.load_launch_tree("default")

        self.panels = {}
        self.frames = {}
        self.panels["default"] = widgets.AwDefaultNodePanel
        self.frames["default"] = widgets.AwDefaultNodeFrame
        self.panels["file_select"] = None
        self.frames["file_select"] = widgets.AwFileSelectFrame

    def start(self):

        application = QtWidgets.QApplication(self.sysarg)
        resolution = application.desktop().screenGeometry()
        resolution = min(resolution.width(), resolution.height())

        stylesheet = []
        stylesheet.append("#FrameHeader { border-top: 1px solid; } #FrameHeader, #FrameWidget { padding: 5px; border-bottom: 1px solid; border-left: 1px solid; border-right: 1px solid; }")
        stylesheet.append("* { font-size: " + str(resolution/100) + "px; }")
        application.setStyleSheet(" ".join(stylesheet))

        window = AwMainWindow(self, None)
        window.show()
        return application.exec_()

    def start_autoware_launcher(self, tree):

        application = QtWidgets.QApplication(self.sysarg)
        resolution = application.desktop().screenGeometry()
        resolution = min(resolution.width(), resolution.height())

        stylesheet = []
        stylesheet.append("#FrameHeader { border-top: 1px solid; } #FrameHeader, #FrameWidget { padding: 5px; border-bottom: 1px solid; border-left: 1px solid; border-right: 1px solid; }")
        stylesheet.append("* { font-size: " + str(resolution/100) + "px; }")
        application.setStyleSheet(" ".join(stylesheet))

        window = AwRootWindow()
        window.set_panel(self.create_main_panel(window, tree))
        window.show()
        return application.exec_()
    
    def create_frame(self, launch, guikey = None):

        guikey = guikey or launch.plugin.gui.get("type", "default")
        print "Create Frame: " + launch.nodename() + " " + guikey
        return self.frames[guikey](self, launch)

    def create_node_panel(self, window, launch):

        gui_type = launch.plugin.gui.get("type", "default")
        print "Create Panel: " + launch.nodename() + " " + guitype
        return widgets.AwDefaultNodePanel(self, launch, window)

    def create_window_open_event(self, frame):
        def config_event(): # @QtCore.Slot
            window = AwNodeWindow(frame)
            window.load_geomerty()
            window.set_panel(self.create_node_panel(window, frame.launch))
            window.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
            window.setWindowModality(QtCore.Qt.ApplicationModal)
            window.show()
        return config_event

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
