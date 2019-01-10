from python_qt_binding import QtCore
from python_qt_binding import QtWidgets

from ..core import console
from ..core import fspath

from .widgets import AwAbstructPanel
from .widgets import AwAbstructFrame



class AwStarterPanel(AwAbstructPanel):

    def __init__(self, client):
        super(AwStarterPanel, self).__init__(None, None, None)
        self.setup_widget()

    def setup_widget(self):
        super(AwStarterPanel, self).setup_widget()
        #self.add_frame(AwProfileFrame(self.guimgr, self.mirror))
        #for child in self.mirror.children():
        #    if child.name() in ["map", "vehicle", "sensing", "rviz"]:
        #        self.add_frame(self.guimgr.create_frame(child, guicls = AwLaunchFrame))
        #self.add_button(AwConfigButton(self.guimgr, self.mirror))
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



