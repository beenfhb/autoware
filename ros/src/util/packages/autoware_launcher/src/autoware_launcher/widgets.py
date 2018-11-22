from python_qt_binding.QtCore import QSettings
from python_qt_binding.QtWidgets import QApplication as AwApplication
from python_qt_binding.QtWidgets import QMainWindow
from python_qt_binding.QtWidgets import QWidget

class AwMainWindow(QMainWindow):

    def __init__(self, widget_class):

        super(AwMainWindow, self).__init__()
        self.setCentralWidget(widget_class())
        
        settings = QSettings("Autoware", "AutowareLauncher")
        self.restoreGeometry(settings.value("geometry"))
 
    def closeEvent(self, event):

        settings = QSettings("Autoware", "AutowareLauncher")
        settings.setValue("geometry", self.saveGeometry())

class AwBasicWidget(QWidget):

    def __init__(self):

        super(AwBasicWidget, self).__init__()
        self.config = None
        self.frames = {}

    def addFrameWidget(name, widget):

        self.frames[name] = widget

    #@QtSlot(AwConfigNode)
    def loadConfig(self, config):

        self.config = config
        for child_config in config.children:
            child_name = child.getNodeName()
            if child_name in self.frames.keys():
                self.frames[child_name].loadConfig(child_config)
