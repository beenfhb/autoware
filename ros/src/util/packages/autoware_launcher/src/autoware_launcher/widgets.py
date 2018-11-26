from python_qt_binding.QtCore import QSettings
from python_qt_binding.QtWidgets import QApplication as AwApplication
from python_qt_binding.QtWidgets import QMainWindow
from python_qt_binding.QtWidgets import QWidget

class AwMainWindow(QMainWindow):

    def __init__(self):

        super(AwMainWindow, self).__init__()
        #self.setCentralWidget(widget_class())
        
        settings = QSettings("Autoware", "AutowareLauncher")
        if settings.contains("geometry"):
            self.restoreGeometry(settings.value("geometry"))
 
    def closeEvent(self, event):

        settings = QSettings("Autoware", "AutowareLauncher")
        settings.setValue("geometry", self.saveGeometry())

class AwNodeWidget(QWidget):

    def __init__(self, parent, plugin, config):

        super(AwNodeWidget, self).__init__(parent)
        self.__plugin = plugin
        self.__config = config
        self.__frames = {}

        print "=================================="
        self.window().setWindowTitle("Window Name")
        print self.__plugin.nodename
        for child in self.__plugin.children.values():
            print child.nodename


    def addFrameWidget(name, widget):

        self.frames[name] = widget

    #@QtSlot(AwConfigNode)
    def loadConfig(self, config):

        self.config = config
        for child_config in config.children:
            child_name = child.getNodeName()
            if child_name in self.frames.keys():
                self.frames[child_name].loadConfig(child_config)

