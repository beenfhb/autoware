from python_qt_binding.QtCore import QSettings
from python_qt_binding.QtWidgets import QApplication as AwApplication
from python_qt_binding.QtWidgets import QMainWindow
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QVBoxLayout

from python_qt_binding import QtCore
from python_qt_binding.QtGui import QPainter
from python_qt_binding.QtWidgets import QSizePolicy
from python_qt_binding.QtWidgets import QStyle
from python_qt_binding.QtWidgets import QStyleOption
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QHBoxLayout



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

    def __init__(self, parent, awnode):

        super(AwNodeWidget, self).__init__(parent)
        #self.__awnode = awnode
        self.__frames = {}

        # Apply plugin
        self.window().setWindowTitle(awnode.schema["text"])
        mylayout = QVBoxLayout()
        for child in awnode.children.values():
            frame = AwNodeFrame(child)
            mylayout.addWidget(frame)
            self.__frames[child.nodename] = frame
        mylayout.addStretch()
        self.setLayout(mylayout)
    
    #@QtSlot(AwConfigNode)
    def loadConfig(self, config):

        self.config = config
        for child_config in config.children:
            child_name = child.getNodeName()
            if child_name in self.frames.keys():
                self.frames[child_name].loadConfig(child_config)



class AwNodeFrame(QWidget):

    def __init__(self, awnode):

    	super(AwNodeFrame, self).__init__()
        self.__awnode = awnode

        self.mylayout = QVBoxLayout()
        self.mylayout.setContentsMargins(0, 0, 0, 0)
        self.mylayout.setSpacing(0)
        self.setLayout(self.mylayout)

        button = QPushButton("Config")
        button.clicked.connect(self.openSubConfig)

        menu = AwNodeFrameMenu(awnode.schema["text"])
        menu.addMenuButton(button)
      
        body = QLabel("Details")
        self.setFrameWidgets(menu, body)

    def setFrameWidgets(self, menu, body):

        self.setObjectName("FrameWidget")
        menu.setObjectName("FrameMenu")
        body.setObjectName("FrameBody")
        self.setStyleSheet("#FrameWidget { border: 1px solid; } #FrameMenu { padding: 3px; border-bottom: 1px solid; } #FrameBody { padding: 3px; }")
        self.mylayout.addWidget(menu)
        self.mylayout.addWidget(body)
        
    def paintEvent(self, event):

        style_option = QStyleOption()
        style_option.initFrom(self)
        painter = QPainter(self)
        self.style().drawPrimitive(QStyle.PE_Widget, style_option, painter, self)

    def openSubConfig(self):

        window = QMainWindow(self)
        widget = AwNodeWidget(window, self.__awnode)
        window.setCentralWidget(widget)
        window.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        window.setWindowModality(QtCore.Qt.ApplicationModal)
        window.show()

class AwNodeFrameMenu(QWidget):

    def __init__(self, title = "[No Title]"):

    	super(AwNodeFrameMenu, self).__init__()
        self.mylayout = QHBoxLayout()
        self.setLayout(self.mylayout)

        label = QLabel(title)
        label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.mylayout.addWidget(label)

    def addMenuButton(self, button):

        self.mylayout.addWidget(button)

    def paintEvent(self, event):

        style_option = QStyleOption()
        style_option.initFrom(self)
        painter = QPainter(self)
        self.style().drawPrimitive(QStyle.PE_Widget, style_option, painter, self)

