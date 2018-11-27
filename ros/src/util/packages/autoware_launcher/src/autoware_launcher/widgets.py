from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets

import os



class AwBasicWindow(QtWidgets.QMainWindow):

    def __init__(self, guimgr, parent, config):

        super(AwBasicWindow, self).__init__(parent)
        self.guimgr = guimgr
        self.config = config
        self.widget = None

    def load_plugin(self):

        config = self.config
        window = self
        widget = QtWidgets.QWidget()

        window.setCentralWidget(widget)
        window.setWindowTitle(config.plugin.schema["text"])
        
        layout = QtWidgets.QVBoxLayout()
        for child in config.children.values():
            frame = self.guimgr.create_frame(self, child)
            layout.addWidget(frame)
        layout.addStretch()
        widget.setLayout(layout)

    def load_geometry(self):
        
        settings = QtCore.QSettings("Autoware", "AutowareLauncher")
        if settings.contains("geometry"):
            self.restoreGeometry(settings.value("geometry"))
 
    def save_geometry(self):

        print "Save geometry"
        settings = QtCore.QSettings("Autoware", "AutowareLauncher")
        settings.setValue("geometry", self.saveGeometry())



class AwBasicFrame(QtWidgets.QWidget):

    def __init__(self, guimgr, parent, config):

    	super(AwBasicFrame, self).__init__(parent)
        self.guimgr = guimgr
        self.config = config
        self.menu = None
        self.body = None

    def paintEvent(self, event):

        style_option = QtWidgets.QStyleOption()
        style_option.initFrom(self)
        painter = QtGui.QPainter(self)
        self.style().drawPrimitive(QtWidgets.QStyle.PE_Widget, style_option, painter, self)

    def set_widgets(self, header, detail):

        config = self.config
        widget = self

        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(header)
        layout.addWidget(detail)
        widget.setLayout(layout)

        header.setObjectName("FrameMenu")
        detail.setObjectName("FrameBody")
        widget.setObjectName("FrameWidget")
        widget.setStyleSheet("#FrameWidget { border: 1px solid; } #FrameMenu { padding: 3px; border-bottom: 1px solid; } #FrameBody { padding: 3px; }")

    def open_window(self):

        window = self.guimgr.create_window(self, self.config)
        window.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        window.setWindowModality(QtCore.Qt.ApplicationModal)
        window.show()



class AwBasicFrameHeader(QtWidgets.QWidget):

    def __init__(self):

    	super(AwBasicFrameHeader, self).__init__()

        self.__title = QtWidgets.QLabel()
        self.__title.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)

        self.__layout = QtWidgets.QHBoxLayout()
        self.__layout.addWidget(self.__title)
        self.setLayout(self.__layout)

    def paintEvent(self, event):

        style_option = QtWidgets.QStyleOption()
        style_option.initFrom(self)
        painter = QtGui.QPainter(self)
        self.style().drawPrimitive(QtWidgets.QStyle.PE_Widget, style_option, painter, self)

    def setHeaderTitle(self, text):

        self.__title.setText(text)

    def addHeaderButton(self, button):

        self.__layout.addWidget(button)



class AwRootWindow(AwBasicWindow):

    def __init__(self, guimgr, parent, config):

        super(AwRootWindow, self).__init__(guimgr, parent, config)
        self.load_plugin()
        self.load_geometry()

    def closeEvent(self, event):

        self.save_geometry()



class AwNodeFrame(AwBasicFrame):

    def __init__(self, guimgr, parent, config):

        super(AwNodeFrame, self).__init__(guimgr, parent, config)
        self.load_plugin()

    def load_plugin(self):

        header = QtWidgets.QLabel("Menu")
        header.setText(self.config.plugin.schema["text"])

        detail = QtWidgets.QPushButton("Config")
        detail.clicked.connect(self.open_window)

        self.set_widgets(header, detail)



class AwNodeWindow(AwBasicWindow):

    def __init__(self, guimgr, parent, config):

        super(AwNodeWindow, self).__init__(guimgr, parent, config)
        self.load_plugin()

        self.load_geometry()



class AwFileSelectFrame(AwBasicFrame):

    def __init__(self, guimgr, parent, config):

        super(AwFileSelectFrame, self).__init__(guimgr, parent, config)
        self.load_plugin()

    def load_plugin(self):

        button = QtWidgets.QPushButton("Browse")
        button.clicked.connect(self.browse_file)

        header = AwBasicFrameHeader()
        header.setHeaderTitle(self.config.plugin.schema["text"])
        header.addHeaderButton(button)

        detail = QtWidgets.QLabel("BODY")
    
        self.set_widgets(header, detail)

    def browse_file(self):

        #default_path = os.path.join( RosPack().get_path("autoware_launcher"), "profiles")
        filename, filetype =  QtWidgets.QFileDialog.getOpenFileName(self, "Select Profile", os.path.expanduser("~"))
        if filename:
            print filename



class AwFileSelectWindow(AwBasicWindow):

    def __init__(self, guimgr, parent, config):

        super(AwFileSelectWindow, self).__init__(guimgr, parent, config)
        self.load_plugin()

        self.load_geometry()
