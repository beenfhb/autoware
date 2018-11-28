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

    def load_basic_gui(self):

        class_name = " ( " + self.__class__.__name__ + " )"
        self.setWindowTitle(self.config.plugin.gui["text"] + class_name)

    def load_basic_geometry(self):
        
        print "Load geometry"
        settings = QtCore.QSettings("Autoware", "AutowareLauncher")
        if settings.contains("geometry"):
            self.restoreGeometry(settings.value("geometry"))
 
    def save_basic_geometry(self):

        print "Save geometry"
        settings = QtCore.QSettings("Autoware", "AutowareLauncher")
        settings.setValue("geometry", self.saveGeometry())



class AwBasicFrame(QtWidgets.QWidget):

    window_closed = QtCore.Signal()

    def __init__(self, guimgr, parent, config):

    	super(AwBasicFrame, self).__init__(parent)
        self.guimgr = guimgr
        self.config = config
        #self.menu = None
        #self.body = None

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

    @QtCore.Slot()
    def open_window(self):

        window = self.guimgr.create_window(self, self.config)
        window.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        window.setWindowModality(QtCore.Qt.ApplicationModal)
        window.destroyed.connect(lambda: self.window_closed.emit())
        window.show()


class AwBasicFrameHeader(QtWidgets.QWidget):

    def __init__(self):

    	super(AwBasicFrameHeader, self).__init__()

        self.__title = QtWidgets.QLabel()
        self.__title.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)

        self.__layout = QtWidgets.QHBoxLayout()
        self.__layout.setContentsMargins(5, 2, 2, 2)
        self.__layout.setSpacing(0)
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



class AwNodeFrame(AwBasicFrame):

    def __init__(self, guimgr, parent, config):

        super(AwNodeFrame, self).__init__(guimgr, parent, config)
        self.load_plugin()

    def load_plugin(self):

        config = QtWidgets.QPushButton("Config")
        config.clicked.connect(self.open_window)

        header = AwBasicFrameHeader()
        header.setHeaderTitle(self.config.plugin.gui["text"])
        header.addHeaderButton(config)

        detail = QtWidgets.QLabel("Data")

        self.set_widgets(header, detail)



class AwNodeWindow(AwBasicWindow):

    def __init__(self, guimgr, parent, config):

        super(AwNodeWindow, self).__init__(guimgr, parent, config)
        self.load_plugin()
        self.load_basic_gui()
        self.load_basic_geometry()
        
    def closeEvent(self, event):

        if self.config.plugin.gui.get("type") == "root":
            self.save_basic_geometry()

    def load_plugin(self):

        config = self.config
        window = self
        widget = QtWidgets.QWidget()

        window.setCentralWidget(widget)
        
        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)
        for child in config.children.values():
            frame = self.guimgr.create_frame(self, child)
            layout.addWidget(frame)
        layout.addStretch()
        widget.setLayout(layout)



class AwLeafFrame(AwBasicFrame):

    def __init__(self, guimgr, parent, config):

        super(AwLeafFrame, self).__init__(guimgr, parent, config)
        self.load_plugin()

    def load_plugin(self):

        config = QtWidgets.QPushButton("Config")
        config.clicked.connect(self.open_window)
        self.window_closed.connect(self.update_config)

        header = AwBasicFrameHeader()
        header.setHeaderTitle(self.config.plugin.gui["text"])
        header.addHeaderButton(config)

        self.detail = QtWidgets.QLabel("__NO_DATA__")

        self.set_widgets(header, self.detail)

    def update_config(self):

        self.detail.setText(str(self.config.data))


class AwLeafWindow(AwBasicWindow):

    def __init__(self, guimgr, parent, config):

        super(AwLeafWindow, self).__init__(guimgr, parent, config)
        self.load_basic_gui()
        self.load_basic_geometry()

        window = self
        widget = QtWidgets.QWidget()

        window.setCentralWidget(widget)
        
        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)
        for child in config.plugin.data:
            frame = self.guimgr.create_data_frame(self, child)
            #frame = self.guimgr.create_class(self, arg_info, AwTextEditFrame)
            layout.addWidget(frame)
        layout.addStretch()
        widget.setLayout(layout)



class AwTextEditFrame(AwBasicFrame):

    def __init__(self, guimgr, parent, config_plugin):

        super(AwTextEditFrame, self).__init__(guimgr, parent, None)

        header = AwBasicFrameHeader()
        header.setHeaderTitle("Arg: " + config_plugin.name)

        detail = QtWidgets.QTextEdit()

        self.set_widgets(header, detail)


class AwFileSelectFrame(AwBasicFrame):

    def __init__(self, guimgr, parent, config_plugin):

        super(AwFileSelectFrame, self).__init__(guimgr, parent, None)

        button = QtWidgets.QPushButton("Browse")

        header = AwBasicFrameHeader()
        header.setHeaderTitle("Arg: " + config_plugin.name)
        header.addHeaderButton(button)

        if not config_plugin.gui.get("list", False):
            self.detail = QtWidgets.QLineEdit()
            self.detail.setReadOnly(True)
            button.clicked.connect(self.browse_file)
        else:
            self.detail = QtWidgets.QTextEdit()
            self.detail.setReadOnly(True)
            button.clicked.connect(self.browse_file_list)
    
        self.set_widgets(header, self.detail)

    def browse_file(self):

        #default_path = os.path.join( RosPack().get_path("autoware_launcher"), "profiles")
        filename, filetype =  QtWidgets.QFileDialog.getOpenFileName(self, "Select Profile", os.path.expanduser("~"))
        if filename:
            self.detail.setText(filename)



class AwFileSelectWindow(AwBasicWindow):

    def __init__(self, guimgr, parent, config):

        super(AwFileSelectWindow, self).__init__(guimgr, parent, config)
        self.load_plugin()
        self.load_basic_geometry()
