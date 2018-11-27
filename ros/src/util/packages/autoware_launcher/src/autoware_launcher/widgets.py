from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets

class AwBasicWindow(QtWidgets.QMainWindow):

    def __init__(self, guimgr, parent, awnode):

        super(AwBasicWindow, self).__init__(parent)
        self.guimgr = guimgr
        self.awnode = awnode
        self.widget = None

    def load_plugin(self):

        awnode = self.awnode
        window = self
        widget = QtWidgets.QWidget()

        window.setCentralWidget(widget)
        window.setWindowTitle(awnode.plugin["text"])
        
        layout = QtWidgets.QVBoxLayout()
        for child in awnode.children.values():
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

    def __init__(self, guimgr, parent, awnode):

    	super(AwBasicFrame, self).__init__(parent)
        self.guimgr = guimgr
        self.awnode = awnode
        self.menu = None
        self.body = None

    def paintEvent(self, event):

        style_option = QtWidgets.QStyleOption()
        style_option.initFrom(self)
        painter = QtGui.QPainter(self)
        self.style().drawPrimitive(QtWidgets.QStyle.PE_Widget, style_option, painter, self)

    def set_widgets(self, header, detail):

        awnode = self.awnode
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

        window = self.guimgr.create_window(self, self.awnode)
        window.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        window.setWindowModality(QtCore.Qt.ApplicationModal)
        window.show()



class AwBasicFrameMenu(QtWidgets.QWidget):

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

