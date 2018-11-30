from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets

from autoware_launcher.server import AwQtTcpServer



class AwLaunchWidgetItem(QtWidgets.QTreeWidgetItem):

    def __init__(self, node, area):
        super(AwLaunchWidgetItem, self).__init__()
        self.user = True
        self.area = area
        self.node = node
        self.node.bind_viewitem(self)

        self.setText(0, self.node.nodename)
        self.setText(1, "")
        self.setData(1, QtCore.Qt.CheckStateRole, QtCore.Qt.Unchecked)
        self.setText(2, "stop")

    def changed(self, column):
        if column == 1:
            state = self.checkState(column)
            if state == QtCore.Qt.Checked:
                self.request_exec()
            elif state == QtCore.Qt.Unchecked:
                self.request_term()

    def request_exec(self):
        if self.user:
            self.node.request_exec()
        else:
            self.user = True

    def request_term(self):
        if self.user:
            self.node.request_term()
        else:
            self.user = True

    def exec_requested(self):
        self.setText(2, "running")
        if self.checkState(1) != QtCore.Qt.Checked:
            self.user = False
            self.setCheckState(1,  QtCore.Qt.Checked)

    def term_requested(self):
        self.setText(2, "terminating")
        if self.checkState(1) != QtCore.Qt.Unchecked:
            self.user = False
            self.setCheckState(1,  QtCore.Qt.Unchecked)

    def term_completed(self):
        self.setText(2, "stop")
        if self.checkState(1) != QtCore.Qt.Unchecked:
            self.user = False
            self.setCheckState(1,  QtCore.Qt.Unchecked)



class AwLaunchExecutor(QtWidgets.QPlainTextEdit):

    STOP_STATE = 1
    EXEC_STATE = 2
    TERM_STATE = 3

    def __init__(self, node):
        super(AwLaunchExecutor, self).__init__()
        self.status = AwLaunchExecutor.STOP_STATE
        self.node = node
        self.node.bind_executor(self)

        self.setReadOnly(True)
        self.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)

        self.proc = QtCore.QProcess(self)
        self.proc.finished.connect(self.on_finished)
        self.proc.readyReadStandardOutput.connect(self.on_ready_stdout)
        self.proc.readyReadStandardError.connect(self.on_ready_stderr)

    # ToDo: add state check
    def request_exec(self):
        self.status = AwLaunchExecutor.EXEC_STATE
        command = "roslaunch " + self.node.generate_launch()
        #print "Execute: " + command
        self.proc.start(command)

    # ToDo: add state check
    def request_term(self):
        self.status = AwLaunchExecutor.TERM_STATE
        self.proc.terminate()

    # ToDo: add state check
    # QtCore.Slot
    def on_finished(self):
        self.status = AwLaunchExecutor.STOP_STATE
        self.node.send_term_completed()

    # QtCore.Slot
    def on_ready_stdout(self):
        byte = self.proc.readAllStandardOutput()
        text = QtCore.QTextStream(byte).readAll()
        self.moveCursor(QtGui.QTextCursor.End)
        self.insertPlainText(text)

    # QtCore.Slot
    def on_ready_stderr(self):
        byte = self.proc.readAllStandardError()
        text = QtCore.QTextStream(byte).readAll()
        self.moveCursor(QtGui.QTextCursor.End)
        self.insertPlainText(text)


class AwLaunchWidget(QtWidgets.QSplitter):

    def __init__(self, tree):
        super(AwLaunchWidget, self).__init__(QtCore.Qt.Horizontal)
        self.tcpserver = AwQtTcpServer(tree)
        self.dummyarea = QtWidgets.QLabel()
        self.executors = QtWidgets.QStackedWidget()
        self.executors.addWidget(self.dummyarea)

        view = QtWidgets.QTreeWidget()
        view.setColumnCount(2)
        view.setHeaderLabels(["Node", "Exec", "Status"])
        for node in tree.children:
            view.addTopLevelItem(self.construct(node))
        view.expandToDepth(0)
        view.itemChanged.connect(self.on_item_changed)
        view.itemClicked.connect(self.on_item_clicked)
        view.itemActivated.connect(self.on_item_clicked)

        self.addWidget(view)
        self.addWidget(self.executors)

    def construct(self, node):
        area = self.dummyarea
        if node.isleaf():
            area = AwLaunchExecutor(node)
            self.executors.addWidget(area)
        item = AwLaunchWidgetItem(node, area)
        for child in node.children:
            item.addChild(self.construct(child))
        return item

    # QtCore.Slot
    def on_item_changed(self, item, column):
        item.changed(column)

    # QtCore.Slot
    def on_item_clicked(self, item, column):
        self.executors.setCurrentWidget(item.area)



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
