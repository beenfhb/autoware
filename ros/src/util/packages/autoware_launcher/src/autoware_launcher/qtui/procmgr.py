from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets

from autoware_launcher.core import AwLaunchNodeListenerIF
from autoware_launcher.core import AwLaunchNodeExecutorIF
from .network import AwTcpServer

class AwLaunchWidgetItem(QtWidgets.QTreeWidgetItem, AwLaunchNodeListenerIF):

    def __init__(self, node, area):
        super(AwLaunchWidgetItem, self).__init__()
        self.user = True
        self.area = area
        self.node = node
        self.node.bind_listener(self)

        self.setText(0, self.node.nodename())
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



class AwLaunchExecutor(QtWidgets.QPlainTextEdit, AwLaunchNodeExecutorIF):

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

    def __init__(self, guimgr, tree):
        super(AwLaunchWidget, self).__init__(QtCore.Qt.Horizontal)
        self.tcpserver = AwTcpServer(tree)
        self.dummyarea = QtWidgets.QLabel()
        self.executors = QtWidgets.QStackedWidget()
        self.executors.addWidget(self.dummyarea)

        view = QtWidgets.QTreeWidget()
        view.setColumnCount(3)
        view.setHeaderLabels(["Node", "Exec", "Status"])
        view.header().setStretchLastSection(False)
        view.header().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        view.header().setSectionResizeMode(1, QtWidgets.QHeaderView.Fixed)
        view.header().setSectionResizeMode(2, QtWidgets.QHeaderView.Fixed)

        for node in tree.children():
            view.addTopLevelItem(self.construct(node))
        view.expandToDepth(0)
        view.itemChanged.connect(self.on_item_changed)
        view.itemClicked.connect(self.on_item_clicked)
        view.itemActivated.connect(self.on_item_clicked)
        #view.selectionModel().selectionChanged

        self.addWidget(view)
        self.addWidget(self.executors)

    def construct(self, node):
        area = self.dummyarea
        if node.isleaf():
            area = AwLaunchExecutor(node)
            self.executors.addWidget(area)
        item = AwLaunchWidgetItem(node, area)
        for child in node.children():
            item.addChild(self.construct(child))
        return item

    # QtCore.Slot
    def on_item_changed(self, item, column):
        item.changed(column)

    # QtCore.Slot
    def on_item_clicked(self, item, column):
        self.executors.setCurrentWidget(item.area)
