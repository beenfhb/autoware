import hashlib
from autoware_launcher.core import fspath

from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets



class AwRunnerPanel(QtWidgets.QSplitter):

    def __init__(self, guimgr, mirror):
        super(AwRunnerPanel, self).__init__(QtCore.Qt.Horizontal)
        self.guimgr = guimgr
        self.mirror = mirror
        self.server = None

        self.exec_dummy = QtWidgets.QLabel("This is node")
        self.exec_items = {}
        self.exec_panel = QtWidgets.QStackedWidget()
        self.exec_panel.addWidget(self.exec_dummy)
        self.exec_panel_setup()

        self.tree_items = {}
        self.tree_panel = QtWidgets.QTreeWidget()
        self.tree_panel.setColumnCount(3)
        self.tree_panel.setHeaderLabels(["Node", "Exec", "Status"])
        self.tree_panel.header().setStretchLastSection(False)
        self.tree_panel.header().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self.tree_panel.header().setSectionResizeMode(1, QtWidgets.QHeaderView.Fixed)
        self.tree_panel.header().setSectionResizeMode(2, QtWidgets.QHeaderView.Fixed)
        self.tree_panel_setup()
        self.tree_panel.expandToDepth(0)
        self.tree_panel.itemChanged.connect(self.tree_panel_item_changed)
        self.tree_panel.currentItemChanged.connect(self.tree_panel_item_selectd)

        self.addWidget(self.tree_panel)
        self.addWidget(self.exec_panel)

    def init(self, server):
        self.server = server
        self.server.register_runner(self)

    def exec_panel_setup(self):
        root = self.mirror.create("root")
        for path in root.listnode(True):
            node = self.mirror.create(path)
            if node.isleaf():
                item = AwRunnerTerminal(self, node)
                self.exec_items[path] = item
                self.exec_panel.addWidget(item)

    def tree_panel_setup(self):
        import os
        root = self.mirror.create("root")
        item = AwRunnerTreeItem(root)
        self.tree_items["root"] = item
        self.tree_panel.addTopLevelItem(item)
        for path in root.listnode(False):
            base = os.path.dirname(path)
            node = self.mirror.create(path)
            item = AwRunnerTreeItem(node)
            self.tree_items[path] = item
            self.tree_items[base].addChild(item)

    def tree_panel_item_changed(self, item, column):
        item.changed(column)

    def tree_panel_item_selectd(self, item, prev):
        terminal = self.exec_items.get(item.mirror.path(), self.exec_dummy)
        self.exec_panel.setCurrentWidget(terminal)

    def roslaunch(self, lpath, xtext):
        xhash = hashlib.md5(lpath).hexdigest()
        xpath = fspath.package() + "/runner/" + xhash + ".xml"
        with open(xpath, mode="w") as fp:
            fp.write(xtext)
        print "roslaunch {}".format(xpath)
        self.exec_items[lpath].proc.start("roslaunch {}".format(xpath))

    def terminate(self, lpath):
        self.exec_items[lpath].proc.terminate()


class AwRunnerTreeItem(QtWidgets.QTreeWidgetItem):

    def __init__(self, mirror):
        super(AwRunnerTreeItem, self).__init__()
        self.ignore = False
        self.mirror = mirror

        self.mirror.bind(self)
        #self.destroyed.connect(self.unbind)

        self.setText(0, self.mirror.name())
        self.setText(1, "")
        self.setData(1, QtCore.Qt.CheckStateRole, QtCore.Qt.Unchecked)
        self.setText(2, self.mirror.status())

    def changed(self, column):
        if column == 1:
            if self.ignore:
                self.ignore = False
            else:
                state = self.checkState(column)
                if state == QtCore.Qt.Checked:
                    self.mirror.launch(True)
                elif state == QtCore.Qt.Unchecked:
                    self.mirror.launch(False)

    def status_updated(self, state):
        print "AwRunnerTreeItem Mirror: " + self.mirror.path()
        self.setText(2, self.mirror.status())

    def exec_requested(self):
        self.setText(2, self.mirror.status())
        if self.checkState(1) != QtCore.Qt.Checked:
            self.user = False
            self.setCheckState(1,  QtCore.Qt.Checked)

    def term_requested(self):
        self.setText(2, self.mirror.status())
        if self.checkState(1) != QtCore.Qt.Unchecked:
            self.user = False
            self.setCheckState(1,  QtCore.Qt.Unchecked)

    def term_completed(self):
        self.setText(2, self.mirror.status())
        if self.checkState(1) != QtCore.Qt.Unchecked:
            self.user = False
            self.setCheckState(1,  QtCore.Qt.Unchecked)



class AwRunnerTerminal(QtWidgets.QPlainTextEdit):

    def __init__(self, runner, mirror):
        super(AwRunnerTerminal, self).__init__()
        self.runner = runner
        self.status = "stop"
        self.mirror = mirror # ToDo: remove

        self.setReadOnly(True)
        self.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)

        self.proc = QtCore.QProcess(self)
        self.proc.finished.connect(self.proc_finished)
        self.proc.readyReadStandardOutput.connect(self.proc_stdouted)
        self.proc.readyReadStandardError.connect (self.proc_stderred)

        import re
        self.bash_regex = re.compile("\033(\[.*?m|\].*?;)")

    # QtCore.Slot
    def proc_finished(self):
        print "finished: " + self.mirror.path()
        self.runner.server.runner_finished(self.mirror.path())

    # QtCore.Slot
    def proc_stdouted(self):
        byte = self.proc.readAllStandardOutput()
        text = QtCore.QTextStream(byte).readAll()
        text = self.bash_regex.sub("", text)
        self.moveCursor(QtGui.QTextCursor.End)
        self.insertPlainText(text)
        self.moveCursor(QtGui.QTextCursor.End)

    # QtCore.Slot
    def proc_stderred(self):
        byte = self.proc.readAllStandardError()
        text = QtCore.QTextStream(byte).readAll()
        text = self.bash_regex.sub("", text)
        self.moveCursor(QtGui.QTextCursor.End)
        self.insertPlainText(text)
        self.moveCursor(QtGui.QTextCursor.End)
