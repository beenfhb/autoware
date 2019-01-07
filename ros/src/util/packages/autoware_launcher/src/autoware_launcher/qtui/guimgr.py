from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets

from autoware_launcher.core import console
from autoware_launcher.core import fspath

from .widgets2 import AwMainWindow
from .treeview import AwTreeViewPanel
from .treeview import AwControlPanel
from .procmgr  import AwProcessPanel
from .summary  import AwSummaryPanel



# ToDo: move package
#     core = client, mirror
#     qtui = guimgr

class AwQtGuiClient(object):

    def __init__(self, sysarg, server):
        self.__sysarg = sysarg
        self.__panels = []
        self.__guimgr = AwQtGuiManager(self)
        self.__mirror = AwLaunchTreeMirror(self)

        self.__server = server
        self.__server.register_client(self)

    def guimgr(self):
        return self.__guimgr

    def select_config(self, lpath): # ToDo: consider moving to guimgr
        self.__treeview.select_config(lpath)

    def start2(self):

        application = QtWidgets.QApplication(self.__sysarg)
        resolution = application.desktop().screenGeometry()
        resolution = min(resolution.width(), resolution.height())

        stylesheet = []
        stylesheet.append("#FrameHeader { border-top: 1px solid; } #FrameHeader, #FrameWidget { padding: 5px; border-bottom: 1px solid; border-left: 1px solid; border-right: 1px solid; }")
        stylesheet.append("* { font-size: " + str(resolution/100) + "px; }")
        application.setStyleSheet(" ".join(stylesheet))

        self.__treeview = AwTreeViewPanel(self) # ToDo: consider moving to guimgr
        self.__control  = AwControlPanel(self)  # ToDo: consider moving to guimgr
        self.__summary  = AwSummaryPanel(self)  # ToDo: consider moving to guimgr
        self.__process  = AwProcessPanel(self)  # ToDo: consider moving to guimgr

        tabwidget = QtWidgets.QTabWidget()
        tabwidget.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        tabwidget.addTab(self.__summary, "Summary")
        tabwidget.addTab(self.__process, "Process")

        #vsplitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        vsplitter = QtWidgets.QWidget()
        vsplitter.setLayout(QtWidgets.QVBoxLayout())
        vsplitter.layout().addWidget(self.__treeview)
        vsplitter.layout().addWidget(self.__control)

        hsplitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        hsplitter.addWidget(vsplitter)
        hsplitter.addWidget(tabwidget)

        window = AwMainWindow(self)
        window.setCentralWidget(hsplitter)
        window.show()

        self.__server.register_runner(self.__process)
        self.__process.register_server(self.__server)

        self.__panels.append(self.__treeview)
        self.__panels.append(self.__summary)
        self.__panels.append(self.__process)
        #self.__panels.append(self.__control)

        self.__treeview.register_select_listener(self.__summary)
        self.__treeview.register_select_listener(self.__process)
        self.__treeview.register_select_listener(self.__control)

        self.__server.make_profile("node/root")
        return application.exec_()

    def config_cleared(self):
        self.__mirror.clear()
        for panel in self.__panels: panel.config_cleared()

        for lpath in self.__server.list_config():
            lnode = self.__mirror.create(lpath)
            for panel in self.__panels: panel.config_created(lnode)

        self.__treeview.expandAll()
        #self.__treeview.expandToDepth(0)

    def config_created(self, lpath):
        print "config_created: " + lpath
        lnode = self.__mirror.create(lpath)
        for panel in self.__panels: panel.config_created(lnode)

        lpath = os.path.dirname(lpath)
        while lpath:
            print lpath
            self.__summary.config_updated(lpath)
            lpath = os.path.dirname(lpath)

    def config_updated(self, lpath):
        print "config_updated:" + lpath
        self.__mirror.clear(lpath)
        #for panel in self.__panels: panel.config_updated(lpath)

    def status_updated(self, lpath, state):
        print "config_updated:" + lpath + " " + str(state)
        self.__treeview.status_updated(lpath, state)











    def start(self):

        application = QtWidgets.QApplication(self.__sysarg)
        resolution = application.desktop().screenGeometry()
        resolution = min(resolution.width(), resolution.height())

        stylesheet = []
        stylesheet.append("#FrameHeader { border-top: 1px solid; } #FrameHeader, #FrameWidget { padding: 5px; border-bottom: 1px solid; border-left: 1px solid; border-right: 1px solid; }")
        stylesheet.append("* { font-size: " + str(resolution/100) + "px; }")
        application.setStyleSheet(" ".join(stylesheet))

        window = widgets.AwMainWindow(self, self.__mirror)
        window.show()
        window.process.init(self.__server) # ToDo: move to __init__
        return application.exec_()

    def save_profile(self, fpath):
        self.__server.save_profile(fpath)

    def load_profile(self, fpath):
        self.__server.load_profile(fpath)
        #if ok
        self.__mirror.clear()

    def find_node(self, lpath):
        return self.__server.find_node(lpath)

    def launch_config(self, lpath, xmode):
        return self.__server.launch_node(lpath, xmode)

    def create_node(self, lpath, ppath):
        return self.__server.create_node(lpath, ppath)

    def update_node(self, lpath, ldata):
        return self.__server.update_node(lpath, ldata)





import importlib
import os

class AwQtGuiManager(object):

    def __init__(self, client):
        self.__client  = client
        self.__widgets = {}

        for filepath in os.listdir(fspath.package("src/autoware_launcher/qtui/plugins")):
            fkey, fext = os.path.splitext(os.path.basename(filepath))
            if (fkey != "__init__") and (fext == ".py"):
                console.info("load plugin module: " + fkey)
                module = importlib.import_module("autoware_launcher.qtui.plugins." + fkey)
                for wkey, wcls in module.plugin_widgets().items():
                     self.__widgets[fkey + "." + wkey] = wcls

    #def widget(self, view):
    #    return self.__widgets[view["view"]]

    def client(self):
        return self.__client

    def create_widget(self, node, view, parent = None, widget = None):
        widget = widget or self.__widgets[view["view"]]
        print widget
        return widget(self, node, view)

    def create_frame(self, mirror, guikey = None, guicls = None):
        #print "Create Frame: {:<7} Key: {} Class: {}".format(mirror.nodename(), guikey, guicls)
        if not guicls:
            guikey = guikey or mirror.plugin().frame()
            guicls = self.__widgets[guikey + "_frame"]
        return guicls(self, mirror)

    def create_panel(self, mirror, guikey = None, guicls = None):
        #print "Create Panel: {:<7} Key: {} Class: {}".format(mirror.nodename(), guikey, guicls)
        if not guicls:
            guikey = guikey or mirror.plugin().panel()
            guicls = self.__widgets[guikey + "_panel"]
        return guicls(self, mirror)

    def create_arg_frame(self, parent, view):
        guicls = self.__widgets["args." + view["type"]]
        return guicls(self, parent, view)

    def create_frame_entire_vlayout(self):
        layout = QtWidgets.QVBoxLayout()
        layout.setSpacing(0)
        layout.setContentsMargins(0, 0, 0, 0)
        return layout

    def create_frame_header_hlayout(self):
        layout = QtWidgets.QHBoxLayout()
        layout.setSpacing(0)
        layout.setContentsMargins(5, 2, 2, 2)
        return layout



class AwLaunchTreeMirror(object):
    
    def __init__(self, client):
        self.nodes = {}
        self.cache = {}
        self.client = client

    def clear(self, lpath = None):
        if lpath:
            self.cache.pop(lpath, None)
        else:
            self.cache.clear()

    def find(self, path):
        if path not in self.cache:
            self.cache[path] = self.client.find_node(path)
        return self.cache[path]

    def create(self, path):
        #console.warning("create_node {}".format(path))
        if path not in self.nodes:
            self.nodes[path] = AwLaunchNodeMirror(self, path)
        return self.nodes[path]

    def remove(self, path, node):
        #console.warning("remove_node {}".format(path))
        self.nodes.pop(path)

    def status_updated(self, lpath, state):
        self.nodes[lpath].status_updated(state)



class AwLaunchNodeMirror(object):

    def __init__(self, tree, path):
        self.__tree = tree
        self.__path = path
        self.__refs = []

    def __find(self):
        return self.__tree.find(self.__path)

    def tostring(self):
        return self.__find().tostring()

    def status(self):
        node = self.__find()
        if node.status == node.STOP: return "stop"
        if node.status == node.EXEC: return "exec"
        if node.status == node.TERM: return "term"
        return "exec/term"

    def isleaf(self):
        return self.__find().plugin.isleaf()

    def path(self):
        return self.__find().nodepath()

    def name(self):
        return self.__find().nodename()
    
    def plugin(self):
        return self.__find().plugin

    def config(self):
        #return self.__find().config
        return self.__find().config.copy()

    def update(self, ldata):
        return self.__tree.client.update_node(self.__path, ldata)

    def launch(self, mode):
        self.__tree.client.launch_node(self.__path, mode)

    def listnode(self, this):
        return map(lambda node: node.nodepath(), self.__find().listnode(this))

    def haschild(self, name):
        return self.__find().haschild(name)

    def getchild(self, name):
        return self.__tree.create(self.__path + "/" + name)

    def addchild(self, lname, ppath):
        return self.__tree.client.create_node(self.__path + "/" + lname, ppath)

    def children(self):
        mirrored_children = []
        for child in self.__find().children():
            mirrored_children.append(self.__tree.create(child.nodepath()))
        return mirrored_children

    def updated(self):
        for widget in self.__refs:
            if hasattr(widget, "config_updated"): widget.config_updated()

    def status_updated(self, state):
        for widget in self.__refs:
            if hasattr(widget, "status_updated"): widget.status_updated(state)

    def get_config(self, key, value):
        return self.__find().config.get(key, value)

    def set_config(self, key, value):
        self.__find().config[key] = value

    def generate_launch(self):
        return self.__find().generate_launch()

    def send_term_completed(self):
        print "send_term_completed:" + self.__path