from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets

from autoware_launcher.core import console
from autoware_launcher.core import AwLaunchClientIF

from . import primitives
from . import widgets
from . import procmgr

from .widgets  import AwMainWindow
from .treeview import AwTreeViewPanel
from .treeview import AwControlPanel
from .procmgr  import AwProcessPanel
from .summary  import AwSummaryPanel



class AwQtGuiClient(AwLaunchClientIF):

    def __init__(self, sysarg, server):
        self.__sysarg = sysarg
        self.__server = server
        self.__panels = []
        self.__mirror = AwLaunchTreeMirror(self)

        self.__server.register_client(self)
        #self.__server.make_profile("root/autoware")
        #self.__server.load_profile("tmp_sensing")

        self.classes = {}
        self.classes["default_node.panel"] = widgets.AwDefaultNodePanel
        self.classes["default_node.frame"] = widgets.AwDefaultNodeFrame
        self.classes["default_leaf.panel"] = widgets.AwDefaultLeafPanel
        self.classes["default_leaf.frame"] = widgets.AwDefaultLeafFrame

        self.classes["args.text"]  = primitives.AwTextTypeFrame
        self.classes["args.file"]  = primitives.AwFileTypeFrame
        self.classes["args.tf"]    = primitives.AwTransformFrame
        self.classes["args.calib"] = primitives.AwCameraCalibFrame

    def start2(self):

        application = QtWidgets.QApplication(self.__sysarg)
        resolution = application.desktop().screenGeometry()
        resolution = min(resolution.width(), resolution.height())

        stylesheet = []
        stylesheet.append("#FrameHeader { border-top: 1px solid; } #FrameHeader, #FrameWidget { padding: 5px; border-bottom: 1px solid; border-left: 1px solid; border-right: 1px solid; }")
        stylesheet.append("* { font-size: " + str(resolution/100) + "px; }")
        application.setStyleSheet(" ".join(stylesheet))

        self.__treeview = AwTreeViewPanel(self)
        self.__control  = AwControlPanel(self)
        self.__summary  = AwSummaryPanel(self)
        self.__process  = AwProcessPanel(self)

        tabwidget = QtWidgets.QTabWidget()
        tabwidget.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        tabwidget.addTab(self.__summary, "Summary")
        tabwidget.addTab(self.__process, "Process")

        vsplitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        vsplitter.addWidget(tabwidget)
        vsplitter.addWidget(self.__control)

        hsplitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        hsplitter.addWidget(self.__treeview)
        hsplitter.addWidget(vsplitter)

        window = widgets.AwMainWindow(self)
        window.setCentralWidget(hsplitter)
        window.show()

        self.__server.register_runner(self.__process)
        self.__process.register_server(self.__server)

        self.__panels.append(self.__treeview)
        self.__panels.append(self.__summary)
        self.__panels.append(self.__process)

        self.__treeview.register_select_listener(self.__control)
        self.__treeview.register_select_listener(self.__summary)
        self.__treeview.register_select_listener(self.__process)

        self.__server.make_profile("node/sensing")
        return application.exec_()

    def config_cleared(self):
        self.__mirror.clear()
        for panel in self.__panels: panel.config_cleared()

        for lpath in self.__server.list_config():
            lnode = self.__mirror.create(lpath)
            for panel in self.__panels: panel.config_created(lnode)

        self.__treeview.expandToDepth(0)

    def config_updated(self):
        pass        

    def status_updated(self, lpath, state):
        print (lpath, state) 
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

    def create_widget(self, mirror, parent, guicls = None):
        return guicls(self, mirror, parent)

    def create_frame(self, mirror, guikey = None, guicls = None):
        #print "Create Frame: {:<7} Key: {} Class: {}".format(mirror.nodename(), guikey, guicls)
        if not guicls:
            guikey = guikey or mirror.plugin().gui["type"]
            guicls = self.classes[guikey + ".frame"]
        return guicls(self, mirror)

    def create_panel(self, mirror, guikey = None, guicls = None):
        #print "Create Panel: {:<7} Key: {} Class: {}".format(mirror.nodename(), guikey, guicls)
        if not guicls:
            guikey = guikey or mirror.plugin().gui["type"]
            guicls = self.classes[guikey + ".panel"]
        return guicls(self, mirror)

    def create_arg_frame(self, parent, view):
        guicls = self.classes[view.guikey]
        return guicls(self, parent, view.option)

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

    def node_updated(self, lpath):
        self.__mirror.updated(lpath)


    #def node_patched



class AwAwQtGuiManager(object):
    pass




class AwLaunchTreeMirror(object):
    
    def __init__(self, client):
        self.nodes = {}
        self.cache = {}
        self.client = client

    def clear(self):
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

    def updated(self, path):
        self.nodes[path].updated()

    def status_updated(self, lpath, state):
        self.nodes[lpath].status_updated(state)



class AwLaunchNodeMirror(object):

    def __init__(self, tree, path):
        self.__tree = tree
        self.__path = path
        self.__refs = []

    def __find(self):
        return self.__tree.find(self.__path)

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

    def getchild(self, name):
        return self.__tree.create(self.__path + "/" + name)

    def addchild(self, lname, ppath):
        return self.__tree.client.create_node(self.__path + "/" + lname, ppath)

    def children(self):
        mirrored_children = []
        for child in self.__find().children():
            mirrored_children.append(self.__tree.create(child.nodepath()))
        return mirrored_children

    def bind(self, item):
        print "bind: {} {}".format(self.__path, str(item))
        self.__refs.append(item)

    def unbind(self, item):
        print "unbind: {} {}".format(self.__path, str(item))
        self.__refs.remove(item)

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