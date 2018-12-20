from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets

from autoware_launcher.core import console
from autoware_launcher.core import AwLaunchClientIF

from . import primitives
from . import widgets
from . import procmgr



class AwQtGuiClient(AwLaunchClientIF):

    def __init__(self, sysarg, server):
        self.__sysarg = sysarg
        self.__server = server
        self.__mirror = AwLaunchTreeMirror(self)
        self.elements = {}

        self.__server.register_client(self)
        #self.server.request_launch_load("test")
        #self.server.request_launch_make("node/root")
        #self.__server.make_profile("node/sensing")
        self.__server.load_profile("tmp_sensing")

        self.panels = {}
        self.frames = {}
        self.panels["default_node"] = widgets.AwDefaultNodePanel
        self.frames["default_node"] = widgets.AwDefaultNodeFrame
        self.panels["default_leaf"] = widgets.AwDefaultLeafPanel
        self.frames["default_leaf"] = widgets.AwDefaultLeafFrame

        self.frames["args.text"]  = primitives.AwTextTypeFrame
        self.frames["args.file"]  = primitives.AwFileTypeFrame
        self.frames["args.tf"]    = primitives.AwTransformFrame
        self.frames["args.calib"] = primitives.AwCameraCalibFrame

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
            guicls = self.frames[guikey]
        return guicls(self, mirror)

    def create_panel(self, mirror, guikey = None, guicls = None):
        #print "Create Panel: {:<7} Key: {} Class: {}".format(mirror.nodename(), guikey, guicls)
        if not guicls:
            guikey = guikey or mirror.plugin().gui["type"]
            guicls = self.panels[guikey]
        return guicls(self, mirror)

    def create_arg_frame(self, parent, view):
        guicls = self.frames[view.guikey]
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

    def exec_node(self, lpath):
        return self.__server.exec_node(lpath)

    def term_node(self, lpath):
        return self.__server.term_node(lpath)

    def launch_node(self, lpath, xmode):
        return self.__server.launch_node(lpath, xmode)

    def create_node(self, lpath, ppath):
        return self.__server.create_node(lpath, ppath)

    def update_node(self, lpath, ldata):
        return self.__server.update_node(lpath, ldata)

    def node_updated(self, lpath):
        self.__mirror.updated(lpath)

    def status_updated(self, lpath, state):
        self.__mirror.status_updated(lpath, state)


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

    def bind(self, widget):
        print "bind: {} {}".format(self.__path, str(widget))
        self.__refs.append(widget)

    def unbind(self, widget):
        print "unbind: {} {}".format(self.__path, str(widget))
        self.__refs.remove(widget)

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