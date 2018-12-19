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
        #self.__server.make_profile("node/lidar")
        self.__server.load_profile("tmp_lidar")

        self.panels = {}
        self.frames = {}
        self.panels["default_node"] = widgets.AwDefaultNodePanel
        self.frames["default_node"] = widgets.AwDefaultNodeFrame
        self.panels["default_leaf"] = widgets.AwDefaultLeafPanel
        self.frames["default_leaf"] = widgets.AwDefaultLeafFrame

        self.frames["str_type"] = primitives.AwStrTypeFrame
        self.frames["int_type"] = primitives.AwIntTypeFrame
        self.frames["real_type"] = primitives.AwRealTypeFrame
        self.frames["file_select"] = primitives.AwFileSelectFrame


    #def element_created(self, path, listener):
    #    console.info("element_created {} {}".format(path, listener))
    #    if not self.elements.get(path): self.elements[path] = []
    #    self.elements[path].append(listener)

    #def element_deleted(self, path, listener):
    #    console.info("element_deleted {} {}".format(path, listener))
    #    self.elements[path].remove(listener)
    #    if not self.elements.get(path): self.elements.pop(path)

    def start(self):

        application = QtWidgets.QApplication(self.__sysarg)
        resolution = application.desktop().screenGeometry()
        resolution = min(resolution.width(), resolution.height())

        stylesheet = []
        stylesheet.append("#FrameHeader { border-top: 1px solid; } #FrameHeader, #FrameWidget { padding: 5px; border-bottom: 1px solid; border-left: 1px solid; border-right: 1px solid; }")
        stylesheet.append("* { font-size: " + str(resolution/100) + "px; }")
        application.setStyleSheet(" ".join(stylesheet))

        mirror = self.__mirror.create("root")
        window = QtWidgets.QMainWindow()
        window = widgets.AwMainWindow(self, mirror)
        window.show()
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

    def create_arg_frame(self, parent, argdef):
        guikey = argdef["type"] + "_type"
        guicls = self.frames[guikey]
        return guicls(self, parent, argdef)

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
        console.info("create_node {}".format(path))
        node = AwLaunchNodeMirror(self, path)
        if not self.nodes.get(path): self.nodes[path] = []
        self.nodes[path].append(node)
        return node

    def remove(self, path, node):
        console.info("remove_node {}".format(path))
        self.nodes[path].remove(node)
        if not self.nodes.get(path): self.nodes.pop(path)

    def updated(self, path):
        for node in self.nodes[path]: node.updated()


class AwLaunchNodeMirror(object):

    def __init__(self, tree, path):
        self.__tree = tree
        self.__path = path
        self.__widget = None

    def __find(self):
        return self.__tree.find(self.__path)

    def nodename(self):
        return self.__find().nodename()

    def isleaf(self):
        return self.__find().plugin.isleaf()

    def plugin(self):
        return self.__find().plugin

    def config(self):
        #return self.__find().config
        return self.__find().config.copy()

    def update(self, ldata):
        return self.__tree.client.update_node(self.__path, ldata)

    def get_config(self, key, value):
        return self.__find().config.get(key, value)

    def set_config(self, key, value):
        self.__find().config[key] = value

    def nodename(self):
        return self.__find().nodename()

    def getchild(self, name):
        return self.__tree.create(self.__path + "/" + name)

    def addchild(self, lname, ppath):
        return self.__tree.client.create_node(self.__path + "/" + lname, ppath)

    def children(self):
        mirrored_children = []
        for child in self.__find().children():
            mirrored_children.append(self.__tree.create(child.nodepath()))
        return mirrored_children


    def connect(self, widget): # ToDo: move to __init__
        self.__widget = widget
        self.__widget.destroyed.connect(self.destroy)

    def destroy(self):
        self.__tree.remove(self.__path, self)

    def updated(self):
        if hasattr(self.__widget, "mirror_updated"): self.__widget.mirror_updated()

    def patched(self, diff):
        if hasattr(self.__widget, "mirror_patched"): self.__widget.mirror_patched(diff)



    def request_exec(self):
        self.__tree.client.exec_node(self.__path)

    def request_term(self):
        self.__tree.client.term_node(self.__path)

    def bind_listener(self, listener):
        return self.__find().bind_listener(listener)

    def unbind_listener(self, listener):
        return self.__find().unbind_listener(listener)

    def bind_executor(self, executor):
        return self.__find().bind_executor(executor)

    def unbind_executor(self, executor):
        return self.__find().unbind_executor(executor)

    def generate_launch(self):
        return self.__find().generate_launch()

    def send_term_completed(self):
        print "send_term_completed:" + self.__path