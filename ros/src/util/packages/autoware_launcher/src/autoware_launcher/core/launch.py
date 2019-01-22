import os
import yaml

from . import basetree
from . import console
from . import fspath



class AwBaseNode(object):

    def __init__(self, name):
        self.__nodename = name
        self.__parent   = None
        self.__children = []
        self.__childmap = {}

    def dump(self, indent = 0):
        print((indent * " ") + str(self))
        for child in self.children(): child.dump(indent + 2)

    #def setleaf(self):
    #    self.__nodetype = False

    #def isnode(self):
    #    return self.__nodetype is True

    #def isleaf(self):
    #    return self.__nodetype is False

    def tree(self):
        return self.__parent.tree()

    def nodename(self):
        return self.__nodename

    def nodepath(self):
        return os.path.join(self.__parent.nodepath(), self.nodename())

    def fullpath(self):
        return os.path.join(self.__parent.fullpath(), self.nodename())

    def children(self):
        return self.__children

    def getchild(self, name):
        return self.__childmap.get(name)

    def haschild(self, name):
        return name in self.__childmap

    def addchild(self, node):
        self.__children.append(node)
        self.__childmap[node.nodename()] = node
        node.__parent = self

    def delchild(self, node):
        self.__children.remove(node)
        self.__childmap.pop(node.nodename())
        node.__parent = None

    def listnode(self, this = False):
        result = [self] if this else []
        for child in self.children(): result.extend(child.listnode(True))
        return result



class AwBaseTree(AwBaseNode):

    def __init__(self):
        super(AwBaseTree, self).__init__(None)
        self.treepath = ""

    def tree(self):
        return self

    def nodepath(self):
        return ""
    
    def fullpath(self):
        return self.treepath

    def find(self, path):
        node = self
        for name in path.split("/"):
            node = node.getchild(name)
        return node



class AwLaunchTree(AwBaseTree):

    def __init__(self, server, plugins):
        super(AwLaunchTree, self).__init__()
        self.server  = server
        self.plugins = plugins

    def __str__(self):
        childnames = map(lambda child: child.nodename(), self.children())
        return "Tree:{} Children:{}".format(self.nodename(), childnames)

    def save(self, treepath):
        self.treepath = treepath
        with open(treepath + ".launch", mode = "w") as fp:
            fp.write("dummy")
        for node in self.listnode():
            fullpath = node.fullpath() + ".yaml"
            fspath.makedirs(os.path.dirname(fullpath), exist_ok = True)
            with open(fullpath, mode = "w") as fp:
                fp.write(yaml.safe_dump(node.export_data(), default_flow_style = False))

    def load(self, treepath, plugins):
        def load_node(node):
            fullpath = node.fullpath()
            with open(fullpath + ".yaml") as fp:
                node.import_data(yaml.safe_load(fp), plugins)
            for child in node.children():
                load_node(child)
        root = AwLaunchNode("root")
        self.addchild(root)
        self.treepath = treepath
        load_node(root)

    def make(self, ppath, plugins):
        plugin = plugins.find(ppath)
        launch = AwLaunchNode("root")
        launch.plugin = plugin
        launch.config = plugin.default_config()
        self.addchild(launch)

    def create(self, lpath, ppath):
        print "Tree Create: " + lpath + ", " + ppath
        parent = self.find(os.path.dirname(lpath))
        if not parent:
            return "parent is not found"
        if self.find(lpath):
            return "name exists"

        plugin = self.plugins.find(ppath)
        launch = AwLaunchNode(os.path.basename(lpath))
        launch.plugin = plugin
        launch.config = plugin.default_config()
        parent.addchild(launch)
        return None




class AwLaunchNode(AwBaseNode):

    STOP, EXEC, TERM = 0x00, 0x01, 0x02

    def __init__(self, name):
        super(AwLaunchNode, self).__init__(name)
        self.plugin = None
        self.config = None
        self.status = self.STOP

    def tostring(self):
        data = {}
        data["plugin"]   = self.plugin.todict()
        data["config"]   = self.config
        data["children"] = map(lambda child: child.nodename(), self.children())
        return yaml.safe_dump(data)

    # experimental
    def remove_child(self, name):
        if not name:
            return "name is empty"
        if not self.haschild(name):
            return "name does not exist"
        self.delchild(name)
        self.send_config_removed(name)
        return None

    def update(self, ldata):
        self.config.update(ldata["config"])
        return None

    def launch(self, xmode):
        if xmode:
            return self.__exec()
        else:
            return self.__term()

    def __exec(self):
        if self.plugin.isleaf():
            if self.status == self.STOP:
                self.status = self.EXEC
                return (True, True)
        else:
            status = self.STOP
            for child in self.children():
                status |= child.status
            if self.status != status:
                self.status = status
                return (True, False)
        return (False, False)

    def __term(self):
        if self.plugin.isleaf():
            if self.status == self.EXEC:
                self.status = self.TERM
                return (True, True)
        else:
            status = self.STOP
            for child in self.children():
                status |= child.status
            if self.status != status:
                self.status = status
                return (True, False)
        return (False, False)

    def get_config(self, key, value):
        return self.config.get(key, value)

    def generate_launch(self): # ToDo: list args from plugin
        lines = []
        lines.append('<launch>')
        lines.append('  <include file="' + self.plugin.launch() + '">')
        for arg_name, arg_data in self.config.items():
            if arg_name.startswith("args."):
                if arg_data:
                    if type(arg_data) is list:
                        arg_data = " ".join(arg_data)
                    lines.append('    <arg name="' + arg_name[5:] + '" value="' + str(arg_data) + '"/>')
        lines.append('  </include>')
        lines.append('</launch>')
        return "\n".join(lines)

    def import_data(self, data, plugins):
        self.plugin = plugins.find(data["plugin"])
        self.config = data["config"]
        if data["children"] is None:
            self.setleaf()
        else:
            for childname in data["children"]:
                self.addchild(AwLaunchNode(childname))

    def export_data(self):
        children = map(lambda node: node.nodename(), self.children())
        plugin = self.plugin.path()
        config = self.config
        return { "children": children, "plugin": plugin, "config": config }


    # ToDo: remove function
    def bind_listener(self, listener):
        console.warning("bind_listener: " + listener.__class__.__name__)

    # ToDo: remove function
    def unbind_listener(self, listener):
        console.warning("unbind_listener: " + listener.__class__.__name__)



if __name__ == "__main__":
    from .plugin import AwPluginTree
    plugin = AwPluginTree()
    launch = AwLaunchTree(None)
    launch.load(fspath.profile("default"), plugin)
    launch.dump()
    launch.save(fspath.profile("sample.bak"))