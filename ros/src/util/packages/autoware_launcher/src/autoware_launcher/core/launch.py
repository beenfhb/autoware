#import collections
#import glob
import json
import os
import yaml

from . import console
from . import fspath

class AwLaunchNodeListenerIF(object):
    def exec_requested(self): pass  #raise NotImplementedError()
    def term_requested(self): pass  #raise NotImplementedError()
    def term_completed(self): pass  #raise NotImplementedError()
    def config_updated(self): pass  #raise NotImplementedError()
    def config_created(self, node): pass  #raise NotImplementedError()
    def config_removed(self, name): pass  #raise NotImplementedError()
    def config_swapped(self): pass  #raise NotImplementedError()

class AwLaunchNodeExecutorIF(object):
    def request_exec(self): pass  #raise NotImplementedError()
    def request_term(self): pass  #raise NotImplementedError()



class AwBaseNode(object):

    def __init__(self, name):
        self.__nodename = name
        #self.__nodetype = True
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
        return self.__childmap[name]

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

    def nodelist(self, this = False):
        result = [self] if this else []
        for child in self.children(): result.extend(child.nodelist(True))
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
        for node in self.nodelist():
            fullpath = node.fullpath() + ".yaml"
            fspath.makedirs(os.path.dirname(fullpath), exist_ok = True)
            with open(fullpath, mode = "w") as fp:
                fp.write(yaml.safe_dump(node.export_data(), default_flow_style = False))

    def load(self, treepath, plugins):
        def load_node(node):
            fullpath = node.fullpath()
            print fullpath
            with open(fullpath + ".yaml") as fp:
                node.import_data(yaml.safe_load(fp), plugins)
            for child in node.children():
                load_node(child)
        root = AwLaunchNode("root")
        self.addchild(root)
        self.treepath = treepath
        load_node(root)

    def make(self, ppath, plugins):
        def make_node(parent, lname, ppath):
            plugin = plugins.find(ppath)
            launch = AwLaunchNode(lname)
            launch.plugin = plugin
            launch.config = plugin.default_config()
            parent.addchild(launch)

            if plugin.children():
                for cinfo in plugin.children():
                    if cinfo["type"] == "single":
                        make_node(launch, cinfo["name"], cinfo["plugin"])
            else:
                launch._AwBaseNode__nodetype = False
        make_node(self, "root", ppath)

    def create(self, lpath, ppath):
        if not os.path.basename(lpath):
            return "name empty"
        #if exists
        #    console.warning("name exists")
        #    return  

        parent = self.find(os.path.dirname(lpath))
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
        self.status = AwLaunchNode.STOP
        self.executor = AwLaunchNodeExecutorIF()

    def __str__(self):
        nodetype = "Node" if self.isnode() else "Leaf"
        plugin = None if not self.plugin else self.plugin.path() 
        config = None if not self.config else self.config.keys()
        childnames = map(lambda child: child.nodename(), self.children())
        return "{}:{:<13} Plugin:{} Config:{} Children:{}".format(nodetype, self.nodename(), plugin, config, childnames)
    
    def bind_executor(self, executor):
        if not isinstance(executor, AwLaunchNodeExecutorIF):
            raise TypeError(executor.__class__.__name__ + " does not inherit to AwLaunchNodeExecutorIF")
        self.executor = executor

    def unbind_executor(self, executor):
        self.executor = AwLaunchNodeExecutor()

    # experimental
    def remove_child(self, name):
        if not name:
            return "name is empty"
        if not self.haschild(name):
            return "name does not exist"
        self.delchild(name)
        self.send_config_removed(name)
        return None

    #def pull_config(self):
    #def push_config(self, config):

    def update(self, ldata):
        self.config = ldata["config"]
        return {"error": None}

    def get_config(self, key, value):
        return self.config.get(key, value)

    def set_config(self, key, value):
        self.config[key] = value
        self.send_config_updated()

    def request_exec(self):
        if self.plugin.isnode():
            self.send_exec_requested()
            for child_node in self.children():
                child_node.request_exec()
        else:
            self.send_exec_requested()
            self.executor.request_exec()

    def request_term(self):
        if self.isnode():
            self.send_term_completed()
            for child_node in self.children():
                child_node.request_term()
        else:
            self.send_term_requested()
            self.executor.request_term()

    # ToDo: apply state check of executor
    def send_exec_requested(self):
        for item in self.listener: item.exec_requested()

    # ToDo: apply state check of executor
    def send_term_requested(self):
        for item in self.listener: item.term_requested()

    # ToDo: apply state check of executor
    def send_term_completed(self):
        for item in self.listener: item.term_completed()

    def send_config_updated(self):
        for item in self.listener: item.config_updated()

    def send_config_created(self, node):
        for item in self.listener: item.config_created(node)

    def send_config_removed(self, name):
        for item in self.listener: item.config_removed(name)

    def generate_launch(self):
        import hashlib
        xml_hash = hashlib.md5(self.nodepath()).hexdigest()
        xml_path = fspath.package() + "/runner/" + xml_hash + ".xml"
        with open(xml_path, mode="w") as fp:
            fp.write('<launch>\n')
            fp.write('  <include file="' + os.path.join("$(find autoware_launcher)", "plugins", self.plugin.path() + ".xml") + '">\n')
            for arg_name, arg_data in self.config.items():
                if arg_name.startswith("args."):
                    if type(arg_data) is list:
                        arg_data = " ".join(arg_data)
                    fp.write('    <arg name="' + arg_name[5:] + '" value="' + str(arg_data) + '"/>\n')
            fp.write('  </include>\n')
            fp.write('</launch>\n')
        return xml_path

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
        #if not isinstance(listener, AwLaunchNodeListenerIF):
        #    raise TypeError(listener.__class__.__name__ + " does not inherit to AwLaunchNodeListenerIF")
        #self.listener.append(listener)

    # ToDo: remove function
    def unbind_listener(self, listener):
        console.warning("unbind_listener: " + listener.__class__.__name__)
        #self.listener.remove(listener)



if __name__ == "__main__":
    from .plugin import AwPluginTree
    plugin = AwPluginTree()
    launch = AwLaunchTree(None)
    launch.load(fspath.profile("default"), plugin)
    launch.dump()
    launch.save(fspath.profile("sample.bak"))