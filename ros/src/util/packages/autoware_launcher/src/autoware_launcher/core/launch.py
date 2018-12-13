#import collections
#import glob
import json
import os
import yaml



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

    def __init__(self, parent, name):
        self.__nodetype = True
        self.__nodename = name
        self.__parent   = parent
        self.__children = []
        self.__childmap = {}
        if parent:
            parent._AwBaseNode__childmap[name] = self
            parent._AwBaseNode__children.append(self)

    def dump(self, indent = 0):
        print((indent * " ") + str(self))
        for child in self.children(): child.dump(indent + 2)

    def isnode(self):
        return self.__nodetype is True

    def isleaf(self):
        return self.__nodetype is False

    def tree(self):
        return self.__parent.tree()

    def nodename(self):
        return self.__nodename

    def nodepath(self):
        return os.path.join(self.__parent.nodepath(), self.nodename())

    def children(self):
        return self.__children

    def getchild(self, name):
        return self.__childmap[name]

    def haschild(self, name):
        return name in self.__childmap

    def __delchild(self, name):
        child = self.__childmap.pop(name)
        self.__children.remove(child)



class AwBaseTree(AwBaseNode):

    def __init__(self):
        super(AwBaseTree, self).__init__(None, None)

    def isleaf(self):
        return False

    def isnode(self):
        return False

    def tree(self):
        return self

    def nodepath(self):
        return ""

    def find(self, path):
        if not path:
            return self
        else:
            child = self.__childmap.get(path.pop())
            if child:
                return child.find(path)



class AwLaunchTree(AwBaseTree):

    def __init__(self, server):
        super(AwLaunchTree, self).__init__()
        self.server = server

    def __str__(self):
        return "Tree:{}".format(self.nodename())

    def find(self, path):
        node = self
        for name in path.split("/"):
            node = node.getchild(name)
        return node

    # Move to Server
    def request_json(self, json_string):
        request = json.loads(json_string)
        if request.get("command") == "launch":
            node = self.find(request["path"])
            if node:
                node.request_exec()
                return '{"response":"ok"}'
        return None



class AwLaunchNode(AwBaseNode):

    def __init__(self, parent, name, plugin, config):
        super(AwLaunchNode, self).__init__(parent, name)
        self.plugin = plugin
        self.config = config
        self.listener = []
        self.executor = AwLaunchNodeExecutorIF()

    def __str__(self):
        nodetype = "Node" if self.isnode() else "Leaf"
        return "{}:{:<13} Plugin:{}".format(nodetype, self.nodename(), self.plugin.nodepath())

    def bind_listener(self, listener):
        if not isinstance(listener, AwLaunchNodeListenerIF):
            raise TypeError(listener.__class__.__name__ + " does not inherit to AwLaunchNodeListenerIF")
        self.listener.append(listener)
    
    def bind_executor(self, executor):
        if not isinstance(executor, AwLaunchNodeExecutorIF):
            raise TypeError(executor.__class__.__name__ + " does not inherit to AwLaunchNodeExecutorIF")
        self.executor = executor
    
    def unbind_listener(self, listener):
        self.listener.remove(listener)

    def unbind_executor(self, executor):
        self.executor = AwLaunchNodeExecutor()

    # experimental
    def create_child(self, name, plugin):
        if not name:
            return "name is empty"
        if self.haschild(name):
            return "name exists"
        child = AwLaunchNode(self, name)
        child.plugin = plugin
        child.config["info"] = {}
        child.config["args"] = {}
        self.send_config_created(child)
        return None

    # experimental
    def remove_child(self, name):
        if not name:
            return "name is empty"
        if not self.haschild(name):
            return "name does not exist"
        self.delchild(name)
        self.send_config_removed(name)
        return None

    def set_data(self, grp, key, value):
        self.config[grp][key] = value
        self.send_config_updated()

    def get_data(self, grp, key, value = None):
        return self.config[grp].get(key, value)

    def request_exec(self):
        if self.isnode():
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
        xml_path = os.path.join(self.tree().profile_path, self.nodepath() + ".xml")
        with open(xml_path, mode="w") as fp:
            fp.write('<launch>\n')
            fp.write('  <include file="' + os.path.join("$(find autoware_launcher)", "plugins", self.plugin.nodepath() + ".xml") + '">\n')
            for arg_name, arg_data in self.config["args"].items():
                if type(arg_data) is list:
                    arg_data = " ".join(arg_data)
                fp.write('    <arg name="' + arg_name + '" value="' + str(arg_data) + '"/>\n')
            fp.write('  </include>\n')
            fp.write('</launch>\n')
        return xml_path