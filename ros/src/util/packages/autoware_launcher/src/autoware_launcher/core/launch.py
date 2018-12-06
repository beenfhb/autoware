#import collections
#import glob
import json
import os
import yaml



# class AwBaseTree(object):
# class AwBaseNode(object):

class AwLaunchNodeListener(object): #interface
    def exec_requested(self): pass 
    def term_requested(self): pass
    def term_completed(self): pass

class AwLaunchNodeExecutor(object): #interface
    def request_exec(self): pass 
    def request_term(self): pass



class AwLaunchTree(object):

    def __init__(self, loader):

        #self.parent   = None
        self.__children = []
        self.__childmap = {}
        
        self.package_path = os.path.dirname(__file__)
        self.package_path = os.path.join   (self.package_path, "../../..")
        self.package_path = os.path.abspath(self.package_path)

        self.profile_path = os.path.join(self.package_path, "profiles", "default")
        self.root = AwLaunchNode(self, "root")
        self.root.load(self.profile_path, loader)

    # Move to AwBaseNode/Tree
    def tree(self): return self

    # Move to AwBaseNode/Tree
    def dump(self): self.root.dump(0)

    # Move to AwBaseNode/Tree
    def find(self, path):
        path.reverse()
        name = path.pop()
        return self.getchild(name).find(path)

    # Move to AwBaseNode/Tree
    def nodepath(self):
        return ""

    def request_json(self, json_string):
        request = json.loads(json_string)
        if request.get("command") == "launch":
            node = self.find(request["path"])
            if node:
                node.request_exec()
                return '{"response":"ok"}'
        return None


    #temporary
    def children(self):
        return self.__children
    def getchild(self, name):
        return self.__childmap[name]
    def addchild(self, name, child):
        self.__children.append(child)
        self.__childmap[name] = child


class AwBaseNode(object):

    def __init__(self, parent, name):
        self.nodetype = True
        self.__nodename = name
        self.__parent   = parent
        self.__children = []
        self.__childmap = {}
        parent.addchild(name, self)

    def isleaf(self):
        return not self.nodetype

    def isnode(self):
        return self.nodetype

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

    def addchild(self, name, child):
        self.__children.append(child)
        self.__childmap[name] = child

    def find(self, path):
        if not path:
            return self
        else:
            child = self.__childmap.get(path.pop())
            if child:
                return child.find(path)



class AwLaunchNode(AwBaseNode):

    def __init__(self, parent, name):
        super(AwLaunchNode, self).__init__(parent, name)
        self.plugin   = None
        self.widget   = None
        self.__config = {}
        self.listener = [] #AwLaunchNodeListener
        self.executor = AwLaunchNodeExecutor()

    # Move to AwBaseNode
    def dump(self, indent):
        nodetype = "node" if self.isnode() else "leaf"
        print((" " * indent) + str((nodetype, self.nodename(), self.plugin.nodepath(), self.widget)))
        if self.isnode():
            for child_node in self.children():
                child_node.dump(indent + 2)

    def bind_listener(self, listener):
        #isinstance
        self.listener.append(listener)
    
    def unbind_listener(self, listener):
        self.listener.remove(listener)

    def bind_executor(self, executor):
        #isinstance
        self.executor = executor
    
    def unbind_executor(self, executor):
        self.executor = AwLaunchNodeExecutor()

    def set_data(self, grp, key, value):
        self.__config[grp][key] = value
        self.send_config_updated()

    def get_data(self, grp, key, value = None):
        return self.__config[grp].get(key, value)

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

    def load(self, fpath, loader):
        fpath = os.path.join(fpath, self.nodename()) 
        with open(fpath + ".yaml") as fp:
            ydata = yaml.safe_load(fp)
        self.plugin = loader.find(ydata.get("plugin", "node/default"))
        self.widget = ydata.get("widget")
        self.__config["info"] = ydata.get("info", {})
        self.__config["args"] = ydata.get("args", {})
        if ydata.get("children") is None:
            self.nodetype = False
        else:
            for child_name in ydata["children"]:
                child_node = AwLaunchNode(self, child_name)
                child_node.load(fpath, loader)

    def generate_launch(self):
        xml_path = os.path.join(self.tree().profile_path, self.nodepath() + ".xml")
        with open(xml_path, mode="w") as fp:
            fp.write('<launch>\n')
            fp.write('  <include file="' + os.path.join("$(find autoware_launcher)", "plugins", self.plugin.nodepath() + ".xml") + '">\n')
            for arg_name, arg_data in self.__config["args"].items():
                if type(arg_data) is list:
                    arg_data = " ".join(arg_data)
                fp.write('    <arg name="' + arg_name + '" value="' + str(arg_data) + '"/>\n')
            fp.write('  </include>\n')
            fp.write('</launch>\n')
        return xml_path
