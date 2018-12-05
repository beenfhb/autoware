#import collections
#import glob
import json
import os
import sys
import yaml



# class AwBaseTree(object):
# class AwBaseNode(object):

class AwLaunchNodeViewItem(object): #interface
    def exec_requested(self): pass 
    def term_requested(self): pass
    def term_completed(self): pass

class AwLaunchNodeExecutor(object): #interface
    def request_exec(self): pass 
    def request_term(self): pass



class AwLaunchTree(object):

    def __init__(self):

        #self.parent   = None
        self.children = []
        self.childmap = {}
        
        self.package_path = os.path.dirname(__file__)
        self.package_path = os.path.join   (self.package_path, "../../..")
        self.package_path = os.path.abspath(self.package_path)

        self.profile_path = os.path.join(self.package_path, "profiles", "default")
        self.root = AwLaunchNode(self, "root")
        self.root.load(self.profile_path)

    # Move to AwBaseNode/Tree
    def tree(self): return self

    # Move to AwBaseNode/Tree
    def dump(self): self.root.dump(0)

    # Move to AwBaseNode/Tree
    def find(self, path):
        path.reverse()
        name = path.pop()
        return self.childmap[name].find(path)

    # Move to AwBaseNode/Tree
    def nodepath(self):
        return ""

    def request_json(self, json_string):
        request = json.loads(json_string)
        if request.get("command") == "launch":
            self.find(request["path"]).request_exec()
            return '{"response":"ok"}'
        return None



class AwLaunchNode(object):

    def __init__(self, parent, name):
        self.nodetype = True
        self.nodename = name
        self.parent   = parent
        self.children = []
        self.childmap = {}

        if parent:
            parent.children.append(self)
            parent.childmap[name] = self

        self.setting  = {}
        self.listener = [] #AwLaunchNodeViewItem
        self.executor = AwLaunchNodeExecutor()

    # Move to AwBaseNode
    def isleaf(self):
        return not self.nodetype

    # Move to AwBaseNode
    def isnode(self):
        return self.nodetype

    # Move to AwBaseNode
    def tree(self):
        return self.parent.tree()

    # Move to AwBaseNode
    def dump(self, indent):
        nodetype = "node" if self.isnode() else "leaf"
        print((" " * indent) + str((nodetype, self.nodename)))
        if self.isnode():
            for child_node in self.children:
                child_node.dump(indent + 2)

    # Move to AwBaseNode
    def find(self, path):
        if not path:
            return self
        else:
            name = path.pop()
            return self.childmap[name].find(path)

    # Move to AwBaseNode
    def nodepath(self):
        return os.path.join(self.parent.nodepath(), self.nodename)

    def bind_listener(self, listener):
        #isinstance
        self.listener.append(listener)

    def bind_executor(self, executor):
        #isinstance
        self.executor = executor

    def request_exec(self):
        if self.isnode():
            self.send_exec_requested()
            for child_node in self.children:
                child_node.request_exec()
        else:
            self.send_exec_requested()
            self.executor.request_exec()

    def request_term(self):
        if self.isnode():
            self.send_term_completed()
            for child_node in self.children:
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

    def load(self, fpath):
        fpath = os.path.join(fpath, self.nodename) 
        with open(fpath + ".yaml") as fp:
            ydata = yaml.safe_load(fp)
        self.setting["info"] = ydata.get("info", {})
        if ydata.get("children") is None:
            self.nodetype = False
            self.setting["args"] = ydata.get("args", {})
        else:
            for child_name in ydata["children"]:
                child_node = AwLaunchNode(self, child_name)
                child_node.load(fpath)

    def generate_launch(self):
        xml_path = os.path.join(self.tree().profile_path, self.nodepath() + ".xml")
        with open(xml_path, mode="w") as fp:
            fp.write('<launch>\n')
            fp.write('  <include file="' + os.path.join("$(find autoware_launcher)", "plugins", self.nodename + ".xml") + '">\n')
            for arg_name, arg_data in self.setting["args"].items():
                if type(arg_data) is list:
                    arg_data = " ".join(arg_data)
                fp.write('    <arg name="' + arg_name + '" value="' + str(arg_data) + '"/>\n')
            fp.write('  </include>\n')
            fp.write('</launch>\n')
        return xml_path
