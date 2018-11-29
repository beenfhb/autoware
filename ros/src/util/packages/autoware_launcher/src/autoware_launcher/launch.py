import collections
import glob
import os
import sys
import yaml

class AwLaunchTree(object):

    def __init__(self):

        self.parent   = None
        self.children = []
        
        self.package_path = os.path.dirname(__file__)
        self.package_path = os.path.join   (self.package_path, "../../")
        self.package_path = os.path.abspath(self.package_path)

        self.profile_path = os.path.join(self.package_path, "profiles", "default")
        self.root = AwLaunchNode(self)
        self.root.load(self.profile_path, "root")

        #self.plugin = AwPluginNode("root", None)
        #self.plugin.load(os.path.join(package_dir, "plugins"))

        #self.config = AwConfigNode("root", None, self.plugin)
        #self.config.init()

        #config = AwConfigNode.load(os.path.join(os.path.dirname(__file__), "../profiles/default/"), plugin)

    def tree(self): return self
    def dump(self): self.root.dump(0)



class AwLaunchNodeViewItem(object):
    def exec_requested(self): pass 
    def term_requested(self): pass
    def term_completed(self): pass

class AwLaunchNodeExecutor(object):
    def request_exec(self): pass 
    def request_term(self): pass



class AwLaunchNode(object):

    def __init__(self, parent):
        self.parent   = parent
        self.children = []
        self.nodetype = True
        self.nodename = None    #temporary
        self.nodepath = None    #temporary
        self.setting  = {}
        self.executor = AwLaunchNodeExecutor()
        self.viewitem = AwLaunchNodeViewItem()

        if parent:
            parent.children.append(self)

    def isleaf(self):
        return not self.nodetype

    def isnode(self):
        return self.nodetype

    def tree(self):
        return self.parent.tree()

    def dump(self, indent):
        nodetype = "node" if self.isnode() else "leaf"
        print((" " * indent) + str((nodetype, self.nodepath)))
        if self.isnode():
            for child_node in self.children:
                child_node.dump(indent + 2)

    def bind_viewitem(self, viewitem):
        self.viewitem = viewitem

    def bind_executor(self, executor):
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
        self.viewitem.exec_requested()

    # ToDo: apply state check of executor
    def send_term_requested(self):
        self.viewitem.term_requested()

    # ToDo: apply state check of executor
    def send_term_completed(self):
        self.viewitem.term_completed()


    def load(self, treepath, nodepath):
        self.nodepath = nodepath
        with open(os.path.join(treepath, nodepath + ".yaml")) as fp:
            ydata = yaml.safe_load(fp)
            if ydata.get("children") is None:
                self.nodetype = False
                self.setting["args"] = ydata.get("args", {})
            else:
                for child_name in ydata["children"]:
                    child_node = AwLaunchNode(self)
                    child_node.load(treepath, child_name)

    def generate_launch(self):
        xml_path = os.path.join(self.tree().profile_path, self.nodepath + ".xml")
        with open(xml_path, mode="w") as fp:
            fp.write('<launch>\n')
            fp.write('  <include file="' + os.path.join("$(find autoware_launcher)", "plugins", self.nodepath + ".xml") + '">\n')
            for arg_name, arg_data in self.setting["args"].items():
                if type(arg_data) is list:
                    arg_data = " ".join(arg_data)
                fp.write('    <arg name="' + arg_name + '" value="' + str(arg_data) + '"/>\n')
            fp.write('  </include>\n')
            fp.write('</launch>\n')
        return xml_path

if __name__ == "__main__":
    tree = AwLaunchTree()
    tree.dump()

    print
    tree.root.execute()