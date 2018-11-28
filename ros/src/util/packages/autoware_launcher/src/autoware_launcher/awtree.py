import collections
import glob
import os
import sys
import yaml

#def extract_nodename(path):
#    return os.path.splitext(os.path.basename(path))[0]

class AwLaunchTree(object):

    def __init__(self):

        package_dir = os.path.dirname(__file__)
        package_dir = os.path.join(package_dir, "../../")
        package_dir = os.path.abspath(package_dir)

        self.plugin = AwPluginNode("root", None)
        self.plugin.load(os.path.join(package_dir, "plugins"))

        self.config = AwConfigNode("root", None, self.plugin)
        self.config.init()

        #config = AwConfigNode.load(os.path.join(os.path.dirname(__file__), "../profiles/default/"), plugin)

class AwPluginXml(object):

    def __init__(self):
        self.params  = []
        self.args    = []
        self.include = []

    def load_obj(self, obj):
        self.params   = obj.get("params",   [])
        self.args     = obj.get("args",     [])
        self.includes = obj.get("includes", [])

class AwPluginGuiNode(object):
    pass

class AwPluginNode(object):
    
    def __init__(self, name, parent):
        self.type = None
        self.name = name
        self.data = []
        self.xml = AwPluginXml()
        self.gui = {"text":"===== "+name+" ====="}
        self.parent = parent
        self.children = collections.OrderedDict()

    def isnode(self):
        return self.type == "node"

    def create_child(self, name):
        self.children[name] = AwPluginNode(name, self)
        return self.children[name]

    def dump(self, indent = 0):
        print((" " * indent) + self.type + " " + self.name + " GUI:" + str(self.gui))
        for child in self.data:
            child.dump(indent + 2)
        for child in self.children.values():
            child.dump(indent + 2)

    def load(self, path):
        path = os.path.join(path, self.name)
        self.type = "leaf" if os.path.exists(path + ".xml") else "node"
        try:
            with open(path + ".yaml") as fp:
                yaml_data = yaml.safe_load(fp)
                self.xml.load_obj(yaml_data.get("xml", {}))
                self.gui.update( yaml_data.get("gui", {}) )
            if self.isnode():
                for inc_name in self.xml.includes:
                    child = self.create_child(inc_name)
                    child.load(path)
            else:
                for arg_info in self.xml.args:
                    child = AwPluginNode(arg_info["name"], self)
                    child.type = "arg"
                    child.gui.update(self.gui.get("args", {}).get(arg_info["name"], {}))
                    self.data.append(child)
        except:
            sys.stderr.write("failed to load yaml file for plugin: " + path + "\n")
            import traceback
            traceback.print_exc()



class AwConfigNode(object):

    def __init__(self, name, parent, plugin):
        self.name = name
        self.data = None
        self.plugin = plugin
        self.parent = parent
        self.children = collections.OrderedDict()

    def create_child(self, name, plugin):
        self.children[name] = AwConfigNode(name, self, plugin)
        return self.children[name]

    def dump(self, indent = 0):
        print((" " * indent) + self.name + " " + str(self.data))
        for child in self.children.values():
            child.dump(indent + 2)

    def init(self):
        for pnode in self.plugin.children.values():
            child = self.create_child(pnode.name, pnode)
            child.init()

    def save(self, path):
        pass

    def load(self, path):
        nodepath = os.path.join(path, self.nodename)
        try:
            with open(nodepath + ".xml") as fp:
                self.schema.update(yaml.safe_load(fp))
        except:
            sys.stderr.write("failed to load yaml file for profile: " + nodepath + "\n")



if __name__ == "__main__":
    path = os.path.join(os.path.dirname(__file__), "../../plugins/")
    root = AwPluginNode.load(path)
    root.dump()
