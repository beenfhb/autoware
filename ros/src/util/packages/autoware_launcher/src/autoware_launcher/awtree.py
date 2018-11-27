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



class AwPluginNode(object):
    
    def __init__(self, nodename, parent):
        self.nodename = nodename
        self.children = collections.OrderedDict()
        self.parent = parent
        self.schema = {"text":"__NO_TITLE__"}

    def create_child(self, name):
        self.children[name] = AwPluginNode(name, self)
        return self.children[name]

    def dump(self, indent = 0):
        print((" " * indent) + self.nodename + " " + str(self.schema))
        for child in self.children.values():
            child.dump(indent + 2)

    def load(self, path):
        nodepath = os.path.join(path, self.nodename)
        try:
            with open(nodepath + ".yaml") as fp:
                self.schema.update(yaml.safe_load(fp))
            if self.schema["type"] == "node":
                if self.schema["children"] == "scan":
                    print "scan is not currently supported"
                for subpath in self.schema["children"]:
                    child = self.create_child(subpath)
                    child.load(nodepath)
            elif self.schema["type"] == "leaf":
                for arg_name, arg_info in self.schema["args"].items():
                    child = self.create_child(arg_name)
                    child.schema.update(arg_info)
        except:
            sys.stderr.write("failed to load yaml file for plugin: " + nodepath + "\n")



class AwConfigNode(object):

    def __init__(self, nodename, parent, plugin):
        self.nodename = nodename
        self.children = collections.OrderedDict()
        self.parent = parent
        self.plugin = plugin
        self.params = {}

    def create_child(self, name, plugin):
        self.children[name] = AwConfigNode(name, self, plugin)
        return self.children[name]

    def dump(self, indent = 0):
        print((" " * indent) + self.nodename)
        for child in self.children.values():
            child.dump(indent + 2)

    def init(self):
        if self.plugin.schema["type"] == "node":
            for cp in self.plugin.children.values():
                child = self.create_child(cp.nodename, cp)
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
