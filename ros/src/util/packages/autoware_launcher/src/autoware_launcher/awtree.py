import collections
import glob
import os
import sys
import yaml

def extract_nodename(path):
    return os.path.splitext(os.path.basename(path))[0]

class AwPluginNode(object):
    
    def __init__(self, nodename, parent = None):
        self.nodename = nodename
        self.nodeinfo = {}
        self.parent   = parent
        self.children = collections.OrderedDict()

    def create_child(self, name):
        self.children[name] = AwPluginNode(name, self)
        return self.children[name]

    def dump(self, indent = 0):
        print((" " * indent) + self.nodename)
        for child in self.children.values():
            child.dump(indent + 2)

    def __load_node(self, path):
        nodepath = os.path.join(path, self.nodename)
        try:
            with open(nodepath + ".yaml") as fp:
                self.nodeinfo.update(yaml.safe_load(fp))
            if self.nodeinfo["type"] == "node":
                if self.nodeinfo["children"] == "scan":
                    print "scan is not currently supported"
                for subpath in self.nodeinfo["children"]:
                    subnode = self.create_child(extract_nodename(subpath))
                    subnode.__load_node(nodepath)
        except:
            sys.stderr.write("failed to load yaml file for plugin: " + nodepath + "\n")

    @staticmethod
    def load(path):
        root = AwPluginNode("root")
        root.__load_node(path)
        return root

if __name__ == "__main__":
    path = os.path.join(os.path.dirname(__file__), "../../plugins/")
    root = AwPluginNode.load(path)
    root.dump()
