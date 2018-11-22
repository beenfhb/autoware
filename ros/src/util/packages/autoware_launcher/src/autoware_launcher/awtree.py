import collections
import glob
import os
import sys
import yaml

def extract_nodename(path):
    return os.path.splitext(os.path.basename(path))[0]

class AwPluginNode(object):
    
    def __init__(self, nodename, parent = None):
        self.__nodename = nodename
        self.__parent   = parent
        self.__children = collections.OrderedDict()
        self.__nodeinfo   = {}

    def create_child(self, name):
        self.__children[name] = AwPluginNode(name, self)
        return self.__children[name]

    def dump(self, indent = 0):
        print((" " * indent) + self.__nodename)
        for child in self.__children.values():
            child.dump(indent + 2)

    def __load_node(self, path):
        nodepath = os.path.join(path, self.__nodename)
        print "NodePath: " + nodepath 
        try:
            with open(nodepath + ".yaml") as fp:
                self.__nodeinfo.update(yaml.safe_load(fp))
            if self.__nodeinfo["type"] == "node":
                for subpath in glob.glob(os.path.join(nodepath, "*.yaml")):
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


    #root = AwConfigNode("root")
    #root.create_child("map")
    #root.dump()tree/../../../plugins/root.yaml