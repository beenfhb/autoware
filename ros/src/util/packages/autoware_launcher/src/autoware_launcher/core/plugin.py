import os
import yaml
import glob

from . import console
from . import fspath

def get_package_path():
    package_path = os.path.dirname(__file__)
    package_path = os.path.join   (package_path, "../../..")
    package_path = os.path.abspath(package_path)
    return package_path

class AwPluginTree(object):
    
    def __init__(self):
        self.__plugins = {}
        self.__path = fspath.plugins()

        self.__load_plugins()

    def __load_plugins(self):
        for dirpath, dirs, files in os.walk(self.__path):
            relpath = os.path.relpath(dirpath, self.__path)
            for filename in files:
                fkey, fext = os.path.splitext(os.path.join(relpath, filename))
                if fext == ".yaml":
                    self.__plugins[fkey] = AwPluginSchema(self, fkey)
                elif fext == ".xml":
                    if not os.path.isfile(os.path.join(self.__path, fkey + ".yaml")):
                        console.warning("since yaml is not found, ignoring {}".format(fkey + fext))
                else:
                    console.warning("since unknown extension, ignoring {}".format(fkey + fext))

    def path(self):
        return self.__path

    def find(self, nodepath):
        return self.__plugins[nodepath]

    def scan(self, nodepath):
        return filter(lambda p: p.startswith(nodepath), self.__plugins.keys())

    def dump(self):
        for nodename in sorted(self.__plugins.keys()):
            self.__plugins[nodename].dump()


class AwPluginSchema(object):

    def __init__(self, tree, path):
        self.__tree = tree
        self.__path = path
        self.__node = True
        self.__data = None

        self.gui = {}
        self.__load_yaml(self.tree().path() + "/" + self.path())

    def isnode(self):
        return self.__node is True

    def isleaf(self):
        return self.__node is False

    def tree(self): # Move to BaseNode
        return self.__tree

    def path(self): # Move to BaseNode
        return self.__path

    def __load_yaml(self, fullpath):

        fullpath = fullpath + ".yaml"
        with open(fullpath) as fp:
            self.__data = yaml.safe_load(fp)

        if type(self.__data) is not dict:
            self.error("yaml data is not dictionary")

        if self.__data.get("children") is None:
            self.__node = False
            self.__data["children"] = []
        self.__data.setdefault("args", [])

        self.gui = self.__data.pop("gui", {})
        self.gui.setdefault("type", "default_node" if self.__node else "default_leaf")

    def default_config(self):
        config = {}
        for argdef in self.__data["args"]:
            config["args." + argdef["name"]] = "" if argdef["type"] == "str" else "0"
        return config

    def args(self):
        return self.__data["args"]

    def children(self):
        return self.__data["children"]

    def error(self, text):
        console.error("{}: {} ({})".format(self.__class__.__name__, text, self.__nodepath))

    def dump(self):
        print self.__nodepath
        print "  YML: " + str(self.__data)
        print "  GUI: " + str(self.gui       )

    def optional_children(self):
        plugins = {}
        for cinfo in self.children():
            if cinfo["type"] == "optional":
                plugins[cinfo["name"]] = self.__tree.scan(cinfo["plugin"])
        return plugins


if __name__ == "__main__":
    plugin = AwPluginTree()
    plugin.dump()