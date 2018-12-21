import os
import yaml
import glob

from . import console
from . import fspath



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
                    self.__plugins[fkey] = AwPluginNode(self, fkey)
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


class AwPluginNode(object):

    def __init__(self, tree, path):
        self.__tree = tree
        self.__path = path
        self.__node = True
        self.__data = None
        self.__view = None

        #self.gui = {}
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


        self.__view = []
        if self.__data.get("view") is None:
            for args in self.__data["args"]:
                name = args["name"]
                self.__view.append(AwPluginView("args.text", {"title": name, "args": name}))
        else:
            for viewdef in self.__data["view"]:
                guikey = viewdef.pop("type")
                self.__view.append(AwPluginView(guikey, viewdef))

        print "================================================================"
        for view in self.__view:
            print view

        self.gui = self.__data.pop("gui", {})
        self.gui.setdefault("type", "default_node" if self.__node else "default_leaf")

    def default_config(self):
        config = {}
        for argdef in self.__data["args"]:
            config["args." + argdef["name"]] = ""
        return config

    def args(self):
        return self.__data["args"]

    def view(self):
        return self.__view

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

    # temporary
    def argstr(self, config):
        lines = []
        for argdef in self.__data["args"]:
            name = argdef["name"]
            lines.append(name + ": " + config["args." + name])
            #argstr.append(self.mirror.plugin().argstr(argdef, config))
        return "\n".join(lines)



class AwPluginView(object):

    def __init__(self, guikey, option):
        self.guikey = guikey
        self.option = option
    
    def __str__(self):
        return "Type:{} Option:{}".format(self.guikey, self.option)



if __name__ == "__main__":
    plugin = AwPluginTree()
    plugin.dump()