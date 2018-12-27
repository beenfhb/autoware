import os
import xml.etree.ElementTree as xmltree
import yaml

from . import console
from . import fspath



class AwPluginTree(object):
    
    def __init__(self):
        super(AwPluginTree, self).__init__()
        self.__plugins = {}

        filelist = fspath.listfiles(fspath.plugins(), relative=True)
        nodelist = list()
        for filepath in filelist:
            fkey, fext = os.path.splitext(filepath)
            if fext in [".yaml", ".xml"]:
                if fkey not in self.__plugins:
                    self.__plugins[fkey] = AwPluginNode(self, fkey)
            else:
                console.warning("plugin ignored: unknown extension ({})".format(filepath))

        for plugin in self.__plugins.values():
            plugin.load(fspath.plugins())

    def find(self, path):
        return self.__plugins[path]

    def scan(self, path):
        return filter(lambda node: node.startswith(path), self.__plugins.keys())

    def dump(self):
        for name in sorted(self.__plugins.keys()):
            print "=================================================="
            self.__plugins[name].dump()



class AwPluginNode(object):

    def __init__(self, tree, path):
        self.__tree  = tree
        self.__path  = path
        self.__args  = []
        self.__rules = []
        self.__views = []

    def dump(self):
        print self.__path
        print self.__args
        print self.__rules
        print self.__views

    def isnode(self):
        return bool(self.__rules)

    def isleaf(self):
        return not self.isnode()

    def tree(self):
        return self.__tree

    def path(self):
        return self.__path

    def rules(self):
        return self.__rules

    def load(self, rootpath):
        filepath = os.path.join(rootpath, self.path())
        print filepath

        # load xml (roslaunch)
        xml_args = []
        if os.path.exists(filepath + ".xml"):
            xroot = xmltree.parse(filepath + ".xml").getroot()
            for xnode in xroot:
                if ("arg" == xnode.tag) and ("value" not in xnode.attrib):
                    xml_args.append(xnode.attrib)

        # load yaml
        if os.path.exists(filepath + ".yaml"):
            with open(filepath+ ".yaml") as fp:
                ydata = yaml.safe_load(fp)

            for rdata in ydata.get("children", []):
                self.__rules.append(AwPluginRule(self, rdata))
            self.__args  = ydata.get("args", {})
            self.__views = ydata.get("view", [])

        # validation
        args_type = ["str", "int", "real"]
        for argkey, argdef in self.__args.items():
            if "type" not in argdef: raise Exception("yaml arg does not have type: "  + filepath)
            if argdef["type"] not in args_type: raise Exception("yaml arg has unknown type: " + filepath)
        view_type = ["args.str", "args.int", "args.real", "args.tf", "args.topic", "args.frame", "args.file", "args.filelist"]
        for viewdef in self.__views:
            if "type" not in viewdef: raise Exception("yaml arg does not have type: "  + filepath)
            if viewdef["type"] not in view_type: raise Exception("yaml arg has unknown type: " + viewdef["type"] + " " + filepath)

    def args(self):
        return self.__data["args"]

    def views(self):
        return self.__views

    def panel(self):
        return "node" if self.isnode() else "leaf"

    def frame(self):
        return "node" if self.isnode() else "leaf"

    def error(self, text):
        console.error("{}: {} ({})".format(self.__class__.__name__, text, self.__nodepath))

    def optional_children(self):
        plugins = {}
        for rule in self.rules():
            if rule.type == "optional":
                plugins[rule.name] = self.__tree.scan(rule.plugin)
        return plugins

    def default_config(self):
        config = {}
        for argkey, argdef in self.__args.items():
            cfgkey = "args." + argkey
            config[cfgkey] = argdef.get("default", "")
        return config 

    # temporary
    def argstr(self, config):
        lines = []
        for argkey, argdef in self.__args.items():
            cfgkey = "args." + argkey
            lines.append(argkey + ": " + config[cfgkey])
        return "\n".join(lines)



class AwPluginRule(object):

    def __init__(self, node, data):
        self.type = data["type"]
        self.name = data["name"]
        self.plugin = data["plugin"]



if __name__ == "__main__":
    plugin = AwPluginTree()
    plugin.dump()