import collections
import os
import xml.etree.ElementTree as xmltree
import yaml

from . import basetree
from . import console
from . import myutils



class AwPluginTree(basetree.AwBaseTree):

    def __init__(self):
        super(AwPluginTree, self).__init__()

        filelist = myutils.listfiles(myutils.plugins(), relative=True)
        nodelist = list()
        for filepath in filelist:
            fkey, fext = os.path.splitext(filepath)
            if fext in [".yaml", ".xml"]:
                if fkey not in self.nodes:
                    self.nodes[fkey] = AwPluginNode(self, fkey)
            else:
                console.warning("plugin ignored: unknown extension ({})".format(filepath))

        for plugin in self.nodes.values():
            plugin.load(myutils.plugins())



class AwPluginNode(basetree.AwBaseNode):

    def __init__(self, tree, path):
        super(AwPluginNode, self).__init__(tree, path)
        self.__rosxml = None
        self.__args   = None
        self.__exts   = None
        self.__rules  = None
        self.__panel  = None
        self.__frame  = None

    def dump(self):
        print yaml.safe_dump(self.todict())

    def todict(self):
        return \
        {
            "1.name" : self.path(),
            "2.type" : "Node" if self.isnode() else "Leaf",
            "3.file" : self.__rosxml,
            "4.exts" : [data.todict() for data in self.__exts],
            "5.args" : [data.todict() for data in self.__args],
            "6.rules": [data.todict() for data in self.__rules],
            "7.panel": self.__panel.todict(),
            "8.frame": self.__frame.todict()
        }

    def error(self, text):
        console.error("{}: {} ({})".format(self.__class__.__name__, text, self.__nodepath))

    def isnode(self):
        return bool(self.__rules)

    def isleaf(self):
        return not self.isnode()

    #def rosxml(self):
    #    return self.__launch

    #def exts(self):
    #    return self.__exts

    #def args(self):
    #    return self.__args

    def rules(self):
        return self.__rules

    def panel(self):
        return self.__panel

    def frame(self):
        return self.__frame

    def default_config(self):
        values = {"str":"", "int":"0", "bool":False}
        def default_value(data):
            value = data.rest.get("default")
            if value is None:
                return values[data.type]
            else:
                return value
        config = {}
        config.update({"args." + data.name: default_value(data) for data in self.__args})
        config.update({"exts." + data.name: default_value(data) for data in self.__exts})
        return config

    # temporary
    def argstr(self, config):
        lines = []
        for argkey, argdef in self.__args.items():
            cfgkey = "args." + argkey
            lines.append(argkey + ": " + config[cfgkey])
        return "\n".join(lines)

    def load(self, rootpath):
        filepath = os.path.join(rootpath, self.path())

        # load xml (roslaunch)
        #xml_args = []
        #if os.path.exists(filepath + ".xml"):
        #    xroot = xmltree.parse(filepath + ".xml").getroot()
        #    for xnode in xroot:
        #        if ("arg" == xnode.tag) and ("value" not in xnode.attrib):
        #            xml_args.append(xnode.attrib)

        # load yaml
        if os.path.exists(filepath + ".yaml"):
            with open(filepath+ ".yaml") as fp:
                ydata = yaml.safe_load(fp)

            # format check
            if ydata.get("format") != "Autoware Launcher Plugin Version 0.1":
                raise Exception("unknown plugin format: " + filepath)

            self.__rosxml = ydata.get("rosxml", "$(find autoware_launcher)/plugins/{}.xml".format(self.path()))
            self.__exts   = [AwPluginDataElement(data) for data in ydata.get("exts", [])]
            self.__args   = [AwPluginDataElement(data) for data in ydata.get("args", [])]
            self.__rules  = [AwPluginRuleElement(data, self) for data in ydata.get("rules", [])]
            self.__panel  = AwPluginPanelElement(ydata.get("panel", {}))
            self.__frame  = AwPluginFrameElement(ydata.get("frame", {}))

        """
        # Validation
        view_type = ["text", "file", "filelist"]
        rule_type = ["unit", "list"]
        for data in self.__args:
            if data["view"] not in view_type: raise Exception("Plugin Args Type: " + filepath)
        for name, data in self.__rule.items():
            if data["type"] not in rule_type: raise TypeError("Plugin Rule Type: " + filepath)

        # Auto complete
        default_values = {"bool": "False", "str": "", "int": 0, "real": 0.0}
        for group in self.__args:
            if type(group["data"]) == list:
                for field in group["data"]:
                    field.setdefault("default", default_values[field["type"]])
            else:
                field = group["data"]
                field.setdefault("default", default_values[field["type"]])
        """

    def generate_launch(self, config):
        lines = []
        lines.append('<launch>')
        lines.append('  <include file="{}">'.format(self.__rosxml))
        for data in self.__args:
            argvalue = config.get("args." + data.name)
            if argvalue is not None:
                lines.append('    <arg name="{}" value="{}"/>'.format(data.name, data.xmlstr(argvalue)))
        lines.append('  </include>')
        lines.append('</launch>')
        return "\n".join(lines)



# AwPluginArgumentDataElement
# AwPluginExtendedDataElement
class AwPluginDataElement(object):

    def __init__(self, data):
        self.name = data.pop("name")
        self.type = data.pop("type")
        self.list = data.pop("list", None)
        self.rest = data # temporary, add default field

    def todict(self):
        return vars(self)

    def xmlstr(self, value):
        if self.list is None:
            return value
        if self.list == "space":
            return " ".join(value)
        if self.list == "yaml":
            return "[{}]".format(",".join(value))
        raise Error(__class__.__name__ + ".xmlstr")

class AwPluginRuleElement(object):

    def __init__(self, data, node):
        self.unique = not data.get("list", False)
        self.name = data["name"]
        self.plugins = self.__init_plugins(data["plugin"], node)

    def todict(self):
        return vars(self)

    def __init_plugins(self, plist, pnode):
        plugins = []
        ptree = pnode.tree()
        plist = plist if type(plist) is list else [plist]
        for pdata in plist:
            if type(pdata) is dict:
                plugins.extend(ptree.scan(pdata["scan"]))
            elif type(pdata) is str:
                if ptree.find(pdata):
                    plugins.append(pdata)
                else:
                    console.warning("plugin is not found: {} in {}".format(pdata, pnode.path()))
            else:
                console.warning("unknown plugin rule: {} in {}".format(pdata, pnode.path()))
        return plugins

class AwPluginFrameElement(object):

    def __init__(self, data):
        self.widget = data.get("widget", "node.frame")
        self.target = data.get("target")
        self.title  = data.get("title")

    def todict(self):
        return vars(self)

class AwPluginPanelElement(object):

    def __init__(self, data):
        self.widget = data.get("widget", "node.panel")
        self.frames = [AwPluginFrameElement(f) for f in data.get("frames", [])]

    def todict(self):
        return \
        {
            "widget": self.widget,
            "frames": [vars(data) for data in self.frames]
        }




if __name__ == "__main__":
    plugin = AwPluginTree()
    plugin.dump()