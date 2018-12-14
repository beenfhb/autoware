import os
import yaml
import glob

from . import console
from . import awpath

def get_package_path():
    package_path = os.path.dirname(__file__)
    package_path = os.path.join   (package_path, "../../..")
    package_path = os.path.abspath(package_path)
    return package_path

class AwPluginTree(object):
    
    def __init__(self):
        self.__plugins = {}
        self.__load_plugins()

    def __load_plugins(self):

        plugins_path = awpath.plugins()
        for dirpath, dirs, files in os.walk(plugins_path):
            relpath = os.path.relpath(dirpath, plugins_path)
            for filename in files:
                fkey, fext = os.path.splitext(os.path.join(relpath, filename))
                if fext == ".yaml":
                    self.__plugins[fkey] = AwPluginSchema(plugins_path, fkey)
                elif fext == ".xml":
                    if not os.path.isfile(os.path.join(plugins_path, fkey + ".yaml")):
                        console.warning("since yaml is not found, ignoring {}".format(fkey + fext))
                else:
                    console.warning("since unknown extension, ignoring {}".format(fkey + fext))

    def find(self, nodepath):
        return self.__plugins[nodepath]

    def dump(self):
        for nodename in sorted(self.__plugins.keys()):
            print nodename



class AwPluginSchema(object):

    def __init__(self, treepath, nodepath):

        self.__treepath = treepath
        self.__nodepath = nodepath
        self.__yamldata = None

        self.gui = {}
        self.__load_yaml(self.fullpath())

    def __load_yaml(self, fullpath):

        fullpath = fullpath + ".yaml"
        with open(fullpath) as fp:
            self.__yamldata = yaml.safe_load(fp)
        
        if type(self.__yamldata) is not dict:
            self.error("yaml data is not dictionary")
        else:
            self.__yamldata.setdefault("children", None)

    def default_config(self):
        return {}

    def children(self):
        return self.__yamldata["children"]

    def nodepath(self):
        return self.__nodepath

    def fullpath(self):
        return os.path.join(self.__treepath, self.__nodepath)

    def error(self, text):
        console.error("{}: {} ({})".format(self.__class__.__name__, text, self.__nodepath))
