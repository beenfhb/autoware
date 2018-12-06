import os
import yaml

def get_package_path():
    package_path = os.path.dirname(__file__)
    package_path = os.path.join   (package_path, "../../..")
    package_path = os.path.abspath(package_path)
    return package_path

class AwPluginLoader(object):
    
    def __init__(self):

        package_path = get_package_path()
        plugin_path = os.path.join(package_path, "plugins")

        self.__plugins = {}
        self.__load_plugins(plugin_path)

    def __load_plugins(self, treepath):

        self.__plugins = {}
        for dirname in os.listdir(treepath):
            dirpath = os.path.join(treepath, dirname)
            if os.path.isdir(dirpath):
                self.__plugins[dirname] = {}
                for filename in os.listdir(dirpath):
                    filepath = os.path.join(treepath, dirname, filename)
                    if os.path.isfile(filepath):
                        nodename = os.path.splitext(filename)[0]
                        self.__plugins[dirname][nodename] = None

        for dirname in self.__plugins:
            for filename in self.__plugins[dirname]:
                nodepath = os.path.join(dirname, filename)
                self.__plugins[dirname][filename] = AwPluginSchema(treepath, nodepath)

    def find(self, nodepath):
        dirname, filename = nodepath.split("/")
        return self.__plugins[dirname][filename]

    def dump_plugins(self):
        for dirname in self.__plugins:
            for filename in self.__plugins[dirname]:
                print (dirname, filename)



class AwPluginSchema(object):

    def __init__(self, treepath, nodepath):

        self.__treepath = treepath
        self.__nodepath = nodepath

        self.gui = {}
        self.__load_yaml()

    def __load_yaml(self):

        fpath = self.fullpath() + ".yaml"
        if not os.path.exists(fpath):
            return
        with open(fpath) as fp:
            ydata = yaml.safe_load(fp)
        if ydata is None: # for empty file
            ydata = {}

        self.gui = ydata.get("gui", {})

    def nodepath(self):
        return self.__nodepath

    def fullpath(self):
        return os.path.join(self.__treepath, self.__nodepath)