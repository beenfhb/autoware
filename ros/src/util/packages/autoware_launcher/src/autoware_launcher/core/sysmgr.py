import os
import yaml

from .       import console
from .       import fsys
from .plugin import AwPluginTree
from .launch import AwLaunchTree
from .launch import AwLaunchNode

class AwLaunchServer(object):

    def __init__(self, sysarg):
        self.__plugin = AwPluginTree()
        self.__launch = AwLaunchTree(self)
        self.__client = []

    def __add_client(self, client):
        self.__client.append(client)

    def __init_launch_node(self, parent, name, plugin_path):
        plugin = self.__plugin.find(plugin_path)
        config = plugin.default_config()
        launch = AwLaunchNode(parent, name, plugin, config)
        for cinfo in plugin.children():
            if cinfo["type"] == "single":
                self.__init_launch_node(launch, cinfo["name"], cinfo["plugin"])

    # move loading yaml part to static function of AwLaunchNode
    def __load_launch_node(self, parent, name, launch_path):
        launch_path = os.path.join(fsys.profile_path(), launch_path, name)
        with open(launch_path + ".yaml") as fp:
            yamldata = yaml.safe_load(fp)
        yamldata.setdefault("children", [])

        plugin = self.__plugin.find(yamldata["plugin"])
        config = {}
        for grp in ["info", "args"]:
            for key, val in yamldata.get(grp, {}).items():
                config[grp + "." + key] = val
        

        launch = AwLaunchNode(parent, name, plugin, config)
        for cname in yamldata["children"]:
            self.__load_launch_node(launch, cname, launch_path)

    def init_launch_tree(self, plugin_root):
        console.info("make_launch_tree: " + plugin_root)
        self.__make_launch_node(self.__launch, "root", plugin_root)

    def load_launch_tree(self, launch_root):
        console.info("load_launch_tree: " + launch_root)
        self.__load_launch_node(self.__launch, "root", fsys.profile_path(launch_root))

    def find_launch_node(self, launch_path):
        console.info("find_launch_node: " + launch_path)
        return self.__launch.find(launch_path)



class AwLaunchClient(object):

    def __init__(self, sysarg, server):
        #self.__plugin = 
        self.__launch = AwLaunchTreeMirror(self)
        self.__server = server
        self.__server._AwLaunchServer__add_client(self)

    def mirror(self, path):
        return self.__launch.mirror(path)

    def init_launch_tree(self, plugin_root):
        return self.__server.init_launch_tree(plugin_root)

    def load_launch_tree(self, launch_root):
        return self.__server.load_launch_tree(launch_root)

    def find_launch_node(self, launch_path):
        return self.__server.find_launch_node(launch_path)



class AwLaunchTreeMirror(object):

    def __init__(self, client):
        self.client = client

    def mirror(self, path):
        return AwLaunchNodeMirror(self, path)



class AwLaunchNodeMirror(object):

    def __init__(self, tree, path):
        self.tree = tree
        self.path = path