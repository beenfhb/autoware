import os
import yaml

from .       import console
from .       import fspath
from .plugin import AwPluginTree
from .launch import AwLaunchTree
from .launch import AwLaunchNode



class AwLaunchServerIF(object):
    def request_launch_init(self, plugin_root): raise NotImplementedError("request_launch_init")
    def request_launch_load(self, launch_root): raise NotImplementedError("request_launch_load")
    def request_launch_find(self, launch_path): raise NotImplementedError("request_launch_find")

class AwLaunchClientIF(object):
    pass



class AwLaunchServer(AwLaunchServerIF):

    def __init__(self, sysarg):
        self.__plugins = AwPluginTree()
        self.__profile = AwLaunchTree(self)
        self.__clients = []

    def __add_client(self, client):
        self.__client.append(client)

    # move to tree
    def __init_launch_node(self, parent, name, plugin_path):
        plugin = self.__plugins.find(plugin_path)
        config = plugin.default_config()
        launch = AwLaunchNode(parent, name, plugin, config)
        for cinfo in plugin.children():
            print cinfo
            if cinfo["type"] == "single":
                self.__init_launch_node(launch, cinfo["name"], cinfo["plugin"])

    def request_launch_init(self, plugin_root):
        console.info("request_launch_init: " + plugin_root)
        self.__profile = AwLaunchTree(self, None)
        self.__init_launch_node(self.__profile, "root", plugin_root)

    def request_launch_load(self, launch_root):
        console.info("request_launch_load: " + launch_root)
        self.__profile = AwLaunchTree(self)
        self.__profile.load(fspath.profile(launch_root), self.__plugins)

    def request_launch_find(self, launch_path):
        console.info("request_launch_find: " + launch_path)
        return self.__profile.find(launch_path)

    def request_launch_exec(self, lpath):
        console.info("request_launch_exec: " + lpath)
        self.__profile.find(lpath).request_exec()
