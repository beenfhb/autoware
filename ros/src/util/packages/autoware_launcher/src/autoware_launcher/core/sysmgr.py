import os
import yaml

from .       import console
from .       import fspath
from .plugin import AwPluginTree
from .launch import AwLaunchTree
from .launch import AwLaunchNode



class AwLaunchServerIF(object):
    def list_profile(self, lpath): raise NotImplementedError("tree_profile")
    def make_profile(self, ppath): raise NotImplementedError("make_profile")
    def load_profile(self, fpath): raise NotImplementedError("load_profile")
    def save_profile(self, fpath): raise NotImplementedError("save_profile")
    def find_node   (self, lpath): raise NotImplementedError("find_node")
    def exec_node   (self, lpath): raise NotImplementedError("exec_node")
    def term_node   (self, lpath): raise NotImplementedError("term_node")
    def create_node (self, lpath, ppath): raise NotImplementedError("create_node")
    def update_node (self, lpath, ldata): raise NotImplementedError("update_node")
    def remove_node (self, lpath):        raise NotImplementedError("remove_node")
    #def request_launch_push
    #def request_launch_pull
    #def request_plugin_pull
    #def runner_finished
    #def runner_stdouted
    #def runner_stderred

class AwLaunchClientIF(object):
    def launch_state_changed(self, state): raise NotImplementedError("launch_state_changed")



class AwLaunchServer(AwLaunchServerIF):

    def __init__(self, sysarg):
        self.__plugins = AwPluginTree()
        self.__profile = AwLaunchTree(self, self.__plugins)
        #self.__process
        self.__clients = []

    def register_client(self, client):
        self.__clients.append(client)

    def make_profile(self, ppath):
        console.info("make_profile: " + ppath)
        self.__profile = AwLaunchTree(self, self.__plugins)
        self.__profile.make(ppath, self.__plugins)

    def load_profile(self, fpath):
        console.info("load_profile: " + fpath)
        self.__profile = AwLaunchTree(self, self.__plugins)
        self.__profile.load(fspath.profile(fpath), self.__plugins)

    def save_profile(self, fpath):
        console.info("save_profile: " + fpath)
        self.__profile.save(fspath.profile(fpath))

    def find_node(self, lpath):
        console.info("find_node: " + lpath)
        return self.__profile.find(lpath)

    def exec_node(self, lpath):
        console.info("exec_node: " + lpath)
        self.__profile.find(lpath).request_exec()

    def term_node(self, lpath):
        console.info("term_node: " + lpath)
        self.__profile.find(lpath).request_term()

    def create_node(self, lpath, ppath):
        return self.__profile.create(lpath, ppath)

    def update_node(self, lpath, ldata):
        response =  self.__profile.find(lpath).update(ldata)
        if not response["error"]:
            for client in self.__clients: client.node_updated(lpath)
        return response

    #def request_json(self, json_string):
    #    request = json.loads(json_string)
    #    if request.get("command") == "launch":
    #        node = self.find(request["path"])
    #        if node:
    #            node.request_exec()
    #            return '{"response":"ok"}'
    #    return None
