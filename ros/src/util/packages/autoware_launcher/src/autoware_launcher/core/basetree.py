import os
import yaml



class AwBaseTree(object):
    pass



class AwBaseNode(object):

    def __init__(self):
        self.__tree = tree
        self.__path = path

    def tree(self):
        return self.__tree

    def path(self):
        return self.__path

    def name(self):
        return os.path.basename(self.__path)
