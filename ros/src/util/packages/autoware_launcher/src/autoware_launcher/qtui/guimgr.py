import importlib
import os

from python_qt_binding import QtCore
from python_qt_binding import QtWidgets

from ..core import console
from ..core import fspath

# ToDo: move package
#     core = client, mirror
#     qtui = guimgr



class AwQtGuiManager(object):

    def __init__(self, client):
        self.__client  = client
        self.__widgets = {}

        for filepath in os.listdir(fspath.package("src/autoware_launcher/qtui/plugins")):
            fkey, fext = os.path.splitext(os.path.basename(filepath))
            if (fkey != "__init__") and (fext == ".py"):
                console.info("load plugin module: " + fkey)
                module = importlib.import_module("autoware_launcher.qtui.plugins." + fkey)
                for wkey, wcls in module.plugin_widgets().items():
                     self.__widgets[fkey + "." + wkey] = wcls

    #def widget(self, view):
    #    return self.__widgets[view["view"]]

    def client(self):
        return self.__client

    def create_widget(self, node, view, parent = None, widget = None):
        widget = widget or self.__widgets[view["view"]]
        return widget(self, node, view)

    def create_frame(self, mirror, guikey = None, guicls = None):
        #print "Create Frame: {:<7} Key: {} Class: {}".format(mirror.nodename(), guikey, guicls)
        if not guicls:
            guikey = guikey or mirror.plugin().frame()
            guicls = self.__widgets[guikey + "_frame"]
        return guicls(self, mirror)

    def create_panel(self, mirror, guikey = None, guicls = None):
        #print "Create Panel: {:<7} Key: {} Class: {}".format(mirror.nodename(), guikey, guicls)
        if not guicls:
            guikey = guikey or mirror.plugin().panel()
            guicls = self.__widgets[guikey + "_panel"]
        return guicls(self, mirror)

    def create_arg_frame(self, parent, view):
        guicls = self.__widgets["args." + view["type"]]
        return guicls(self, parent, view)

    def create_frame_entire_vlayout(self):
        layout = QtWidgets.QVBoxLayout()
        layout.setSpacing(0)
        layout.setContentsMargins(0, 0, 0, 0)
        return layout

    def create_frame_header_hlayout(self):
        layout = QtWidgets.QHBoxLayout()
        layout.setSpacing(0)
        layout.setContentsMargins(5, 2, 2, 2)
        return layout
