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

    def client(self):
        return self.__client

    def widget(self, opts):
        return self.__widgets[opts["view"]]

    def create_widget(self, node, opts, parent = None, widget = None):
        widget = widget or self.widget(opts)
        return widget(self, node, opts)

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

    def create_arg_frame(self, parent, opts):
        guicls = self.__widgets["args." + opts["type"]]
        return guicls(self, parent, opts)

    def create_info_frame(self, parent, opts):
        guicls = self.__widgets["info." + opts["type"]]
        return guicls(self, parent, opts)

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


    def setup_panel_layout(self, widget):

        if self.layout() is None:
            self._setup_panel_layout(widget)
        else:
            self._reset_panel_layout(widget)

    def _setup_panel_layout(self, widget):

        footer_layout = QtWidgets.QHBoxLayout()
        footer_layout.setContentsMargins(2, 2, 2, 2)
        footer_layout.setSpacing(2)
        footer_layout.addStretch()
        widget.footer = QtWidgets.QWidget()
        widget.footer.setLayout(layout)

        widget_layout = QtWidgets.QVBoxLayout()
        widget_layout.setContentsMargins(16, 16, 16, 16)
        widget_layout.setSpacing(16)
        widget_layout.addStretch()
        widget_layout.addWidget(widget.footer)
        widget.setLayout(widget_layout)

    def _reset_panel_layout(self, widget):

        footer_layout = self.footer.layout()
        while 1 < footer_layout.count():
            footer_layout.takeAt(footer_layout.count() - 1).widget().deleteLater()

        widget_layout = self.layout()
        while 2 < widget_layout.count():
            widget_layout.takeAt(0).widget().deleteLater()

    #ToDo: create_popup_window
