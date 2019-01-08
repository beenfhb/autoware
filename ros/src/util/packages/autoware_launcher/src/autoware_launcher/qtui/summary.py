import os

from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets



class AwSummaryPanel(QtWidgets.QStackedWidget):

    def __init__(self, client):
        super(AwSummaryPanel, self).__init__()
        self.__client = client
        self.__panels = {}

    def profile_cleared(self):
        for key in self.__panels.keys():
            self.__panels.pop(key).deleteLater()

    def config_created(self, node):
        lpath = node.path()
        panel = self.__client.guimgr().create_widget(node, node.plugin().panel())
        panel.setup_widget()
        self.__panels[lpath] = panel
        self.addWidget(panel)

    def config_updated(self, lpath):
        self.__panels[lpath].setup_widget()


    #def config_removed(self, lpath):

    def config_selected(self, lpath):
        self.setCurrentWidget(self.__panels[lpath])


