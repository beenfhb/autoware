import os

from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets



class AwSummaryPanel(QtWidgets.QStackedWidget):

    def __init__(self, client):
        super(AwSummaryPanel, self).__init__()
        self.__client = client
        self.__items  = {}

    def config_cleared(self):
        for key in self.__items.keys():
            self.__items.pop(key).deleteLater()

    def config_created(self, lnode):
        lpath = lnode.path()

    #def config_removed(self, lpath):

    def config_selected(self, lpath):
        self.setCurrentWidget(self.__items[lpath])


