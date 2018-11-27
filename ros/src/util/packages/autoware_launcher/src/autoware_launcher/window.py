from python_qt_binding import QtWidgets

import widgets

class AwWindowManager(object):

    def __init__(self):

        self.classes = {}
        self.classes["root"] = ( widgets.AwRootWindow,       None                      )
        self.classes["node"] = ( widgets.AwNodeWindow,       widgets.AwNodeFrame       )
        self.classes["leaf"] = ( widgets.AwNodeWindow,       widgets.AwNodeFrame       )
        self.classes["file"] = ( widgets.AwFileSelectWindow, widgets.AwFileSelectFrame )

    def run(self, sysarg, awtree):

        application = QtWidgets.QApplication(sysarg)
        window = self.create_window(None, awtree.config)
        window.show()
        return application.exec_()

    def create_window(self, parent, config):

        print ("window", config.nodename, config.plugin.nodename, config.plugin.schema["type"])
        return self.classes[config.plugin.schema["type"]][0](self, parent, config)

    def create_frame(self, parent, config):

        print ("frame", config.nodename, config.plugin.nodename, config.plugin.schema["type"])
        return self.classes[config.plugin.schema["type"]][1](self, parent, config)