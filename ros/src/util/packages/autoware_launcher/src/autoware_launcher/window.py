from python_qt_binding import QtWidgets

import widgets

class AwWindowManager(object):

    def __init__(self):

        self.classes = {}
        self.classes["root"] = ( widgets.AwNodeWindow,       None                      )
        self.classes["node"] = ( widgets.AwNodeWindow,       widgets.AwNodeFrame       )
        self.classes["leaf"] = ( widgets.AwLeafWindow,       widgets.AwLeafFrame       )
        self.classes["arg" ] = ( None,                       widgets.AwTextEditFrame   )
        self.classes["file_select"] = ( widgets.AwFileSelectWindow, widgets.AwFileSelectFrame )

    def run(self, sysarg, awtree):

        application = QtWidgets.QApplication(sysarg)
        window = self.create_window(None, awtree.config)
        window.show()
        return application.exec_()

    def create_window(self, parent, config):

        print "Create window" + str((config.name, config.plugin.name, config.plugin.type, config.plugin.gui))
        gui_type = config.plugin.gui.get("type", config.plugin.type)
        return self.classes[gui_type][0](self, parent, config)

    def create_frame(self, parent, config):

        print "Create frame" + str((config.name, config.plugin.name, config.plugin.type, config.plugin.gui))
        gui_type = config.plugin.gui.get("type", config.plugin.type)
        return self.classes[gui_type][1](self, parent, config)

    def create_data_frame(self, parent, plugin):

        print "Create data flame" + str((plugin.name, plugin.type, plugin.gui))
        gui_type = plugin.gui.get("type", plugin.type)
        return self.classes[gui_type][1](self, parent, plugin)