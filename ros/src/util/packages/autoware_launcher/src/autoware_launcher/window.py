from python_qt_binding.QtWidgets import QApplication

import widgets
import widgets2

class AwWindowManager(object):

    def __init__(self):

        self.classes = {}
        self.classes["root"] = ( widgets2.AwRootWindow, None                 )
        self.classes["node"] = ( widgets2.AwNodeWindow, widgets2.AwNodeFrame )
        self.classes["leaf"] = ( widgets2.AwNodeWindow, widgets2.AwNodeFrame )

    def run(self, sysarg, awtree):

        application = QApplication(sysarg)
        window = self.create_window(None, awtree)
        window.show()
        return application.exec_()

    def create_window(self, parent, awnode):

        print ("window", awnode.nodename, awnode.plugin["type"])
        return self.classes[awnode.plugin["type"]][0](self, parent, awnode)

    def create_frame(self, parent, awnode):

        print ("frame", awnode.nodename, awnode.plugin["type"])
        return self.classes[awnode.plugin["type"]][1](self, parent, awnode)