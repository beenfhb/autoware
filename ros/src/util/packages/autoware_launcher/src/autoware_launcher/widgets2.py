from python_qt_binding import QtWidgets

from autoware_launcher import widgets



class AwRootWindow(widgets.AwBasicWindow):

    def __init__(self, guimgr, parent, awnode):

        super(AwRootWindow, self).__init__(guimgr, parent, awnode)
        self.load_plugin()
        self.load_geometry()

    def closeEvent(self, event):

        self.save_geometry()



class AwNodeFrame(widgets.AwBasicFrame):

    def __init__(self, guimgr, parent, awnode):

        super(AwNodeFrame, self).__init__(guimgr, parent, awnode)
        self.load_plugin()

    def load_plugin(self):

        header = QtWidgets.QLabel("Menu")
        detail = QtWidgets.QPushButton("Config")
        self.set_widgets(header, detail)

        header.setText(self.awnode.plugin["text"])
        detail.clicked.connect(self.open_window)



class AwNodeWindow(widgets.AwBasicWindow):

    def __init__(self, guimgr, parent, awnode):

        super(AwNodeWindow, self).__init__(guimgr, parent, awnode)
        self.load_plugin()

        self.load_geometry()
