from python_qt_binding import QtCore, QtWidgets
from autoware_launcher.qtui import widgets



def plugin_widgets():
    return \
    {
        "node" : AwNodeInfoEdit,
    }



class AwNodeInfoEdit(widgets.AwAbstructFrame):

    def __init__(self, guimgr, target, option):
        super(AwNodeInfoEdit, self).__init__(guimgr, target, option)
        self.target = target
        self.option = option

        super(AwNodeInfoEdit, self).setup_widget()
        self.edit = QtWidgets.QLineEdit()
        self.edit.setText(self.target.config.get("info.title"))
        self.edit.editingFinished.connect(self.edited)
        self.add_widget(self.edit)
        self.set_title("Node Title")

    def edited(self):
        self.target.config["info.title"] = self.edit.text()

    @staticmethod
    def summary(option, config):
        return "{}: {}".format(option["title"], config.get("info.title"))