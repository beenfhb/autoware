from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets



class AwQuickStartPanel(QtWidgets.QWidget):

    def __init__(self, guimgr, window, launch):
        super(AwQuickStartPanel, self).__init__()
        self.guimgr = guimgr
        self.window = window
        self.launch = launch
        
        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)
        layout.addWidget(AwDefaultNodeFrame(guimgr, launch))
        for child in launch.children():
            layout.addWidget(guimgr.create_frame(child))
        layout.addStretch()

        self.setLayout(layout)



class AwDefaultNodePanel(QtWidgets.QWidget):

    def __init__(self, guimgr, window, launch):
        super(AwDefaultNodePanel, self).__init__()
        self.guimgr = guimgr
        self.window = window
        self.launch = launch
        
        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)
        for child in launch.children():
            layout.addWidget(guimgr.create_frame(child))
        layout.addStretch()

        self.setLayout(layout)



class AwAbstructFrame(QtWidgets.QWidget):

    def __init__(self, guimgr, launch):
        super(AwAbstructFrame, self).__init__()
        self.guimgr = guimgr
        self.launch = launch

        self.title = QtWidgets.QLabel("No Title")
        self.title.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)

        layout = QtWidgets.QHBoxLayout()
        layout.setContentsMargins(5, 2, 2, 2)
        layout.setSpacing(0)
        layout.addWidget(self.title)

        self.header = QtWidgets.QWidget()
        self.header.setObjectName("FrameHeader")
        self.header.setLayout(layout)

        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self.header)
        self.setLayout(layout)

    def set_title(self, title):
        self.title.setText(title + " : " + self.__class__.__name__)

    def add_widget(self, widget):
        widget.setObjectName("FrameWidget")
        self.layout().addWidget(widget)

    def add_button(self, button):
        self.header.layout().addWidget(button)

    def add_launch_button(self, button):
        self.add_button(AwLaunchButton(self.launch))

    def add_config_button(self):

        button = QtWidgets.QPushButton("Config")
        button.clicked.connect(self.guimgr.create_window_open_event(self))
        self.add_button(button)

    def create_dummy_widget(self, text):

        layout = QtWidgets.QHBoxLayout()
        layout.setContentsMargins(5, 2, 2, 2)
        layout.setSpacing(0)
        layout.addWidget(QtWidgets.QLabel(text))

        widget = QtWidgets.QWidget()
        widget.setLayout(layout)
        return widget



class AwDefaultNodeFrame(AwAbstructFrame):

    def __init__(self, guimgr, launch):
        super(AwDefaultNodeFrame, self).__init__(guimgr, launch)
        self.set_title(launch.nodename().capitalize())
        self.add_config_button()
        self.add_widget(self.create_dummy_widget(launch.get_data("info", "title")))


class AwFileSelectFrame(AwAbstructFrame):

    def __init__(self, guimgr, launch):
        super(AwFileSelectFrame, self).__init__(guimgr, launch)

        button = QtWidgets.QPushButton("Browse")
        button.clicked.connect(self.browse)
        self.add_button(button)
        self.set_title(launch.nodename().capitalize())

        if self.launch.plugin.gui.get("list") is not True:
            self.widget = QtWidgets.QLineEdit()
            self.widget.setReadOnly(True)
            self.add_widget(self.widget)
        else:
            self.widget = QtWidgets.QTextEdit()
            self.widget.setReadOnly(True)
            self.add_widget(self.widget)

        self.config_updated()
        self.launch.bind_listener(self)
        self.destroyed.connect(lambda: self.launch.unbind_listener(self))

    def browse(self):

        import os
        if self.launch.plugin.gui.get("list") is not True:
            filename, filetype = QtWidgets.QFileDialog.getOpenFileName(self, "Select File", os.path.expanduser("~"))
        else:
            filename, filetype = QtWidgets.QFileDialog.getOpenFileNames(self, "Select Files", os.path.expanduser("~"))

        if filename:
            grp, key = self.launch.plugin.gui.get("data").split(".")
            self.launch.set_data(grp, key, filename)

    def config_updated(self):
        grp, key = self.launch.plugin.gui.get("data").split(".")
        data = self.launch.get_data(grp, key, "")
        if self.launch.plugin.gui.get("list") is True:
            data = "\n".join(data)
        self.widget.setText(data)








class AwLaunchButton(QtWidgets.QPushButton):

    def __init__(self, node):
        super(AwLaunchButton, self).__init__()
        self.node = node
        self.node.bind_listener(self) # need unbind in destructor
        self.setText("Launch")
        self.clicked.connect(self.on_clicked)

    def exec_requested(self):
        self.setText("Terminate")

    def term_requested(self):
        self.setEnabled(False)

    def term_completed(self):
        self.setText("Launch")
        self.setEnabled(True)

    # QtCore.Slot
    def on_clicked(self):
        state_text = self.text()
        if state_text == "Launch":
            self.node.request_exec()
        if state_text == "Terminate":
            self.node.request_term()

