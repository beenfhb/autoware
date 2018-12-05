from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets



class AwQuickStartWidget(QtWidgets.QWidget):

    def __init__(self, tree):
        super(AwQuickStartWidget, self).__init__()
        
        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(16)
        layout.addWidget(create_default_frame(tree.root))
        for node in tree.root.children:
            layout.addWidget(create_default_frame(node, True))
        layout.addStretch()

        self.setLayout(layout)



def create_default_frame(node, launch = None):

    header = create_default_frame_header(node.nodename, node if launch else None)
    header.setObjectName("FrameHeader")

    detail = create_default_frame_detail(node.setting["info"].get("title", "Content Area"))
    detail.setObjectName("FrameDetail")

    layout = QtWidgets.QVBoxLayout()
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(0)
    layout.addWidget(header)
    layout.addWidget(detail)

    widget = QtWidgets.QWidget()
    widget.setObjectName("FrameWidget")
    widget.setLayout(layout)
    widget.setStyleSheet("#FrameWidget { border: 1px solid; } #FrameHeader { padding: 5px; border-bottom: 1px solid; } #FrameDetail { padding: 5px; }")
    return widget


def create_default_frame_header(text, launch_node):

    title = QtWidgets.QLabel(text.capitalize())
    title.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)

    layout = QtWidgets.QHBoxLayout()
    layout.setContentsMargins(5, 2, 2, 2)
    layout.setSpacing(0)
    layout.addWidget(title)
    print launch_node
    if launch_node:
        layout.addWidget(AwLaunchButton(launch_node, "Launch"))

    widget = QtWidgets.QWidget()
    widget.setLayout(layout)
    return widget

def create_default_frame_detail(text):

    widget = QtWidgets.QLabel(text)
    return widget


class AwLaunchButton(QtWidgets.QPushButton):

    def __init__(self, node, text):
        super(AwLaunchButton, self).__init__(text)
        self.node = node
        self.node.bind_listener(self)
        self.setText("Launch")
        self.clicked.connect(self.on_clicked)
 
    # QtCore.Slot
    def on_clicked(self):
        state_text = self.text()
        if state_text == "Launch":
            self.node.request_exec()
        if state_text == "Terminate":
            self.node.request_term()

    def exec_requested(self):
        self.setText("Terminate")

    def term_requested(self):
        self.setEnabled(False)

    def term_completed(self):
        self.setText("Launch")
        self.setEnabled(True)



class AwDefaultFrame(QtWidgets.QWidget):

    @QtCore.Slot()
    def open_window(self):

        window = self.guimgr.create_window(self, self.config)
        window.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        window.setWindowModality(QtCore.Qt.ApplicationModal)
        window.destroyed.connect(lambda: self.window_closed.emit())
        window.show()
