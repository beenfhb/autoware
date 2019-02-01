from python_qt_binding import QtCore, QtWidgets
from autoware_launcher.core import myutils
from autoware_launcher.gui  import widgets



def plugin_widgets():
    return \
    {
        "bool"     : AwBoolEdit,
        "str"      : AwLineTextEdit,
        "strlist"  : AwTextListEdit,
        "topic"    : AwLineTextEdit,
        "frame"    : AwLineTextEdit,
        "int"      : AwLineTextEdit,
        "real"     : AwLineTextEdit,
        "tf"       : AwTransformEdit,
        "file"     : AwFileSelect,
        "filelist" : AwFileListSelect,
        "calib"    : AwCameraCalibFrame
    }



class AwLineTextEdit(widgets.AwAbstructFrame):

    def __init__(self, guimgr, node, opts):
        super(AwLineTextEdit, self).__init__(guimgr, node, opts)
        self.node = node
        self.opts = opts

        super(AwLineTextEdit, self).setup_widget()
        self.edit = QtWidgets.QLineEdit()
        self.edit.setText(self.node.get_config("args." + self.opts["defs"]["name"]))
        self.edit.editingFinished.connect(self.edited)
        self.add_widget(self.edit)
        self.set_title(self.opts["title"])

    def edited(self):
        cfgkey = "args." + self.opts["defs"]["name"]
        self.node.update({"config": {cfgkey: self.edit.text()}})

    @staticmethod
    def tostring(node, opts):
        return "{}: {}".format(opts["title"], node.get_config("args." + opts["defs"]["name"]))



class AwTextListEdit(widgets.AwAbstructFrame):

    def __init__(self, guimgr, node, opts):
        super(AwTextListEdit, self).__init__(guimgr, node, opts)
        self.node = node
        self.opts = opts

        super(AwTextListEdit, self).setup_widget()
        self.edit = QtWidgets.QLineEdit()
        self.edit.setText("/".join(self.node.get_config("args." + self.opts["defs"]["name"])))
        self.edit.editingFinished.connect(self.edited)
        self.add_widget(self.edit)
        self.set_title(self.opts["title"])

    def edited(self):
        cfgkey = "args." + self.opts["defs"]["name"]
        self.node.update({"config": {cfgkey: self.edit.text().split("/")}})

    @staticmethod
    def tostring(node, opts):
        return "{}: {}".format(opts["title"], "/".join(node.get_config("args." + opts["defs"]["name"])))



class AwBoolEdit(widgets.AwAbstructFrame):

    def __init__(self, guimgr, node, opts):
        super(AwBoolEdit, self).__init__(guimgr, node, opts)
        self.node = node
        self.opts = opts

        super(AwBoolEdit, self).setup_widget()
        self.edit = QtWidgets.QCheckBox()
        state = self.node.get_config("args." + self.opts["defs"]["name"])
        if state == "True":
            self.edit.setCheckState(QtCore.Qt.Checked)
        if state == "False":
            self.edit.setCheckState(QtCore.Qt.Unchecked)
        self.edit.setText(state)
        self.edit.stateChanged.connect(self.edited)
        self.add_widget(self.edit)
        self.set_title(self.opts["title"])

    def edited(self, state):
        print "edited" + str(state)
        cfgkey = "args." + self.opts["defs"]["name"]
        if state == QtCore.Qt.Checked:
            self.node.update({"config": {cfgkey: "True"}})
        if state == QtCore.Qt.Unchecked:
            self.node.update({"config": {cfgkey: "False"}})

    @staticmethod
    def tostring(node, opts):
        return "{}: {}".format(opts["title"], node.get_config("args." + opts["defs"]["name"]))



class AwFileSelect(widgets.AwAbstructFrame):

    def __init__(self, guimgr, node, opts):
        super(AwFileSelect, self).__init__(guimgr, node, opts)
        self.node = node
        self.opts = opts

        super(AwFileSelect, self).setup_widget()
        button = QtWidgets.QPushButton("Browse")
        button.clicked.connect(self.browsed)
        self.add_button(button)
        self.set_title(self.opts["title"])

        self.widget = QtWidgets.QLineEdit()
        self.widget.setReadOnly(True)
        self.add_widget(self.widget)
        self.widget.setText(self.node.get_config("args." + self.opts["defs"]["name"]))

    def browsed(self):
        filepath, filetype = QtWidgets.QFileDialog.getOpenFileName(self, "Select File", myutils.userhome())
        filepath = myutils.envpath(filepath)
        if filepath:
            cfgkey = "args." + self.opts["defs"]["name"]
            self.node.update({"config": {cfgkey: filepath}})
            self.widget.setText(filepath)

    @staticmethod
    def tostring(node, opts):
        return "{}: {}".format(opts["title"], node.get_config("args." + opts["defs"]["name"]))



class AwFileListSelect(widgets.AwAbstructFrame):

    def __init__(self, guimgr, node, opts):
        super(AwFileListSelect, self).__init__(guimgr, node, opts)
        self.node = node
        self.opts = opts

        super(AwFileListSelect, self).setup_widget()
        button = QtWidgets.QPushButton("Browse")
        button.clicked.connect(self.browsed)
        self.add_button(button)
        self.set_title(self.opts["title"])

        self.widget = QtWidgets.QTextEdit()
        self.widget.setReadOnly(True)
        self.add_widget(self.widget)

        filepaths = self.node.get_config("args." + self.opts["defs"]["name"], [])
        self.widget.setText("\n".join(filepaths))

    def browsed(self):
        filepaths, filetype = QtWidgets.QFileDialog.getOpenFileNames(self, "Select Files", myutils.userhome())
        filepaths = map(myutils.envpath, filepaths)
        if filepaths:
            cfgkey = "args." + self.opts["defs"]["name"]
            self.node.update({"config": {cfgkey: filepaths}})
            self.widget.setText("\n".join(filepaths))

    @staticmethod
    def tostring(node, opts):
        return "{}: {} files".format(opts["title"], len(node.get_config("args." + opts["defs"]["name"])))



class AwTransformEdit(widgets.AwAbstructFrame):

    def __init__(self, guimgr, node, opts):
        super(AwTransformEdit, self).__init__(guimgr, node, opts)
        self.node = node
        self.opts = opts
        self.fields = []

        super(AwTransformEdit, self).setup_widget()
        widget = QtWidgets.QWidget()
        widget.setLayout(QtWidgets.QHBoxLayout())

        mapper = QtCore.QSignalMapper(widget)
        for idx, txt in enumerate(["Tx", "Ty", "Tz", "Rx", "Ry", "Rz"]):
            field = QtWidgets.QLineEdit()
            field.setText(self.node.get_config("args." + self.opts["defs"][idx]["name"]))
            field.editingFinished.connect(mapper.map)
            mapper.setMapping(field, idx)
            widget.layout().addWidget(QtWidgets.QLabel(txt + ":"))
            widget.layout().addWidget(field)
            self.fields.append(field)

        mapper.mapped.connect(self.edited)
        self.add_widget(widget)
        self.set_title(self.opts["title"])

    def edited(self, idx):
        cfgkey = "args." + self.opts["defs"][idx]["name"]
        self.node.update({"config": {cfgkey: self.fields[idx].text()}})

    @staticmethod
    def tostring(node, opts):
        result = opts["title"] + ": "
        for idx, txt in enumerate(["Tx", "Ty", "Tz", "Rx", "Ry", "Rz"]):
            result += txt + "=" + node.get_config("args." + opts["defs"][idx]["name"]) + ", "
        return result



class AwCameraCalibFrame(AwFileSelect):

    def __init__(self, guimgr, node, opts):
        super(AwCameraCalibFrame, self).__init__(guimgr, node, opts)

        calib = QtWidgets.QPushButton("Calib")
        calib.setCheckable(True)
        calib.toggled.connect(self.calibrate)
        self.add_button(calib)

    def refresh_image_topic(self):
        from subprocess import Popen, PIPE
        command = "rostopic find sensor_msgs/Image"
        process = Popen(command.split(), stdout=PIPE, stderr=PIPE)
        stdout, stderr = process.communicate()
        self.topic_name.clear()
        for topic in stdout.split("\n"):
            if topic: self.topic_name.addItem(topic)

    def calibrate(self, checked):

        self.intrinsic_calibrator = QtCore.QProcess(self)
        self.extrinsic_calibrator = QtCore.QProcess(self)
        self.corner_size_x    = QtWidgets.QLineEdit("8")
        self.corner_size_y    = QtWidgets.QLineEdit("6")
        self.square_length    = QtWidgets.QLineEdit("0.1")
        self.topic_name       = QtWidgets.QComboBox()
        self.topic_refresh    = QtWidgets.QPushButton("Refresh")
        self.intrinsic_file   = QtWidgets.QLineEdit()
        self.intrinsic_browse = QtWidgets.QPushButton("Browse")
        self.calib_intrinsic  = QtWidgets.QPushButton("Calibrate Intrinsic")
        self.calib_extrinsic  = QtWidgets.QPushButton("Calibrate Extrinsic")

        self.refresh_image_topic()
        self.topic_refresh.clicked.connect(self.refresh_image_topic)
        self.intrinsic_browse.clicked.connect(self.select_intrinsic)

        self.calib_intrinsic.setCheckable(True)
        self.calib_intrinsic.toggled.connect(self.calibrate_intrinsic)
        self.calib_extrinsic.setCheckable(True)
        self.calib_extrinsic.toggled.connect(self.calibrate_extrinsic)

        widget = QtWidgets.QWidget()
        widget.setLayout(QtWidgets.QGridLayout())
        widget.layout().addWidget(QtWidgets.QLabel("Square Corners"), 0, 0)
        widget.layout().addWidget(self.corner_size_x,                 0, 1)
        widget.layout().addWidget(QtWidgets.QLabel("x"),              0, 2)
        widget.layout().addWidget(self.corner_size_y,                 0, 3)
        widget.layout().addWidget(QtWidgets.QLabel("Square Length"),  0, 4)
        widget.layout().addWidget(self.square_length,                 0, 5)
        widget.layout().addWidget(QtWidgets.QLabel("Image Topic"),    1, 0)
        widget.layout().addWidget(self.topic_name,                    1, 1, 1, 4)
        widget.layout().addWidget(self.topic_refresh,                 1, 5)
        widget.layout().addWidget(QtWidgets.QLabel("Intrinsic File"), 2, 0)
        widget.layout().addWidget(self.intrinsic_file,                2, 1, 1, 4)
        widget.layout().addWidget(self.intrinsic_browse,              2, 5)
        widget.layout().addWidget(self.calib_intrinsic,               3, 0)
        widget.layout().addWidget(self.calib_extrinsic,               3, 1)

        window = QtWidgets.QMainWindow(self)
        window.setCentralWidget(widget)
        window.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        window.setWindowModality(QtCore.Qt.ApplicationModal)
        window.show()

    def select_intrinsic(self):
        import os
        filename, filetype = QtWidgets.QFileDialog.getOpenFileName(self, "Select File", os.path.expanduser("~"))
        if filename:
            self.intrinsic_file.setText(filename)
      
    def calibrate_intrinsic(self, checked):
        if checked:
            command = "rosrun autoware_camera_lidar_calibrator cameracalibrator.py --square {} --size {}x{} image:={}"
            command = command.format(self.square_length.text(), self.corner_size_x.text(), self.corner_size_y.text(), self.topic_name.currentText())
            self.intrinsic_calibrator.start(command )
        else:
            self.intrinsic_calibrator.terminate()

    def calibrate_extrinsic(self, checked):
        if checked:
            command = "roslaunch autoware_camera_lidar_calibrator camera_lidar_calibration.launch intrinsics_file:={} image_src:={}"
            command = command.format(self.intrinsic_file.text(), self.topic_name.currentText())
            self.extrinsic_calibrator.start(command )
        else:
            self.extrinsic_calibrator.terminate()