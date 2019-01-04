from python_qt_binding import QtCore, QtWidgets
from autoware_launcher.core import fspath
from autoware_launcher.qtui import widgets



def plugin_widgets():
    return \
    {
        "str"      : AwLineTextEdit,
        "topic"    : AwLineTextEdit,
        "frame"    : AwLineTextEdit,
        "tf"       : AwTransformEdit,
        "file"     : AwFileSelect,
        "filelist" : AwFileListSelect
    }



class AwLineTextEdit(widgets.AwAbstructFrame):

    def __init__(self, guimgr, target, option):
        super(AwLineTextEdit, self).__init__(guimgr, None)
        self.target = target
        self.option = option

        super(AwLineTextEdit, self).setup_widget()
        self.edit = QtWidgets.QLineEdit()
        self.edit.setText(self.target.config.get("args." + self.option["args"]))
        self.edit.editingFinished.connect(self.edited)
        self.add_widget(self.edit)
        self.set_title(self.option["title"])

    def edited(self):
        self.target.config["args." + self.option["args"]] = self.edit.text()

    @staticmethod
    def summary(option, config):
        return "{}: {}".format(option["title"], config["args." + option["args"]])



class AwFileSelect(widgets.AwAbstructFrame):

    def __init__(self, guimgr, target, option):
        super(AwFileSelect, self).__init__(guimgr, None)
        self.target = target
        self.option = option

        super(AwFileSelect, self).setup_widget()
        button = QtWidgets.QPushButton("Browse")
        button.clicked.connect(self.browsed)
        self.add_button(button)
        self.set_title(self.option["title"])

        self.widget = QtWidgets.QLineEdit()
        self.widget.setReadOnly(True)
        self.add_widget(self.widget)
        self.widget.setText(self.target.config.get("args." + self.option["args"]))

    def browsed(self):
        filepath, filetype = QtWidgets.QFileDialog.getOpenFileName(self, "Select File", fspath.userhome())
        filepath = fspath.envpath(filepath)
        if filepath:
            self.target.config["args." + self.option["args"]] = filepath
            self.widget.setText(filepath)

    @staticmethod
    def summary(option, config):
        return "{}: {}".format(option["title"], config["args." + option["args"]])



class AwFileListSelect(widgets.AwAbstructFrame):

    def __init__(self, guimgr, target, option):
        super(AwFileListSelect, self).__init__(guimgr, None)
        self.target = target
        self.option = option

        super(AwFileListSelect, self).setup_widget()
        button = QtWidgets.QPushButton("Browse")
        button.clicked.connect(self.browsed)
        self.add_button(button)
        self.set_title(self.option["title"])

        self.widget = QtWidgets.QTextEdit()
        self.widget.setReadOnly(True)
        self.add_widget(self.widget)

        filepaths = self.target.config.get("args." + self.option["args"], [])
        self.widget.setText("\n".join(filepaths))

    def browsed(self):
        filepaths, filetype = QtWidgets.QFileDialog.getOpenFileNames(self, "Select Files", fspath.userhome())
        filepaths = map(fspath.envpath, filepaths)
        if filepaths:
            self.target.config["args." + self.option["args"]] = filepaths
            self.widget.setText("\n".join(filepaths))

    @staticmethod
    def summary(option, config):
        return "{}: {} files".format(option["title"], len(config["args." + option["args"]]))



class AwTransformEdit(widgets.AwAbstructFrame):

    def __init__(self, guimgr, target, option):
        super(AwTransformEdit, self).__init__(guimgr, None)
        self.target = target
        self.option = option
        self.fields = []

        super(AwTransformEdit, self).setup_widget()
        widget = QtWidgets.QWidget()
        widget.setLayout(QtWidgets.QHBoxLayout())

        mapper = QtCore.QSignalMapper(widget)
        for idx, txt in enumerate(["Tx", "Ty", "Tz", "Rx", "Ry", "Rz"]):
            field = QtWidgets.QLineEdit()
            field.setText(self.target.config.get("args." + self.option["args"][idx]))
            field.editingFinished.connect(mapper.map)
            mapper.setMapping(field, idx)
            widget.layout().addWidget(QtWidgets.QLabel(txt + ":"))
            widget.layout().addWidget(field)
            self.fields.append(field)

        mapper.mapped.connect(self.edited)
        self.add_widget(widget)
        self.set_title(self.option["title"])

    def edited(self, idx):
        self.target.config["args." + self.option["args"][idx]] = self.fields[idx].text()

    @staticmethod
    def summary(option, config):
        result = option["title"] + ": "
        for idx, txt in enumerate(["Tx", "Ty", "Tz", "Rx", "Ry", "Rz"]):
            result += txt + "=" + config["args." + option["args"][idx]] + ", "
        return result



class AwCameraCalibFrame(AwFileSelect):

    def __init__(self, guimgr, target, option):
        super(AwCameraCalibFrame, self).__init__(guimgr, target, option)

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