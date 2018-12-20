from python_qt_binding import QtCore
from python_qt_binding import QtWidgets

from .abstruct import AwAbstructWindow
from .abstruct import AwAbstructPanel
from .abstruct import AwAbstructFrame



class AwTextTypeFrame(AwAbstructFrame):

    def __init__(self, guimgr, target, option):
        super(AwTextTypeFrame, self).__init__(guimgr, None)
        self.target = target
        self.option = option

        super(AwTextTypeFrame, self).setup_widget()
        self.edit = QtWidgets.QLineEdit()
        self.edit.setText(self.target.config.get("args." + self.option["args"]))
        self.edit.editingFinished.connect(self.edited)
        self.add_widget(self.edit)
        self.set_title(self.option["title"])

    def edited(self):
        self.target.config["args." + self.option["args"]] = self.edit.text()



class AwFileTypeFrame(AwAbstructFrame):

    def __init__(self, guimgr, target, option):
        super(AwFileTypeFrame, self).__init__(guimgr, None)
        self.target = target
        self.option = option

        super(AwFileTypeFrame, self).setup_widget()
        button = QtWidgets.QPushButton("Browse")
        button.clicked.connect(self.browsed)
        self.add_button(button)
        self.set_title(self.option["title"])

        if self.option.get("multi_select") is not True:
            self.widget = QtWidgets.QLineEdit()
            self.widget.setReadOnly(True)
            self.add_widget(self.widget)
        else:
            self.widget = QtWidgets.QTextEdit()
            self.widget.setReadOnly(True)
            self.add_widget(self.widget)

    def browsed(self):
        import os, re
        if self.option.get("multi_select") is not True:
            filename, filetype = QtWidgets.QFileDialog.getOpenFileName(self, "Select File", os.path.expanduser("~"))
            filename = re.sub("^" + os.environ['HOME'], "$(env HOME)", filename)
        else:
            filename, filetype = QtWidgets.QFileDialog.getOpenFileNames(self, "Select Files", os.path.expanduser("~"))
            filename = map(lambda v: re.sub("^" + os.environ['HOME'], "$(env HOME)", v), filename)

        if filename:
            self.target.config["args." + self.option["args"]] = filename
            if self.option.get("multi_select") is not True:
                self.widget.setText(filename)
            else:
                self.widget.setText("\n".join(filename))



class AwTransformFrame(AwAbstructFrame):

    def __init__(self, guimgr, target, option):
        super(AwTransformFrame, self).__init__(guimgr, None)
        self.target = target
        self.option = option
        self.fields = []

        super(AwTransformFrame, self).setup_widget()
        widget = QtWidgets.QWidget()
        widget.setLayout(QtWidgets.QHBoxLayout())

        mapper = QtCore.QSignalMapper(widget)
        for idx, txt in enumerate({"Tx", "Ty", "Tz", "Rx", "Ry", "Rz"}):
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



class AwCameraCalibFrame(AwFileTypeFrame):

    def __init__(self, guimgr, target, option):
        super(AwCameraCalibFrame, self).__init__(guimgr, target, option)

        calib = QtWidgets.QPushButton("Calib")
        calib.clicked.connect(self.calibrate_intrinsic)
        self.add_button(calib)
    
    def calibrate_intrinsic(self):
        import subprocess
        command = "rosrun autoware_camera_lidar_calibrator cameracalibrator.py --square 0.1 --size 8x6 image:=/camera0/image_raw"
        subprocess.call(command.split())