from python_qt_binding import QtCore
from python_qt_binding import QtNetwork
from python_qt_binding import QtGui
from python_qt_binding import QtWidgets
from ..core import console
from ..core import fspath

from .widgets import AwFileSelect

class AwRosbagSimulatorWidget(QtWidgets.QWidget):

    def __init__(self, guimgr):
        super(AwRosbagSimulatorWidget, self).__init__()
        self.rosbag_mode_proc = QtCore.QProcess(self)
        self.rosbag_info_proc = QtCore.QProcess(self)
        self.rosbag_play_proc = QtCore.QProcess(self)

        self.rosbag_file   = AwFileSelect(self)
        self.rosbag_info   = QtWidgets.QPushButton("Info")
        self.rosbag_text   = QtWidgets.QLabel("No information")
        self.rosbag_enable = QtWidgets.QCheckBox()
        self.rosbag_label  = QtWidgets.QLabel("Simulation Mode")
        self.rosbag_play   = QtWidgets.QPushButton("Play")
        self.rosbag_stop   = QtWidgets.QPushButton("Stop")
        self.rosbag_pause  = QtWidgets.QPushButton("Pause")
        self.rosbag_state  = QtWidgets.QLabel()
        #start time
        #repeat
        #rate

        self.rosbag_enable.stateChanged.connect(self.simulation_mode_changed)
        self.rosbag_info.clicked.connect(self.rosbag_info_requested)
        self.rosbag_info_proc.finished.connect(self.rosbag_info_completed)

        self.rosbag_play.clicked.connect(self.rosbag_started)
        self.rosbag_stop.clicked.connect(self.rosbag_stopped)
        self.rosbag_play_proc.finished.connect(self.rosbag_finished)
        self.rosbag_play_proc.readyReadStandardOutput.connect(self.rosbag_output)

        self.rosbag_pause.setCheckable(True)
        self.rosbag_pause.toggled.connect(self.rosbag_paused)

        self.setStyleSheet("QCheckBox::indicator { width: 28px; height: 28px; }")
        self.rosbag_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        self.rosbag_text.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
        
        layout = QtWidgets.QGridLayout()
        layout.addWidget(self.rosbag_enable,      0, 0)
        layout.addWidget(self.rosbag_label,       0, 1)
        layout.addWidget(self.rosbag_play,        0, 2)
        layout.addWidget(self.rosbag_stop,        0, 3)
        layout.addWidget(self.rosbag_pause,       0, 4)
        layout.addWidget(self.rosbag_state,       1, 0, 1, 5)
        layout.addWidget(self.rosbag_file.path,   2, 0, 1, 3)
        layout.addWidget(self.rosbag_file.button, 2, 3)
        layout.addWidget(self.rosbag_info,        2, 4)
        layout.addWidget(self.rosbag_text,        3, 0, 1, 5)
        self.setLayout(layout)
        self.simulation_mode_disabled()

    def simulation_mode_changed(self, state):
        if state == QtCore.Qt.Checked:   self.simulation_mode_enabled()
        if state == QtCore.Qt.Unchecked: self.simulation_mode_disabled()

    def simulation_mode_enabled(self):
        self.rosbag_mode_proc.start("rosparam set /use_sim_time true")
        self.rosbag_stopped()

    def simulation_mode_disabled(self):
        self.rosbag_mode_proc.start("rosparam set /use_sim_time false")
        self.rosbag_stopped()
        self.rosbag_play.setEnabled(False)

    def rosbag_info_requested(self):
        self.rosbag_info_proc.start("rosbag info " + self.rosbag_file.path.text())

    def rosbag_info_completed(self):
        stdout = self.rosbag_info_proc.readAllStandardOutput().data().decode('utf-8')
        stderr = self.rosbag_info_proc.readAllStandardError().data().decode('utf-8')
        self.rosbag_text.setText(stdout + stderr)

    def rosbag_started(self):
        xml = fspath.package("launch/rosbagplay.xml")
        arg = self.rosbag_file.path.text()
        self.rosbag_play_proc.start("roslaunch {} bagfile:={}".format(xml, arg))
        self.rosbag_play_proc.processId()
        self.rosbag_play.setEnabled(False)
        self.rosbag_stop.setEnabled(True)
        self.rosbag_pause.setEnabled(True)

    def rosbag_stopped(self):
        self.rosbag_play_proc.terminate()
        self.rosbag_finished()
    
    def rosbag_finished(self):
        self.rosbag_play.setEnabled(True)
        self.rosbag_stop.setEnabled(False)
        self.rosbag_pause.setEnabled(False)
        self.rosbag_pause.setChecked(False)

    def rosbag_paused(self, checked):
        self.rosbag_play_proc.write(" ")

    def rosbag_output(self):
        #print self.rosbag_play_proc.readAllStandardOutput().data().decode('utf-8')
        stdout = str(self.rosbag_play_proc.readAllStandardOutput()).split("\r")
        if 2 <= len(stdout):
            self.rosbag_state.setText(stdout[-2])



class AwLgsvlSimulatorWidget(QtWidgets.QWidget):

    def __init__(self, guimgr):
        super(AwLgsvlSimulatorWidget, self).__init__()
        self.bridge_process = QtCore.QProcess(self)
        self.bridge_console = AwProcessViewer(self.bridge_process)
        self.bridge_button  = QtWidgets.QPushButton("Launch Bridge Server")
        self.bridge_button.setCheckable(True)
        self.bridge_button.toggled.connect(self.launch_bridge)

        #self.lgsvl_manager = QtNetwork.QNetworkAccessManager(self)
        self.lgsvl_button  = QtWidgets.QPushButton("Launch Simulator")
        self.lgsvl_button.clicked.connect(self.launch_lgsvl)

        #self.lgsvl_server_addr = QtWidgets.QLineEdit()
        #self.lgsvl_server_port = QtWidgets.QLineEdit()
        #self.lgsvl_server_addr.setText("10.100.2.1")
        #self.lgsvl_server_port.setText("5000")

        self.lgsvl_client_addr = QtWidgets.QLineEdit()
        self.lgsvl_client_port = QtWidgets.QLineEdit()
        for host in QtNetwork.QNetworkInterface.allAddresses():
            if not host.isLoopback():
                if host.protocol() == QtNetwork.QAbstractSocket.IPv4Protocol:
                    self.lgsvl_client_addr.setText(host.toString())
        self.lgsvl_client_port.setText("9090")

        layout = QtWidgets.QGridLayout()
        layout.addWidget(QtWidgets.QLabel("Address"), 0, 0)
        layout.addWidget(QtWidgets.QLabel("Port"),    1, 0)
        layout.addWidget(self.lgsvl_client_addr, 0, 1)
        layout.addWidget(self.lgsvl_client_port, 1, 1)
        layout.addWidget(self.bridge_button,  2, 0, 1, 2)
        layout.addWidget(self.lgsvl_button,   3, 0, 1, 2)
        #layout.setRowStretch(4, 1)
        layout.addWidget(self.bridge_console, 4, 0, 1, 2)
        self.setLayout(layout)

    def launch_bridge(self, checked):
        if checked:
            self.bridge_process.start("roslaunch rosbridge_server rosbridge_websocket.launch")
        else:
            self.bridge_process.terminate()

    def launch_lgsvl(self):

        lgsvl_param = {
            "bin_type": "tier4-develop",
            "initial_configuration": {
                "map": "SanFrancisco",
                "time_of_day": 12,
                "freeze_time_of_day": True,
                "fog_intensity": 0,
                "rain_intensity": 0,
                "road_wetness": 0,
                "enable_traffic": True,
                "enable_pedestrian": True,
                "traffic_density": 300
            },
            "vehicles": [
                {
                    "type": "XE_Rigged-autoware",
                    "address": "10.254.1.60",
                    "port": 9090,
                    "command_type": "twist",
                    "enable_lidar": True,
                    "enable_gps": True,
                    "enable_main_camera": True,
                    #"enable_high_quality_rendering": True,
                    "enable_high_quality_rendering": False,
                    "position": {
                        "n": 4140310.4,
                        "e": 590681.5,
                        "h": 10
                    },
                    "orientation": {
                        "r": 0,
                        "p": 0,
                        "y": 269.9
                    }
                }
            ]
        }

        import requests
        try:
            responce = requests.post('http://10.100.2.1:5000/simulator/launch', json=lgsvl_param)
            print responce.status_code
            print responce.json()
            responce = requests.post('http://10.100.2.1:5000/simulator/terminate', json={"instance_id": responce.json()["instance_id"]})
            print responce.status_code
            print responce.json()
        except:
            print "Request Error"



class AwProcessViewer(QtWidgets.QPlainTextEdit):

    def __init__(self, process):
        super(AwProcessViewer, self).__init__()
        
        self.setReadOnly(True)
        self.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        #self.setMaximumBlockCount(100)
        self.moveCursor(QtGui.QTextCursor.End)

        self.process = process
        self.process.finished.connect(self.process_finished)
        self.process.readyReadStandardOutput.connect(self.process_stdouted)
        self.process.readyReadStandardError.connect(self.process_stderred)

        import re
        self.bash_regex = re.compile("(\x1b\[.*?m|\x1b\]2;|\x07)")

    def byte2text(self, byte):
        #text = self.bash_regex.sub("", byte.data().decode('utf-8'))
        text = QtCore.QTextStream(byte).readAll()
        text = self.bash_regex.sub("", text)
        #text = str(text).encode("string-escape").replace("\\n", "\n")
        return text

    def process_finished(self):
        pass

    def process_stdouted(self):
        text = self.byte2text(self.process.readAllStandardOutput())
        self.insertPlainText(text)
        self.moveCursor(QtGui.QTextCursor.End)

    def process_stderred(self):
        text = self.byte2text(self.process.readAllStandardError())
        self.insertPlainText(text)
        self.moveCursor(QtGui.QTextCursor.End)

    # Debug
    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_C:
            self.clear()
            event.accept()
        else:
            super(AwProcessViewer, self).keyPressEvent(event)