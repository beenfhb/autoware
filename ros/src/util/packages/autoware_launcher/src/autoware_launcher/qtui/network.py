from python_qt_binding import QtWidgets
from python_qt_binding import QtNetwork

from ..core import AwLaunchClientIF

class AwTcpServerPanel(QtWidgets.QTextEdit, AwLaunchClientIF):

    def __init__(self):

        super(AwTcpServerPanel, self).__init__()
        self.server = None
        self.tcpsvr = QtNetwork.QTcpServer(self)
        self.connections = []
        self.setReadOnly(True)
        
        if self.tcpsvr.listen(QtNetwork.QHostAddress.Any, 33136):
            self.append("Server started")
            self.tcpsvr.newConnection.connect(self.on_new_connection)
        else:
            self.append("Server start error")

    def register_server(self, server):
        self.server = server

    def on_new_connection(self):

        socket = self.tcpsvr.nextPendingConnection()
        socket.string_buffer = ""
        self.connections.append(socket)
        self.append("New connection {} {}".format(socket.localAddress().toString(), socket.localPort()))

        socket.error.connect(self.on_client_error)
        #socket.disconnected.connect
        socket.readyRead.connect(self.on_client_read_ready)

    def on_client_read_ready(self):

        socket = self.sender()
        strings = str(socket.readAll()).split("\0")
        strings[0] = socket.string_buffer + strings[0]
        socket.string_buffer = strings.pop()
        for string in strings:
            self.process_request(string)

    def on_client_error(self):

        socket = self.sender()
        self.panel.write("Error " + socket.errorString())
        if socket in self.connections:
            socket.close()
            self.connections.remove(socket)

    def process_request(self, request):
        self.append("Request " + request)
        #response = self.server.request_json(string)
        #if response:
        #    socket.write(response)