from python_qt_binding import QtNetwork

class AwTcpServer(QtNetwork.QTcpServer):

    def __init__(self, tree, port = 33136):

        super(AwTcpServer, self).__init__()
        self.tree = tree
        self.port = port
        self.connections = []
        
        if self.listen(QtNetwork.QHostAddress.Any, port):
            print "server start"
            self.newConnection.connect(self.on_new_connection)
        else:
            print "server start error"

    def on_new_connection(self):

        socket = self.nextPendingConnection()
        socket.string_buffer = ""
        self.connections.append(socket)

        socket.error.connect(self.on_client_error)
        #socket.disconnected.connect
        socket.readyRead.connect(self.on_client_read_ready)

    def on_client_read_ready(self):

        socket = self.sender()
        strings = str(socket.readAll()).split("\0")
        strings[0] = socket.string_buffer + strings[0]
        socket.string_buffer = strings.pop()
        for string in strings:
            response = self.tree.request_json(string)
            if response:
                socket.write(response)

    def on_client_error(self):

        socket = self.sender()
        print socket.errorString()
        if socket in self.connections:
            socket.close()
            self.connections.remove(socket)



from autoware_launcher.core import AwLaunchTree
if __name__ == "__main__":
    pass
