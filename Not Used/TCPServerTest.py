import threading
import keyboard

# import socket


# TCP_IP = '127.0.0.1'
# TCP_PORT = 5005
# BUFFER_SIZE = 20  # Normally 1024, but we want fast response

# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.bind((TCP_IP, TCP_PORT))
# s.listen(1)
# while True:
#     conn, addr = s.accept()
#     print 'Connection address:', addr
#     while 1:
#         data = conn.recv(BUFFER_SIZE)
#         if not data:
#             break
#         print "received data:", data
#         conn.send(data)  # echo
#     conn.close()

# import SocketServer


# class EchoHandler(SocketServer.BaseRequestHandler):
#     def handle(self):
#         print "Connected from", self.client_address
#         data = ""
#         while True:
#             receivedData = self.request.recv(8192)
#             if not receivedData:
#                 break
#             data = data + receivedData
#             self.request.sendall(receivedData)
#         self.request.close()
#         print data
#         print "Disconnected from", self.client_address


# srv = SocketServer.ThreadingTCPServer(('127.0.0.1', 8881), EchoHandler)
# srv.serve_forever()


# import asyncore
# import asynchat
# import socket


# class MainServerSocket(asyncore.dispatcher):
#     def __init__(self, port):
#         print 'initing MSS'
#         asyncore.dispatcher.__init__(self)
#         self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
#         self.bind(('', port))
#         self.listen(5)

#     def handle_accept(self):
#         newSocket, address = self.accept()
#         print "Connected from", address
#         SecondaryServerSocket(newSocket)


# class SecondaryServerSocket(asynchat.async_chat):
#     def __init__(self, *args):
#         print 'initing SSS'
#         asynchat.async_chat.__init__(self, *args)
#         self.set_terminator('\n')
#         self.data = []

#     def collect_incoming_data(self, data):
#         self.data.append(data)

#     def found_terminator(self):
#         self.push(''.join(self.data))
#         self.data = []

#     def handle_close(self):
#         print "Disconnected from", self.getpeername()
#         self.close()


# def ServerSetup():
#     MainServerSocket(8881)
#     asyncore.loop()


# ServerSetup()


# class EchoHandler(SocketServer.BaseRequestHandler):
#     def handle(self):
#         print "Connected from", self.client_address
#         while True:
#             # receivedData = self.request.recv(8192)
#             # if not receivedData:
#             #     break
#             # print receivedData
#             self.request.sendall("{},{},{},{}".format(vehicle.home_location.lat, vehicle.home_location.lon, vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon))
#         self.request.close()
#         print "Disconnected from", self.client_address


# def TCPServerSetup():
#     srv = SocketServer.ThreadingTCPServer(('127.0.0.1', 8881), EchoHandler)
#     srv.serve_forever()


# TCPServer = Thread(target=TCPServerSetup)
# TCPServer.setDaemon(True)
# TCPServer.start()


from twisted.internet import reactor, protocol, task
from twisted.internet.protocol import Factory
import time

examplecoordinate = [2.1, 234.3336]


def readmission(aFileName):
    """
    Load a mission from a file into a list.
    """
    print "Reading mission from file: %s\n" % aFileName
    missionlist = []
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i == 0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray = line.split('\t')
                lat = float(linearray[8])  # Latitude of Waypoint
                lon = float(linearray[9])  # Longitude of Waypoint
                missionlist.append("{}@{}".format(lat, lon))
    return missionlist


def DataParser(string):
    """
    Parse data from service layer
    """
    stringarray = []
    distance = []
    heading = []
    bearing = []
    stringarray = string.split('@')
    altitude = map(int, stringarray[0])
    distance = map(float, stringarray[1][1:len(stringarray[1]) - 1].split('|'))
    heading = map(int, stringarray[2][1:len(stringarray[2]) - 1].split('|'))
    bearing = stringarray[3][1:len(stringarray[3]) - 1].split('|')
    print altitude
    print distance
    print heading
    print bearing


def HeaderExtractor(string):
    stringarray = []
    stringarray = string.split("://")
    return stringarray[0], stringarray[1]


def function():
    Echo.somevariable = False
    for i in range(5):
        print "blocking"
        time.sleep(1)
    Echo.somevariable = True


class Echo(protocol.Protocol):
    threads = []
    somevariable = True

    def connectionMade(self):
        waypoint = []
        waypoint = readmission("test.waypoints")

        def broadcast_msg():
            self.transport.write(",".join(str(x) for x in waypoint))
        self.looping_call = task.LoopingCall(broadcast_msg)
        self.looping_call.start(3)

    def dataReceived(self, data):
        header, info = HeaderExtractor(data)
        if info:
            DataParser(info)
            if header == "shutdown":
                self.transport.write("Server Shutdown Initiated")
                reactor.stop()
            elif header == "keyframe":
                self.transport.write(",".join(str(x) for x in examplecoordinate))
            elif header == "land":
                # To Be Handled
                self.transport.write("land")
            elif header == "PATH" and self.somevariable:
                Movement = threading.Thread(target=function)
                self.threads.append(Movement)
                Movement.start()
                print data


def ShutDown(*args):
    reactor.stop()
    keyboard.unhook('l')


factory = protocol.ServerFactory()
factory.protocol = Echo
reactor.listenTCP(8081, factory)
keyboard.hook_key('l', ShutDown)
reactor.run()
