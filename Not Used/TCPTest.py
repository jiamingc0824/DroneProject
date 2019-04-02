# import socket
# import time

# TCP_IP = '127.0.0.1'
# TCP_PORT = 8881
# BUFFER_SIZE = 1024
# MESSAGE = "Hello, World!"
# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect((TCP_IP, TCP_PORT))
# while True:
#     s.send(MESSAGE)
#     data = s.recv(BUFFER_SIZE)
#     print "received data:", data
#     time.sleep(2)
# s.close()

import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    sock.connect(('localhost', 8080))
    print "Drone Init Completed, Notifying Service Layer"
    data = "droneserver"
    sock.sendall(data)
    sock.close()
except Exception as e:
    print e
