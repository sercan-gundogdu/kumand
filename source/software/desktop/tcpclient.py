import socket
import finddevice

def Main():
    host = finddevice.getDeviceAddr()
    print("Device found")
    #host = "192.168.1.40"
    port = 500
    
    s = socket.socket()
    s.connect((host, port))
    ip = socket.gethostbyname(socket.gethostname()) + ":5007\0"

    s.send(ip.encode("utf-8"))
    print (s.recv(32))
    #data = str(s.recv(32))
    s.close()

if __name__ == '__main__':
    Main()
