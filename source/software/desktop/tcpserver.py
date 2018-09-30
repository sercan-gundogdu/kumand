import socket
import binascii
import pyautogui

HOST = socket.gethostbyname(socket.gethostname())                 # Symbolic name meaning all available interfaces
PORT = 5007              # Arbitrary non-privileged port
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))

keys = {}
profile = ""

def getKey():
    while True:
        s.listen(1)
        conn, addr = s.accept()
        #print ("Connected by" + str(addr))
        while True:
            data = conn.recv(1024)
            if not data: break
            pressKey(str(binascii.hexlify(data))[2:10])
            conn.sendall("OK\0".encode('utf-8'))
        conn.close()


def pressKey(data):
    global keys
    global profile

    file = open("./globals/profile", "r")
    p = file.read()
    if profile != p:
        profile = p
        file.close()
        with open("./profiles/" + profile, "r") as file:
            keys = dict([line.split() for line in file])
        file.close()
    else:
        file.close()

    print (data)
    key = str(keys.get(data))
    
    if key != 'null':
        if ("+" in key):
            k = key.split("+")
            if len(k) == 2:
                pyautogui.hotkey(k[0],k[1])
            if len(k) == 3:
                pyautogui.hotkey(k[0],k[1],k[3])
            else:
                pass
        else:
            pyautogui.press(key)
    else:
        pass
        

if __name__ == '__main__':
    getKey()
