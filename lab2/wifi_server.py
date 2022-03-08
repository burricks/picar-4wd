import socket
import lab2_utils
#import picar_4wd as fc

HOST = "192.168.4.131" # IP address of your Raspberry PI
PORT = 65432          # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    client, clientInfo = s.accept()

    try:
        while 1:
            carstats = lab2_utils.getStats()
            client.send(carstats.encode('utf-8')) # Echo back to client
            data = client.recv(1024) 
            stringdata = data.decode('utf-8')
            if data:
                if stringdata == "38":
                    print("moving forward")
                    lab2_utils.goForward()
                elif stringdata == "40":
                    print("moving backwards")
                    lab2_utils.goBackwards()
                elif stringdata == "37":
                    print("going left")
                    lab2_utils.goLeft()
                elif stringdata == "39":
                    print("going right")
                    lab2_utils.goRight()
                elif stringdata == "0":
                    print("stopping")
                    lab2_utils.stop()
                else:
                    print("invalid")
                    print(data)
    except: 
        print("Closing socket")
        client.close()
        s.close()    