import bluetooth
import lab2_utils
import uuid
import threading
import sys
import time
import picar_4wd as fc

#uuid = str(uuid.uuid4())
uuidDrive = "278f1937-9a6e-4bef-ac3e-759f9fff8d9a"
uuidStats = "eb106f30-9dd2-4eb8-91c8-276495778f30"

def start_server():
    hostMACAddress = "AA:AA:AA:AA:AA:AA"
    #port = 4
    backlog = 1
    size = 1024
    s = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    #s.bind((hostMACAddress, bluetooth.PORT_ANY))
    s.bind(("", bluetooth.PORT_ANY))
    s.listen(backlog)
    port = s.getsockname()[1]
    print("listening on port ", port)

    bluetooth.advertise_service(s, "PiServer", service_id=uuidDrive,
    service_classes=[uuidDrive,bluetooth.SERIAL_PORT_CLASS], 
    profiles=[bluetooth.SERIAL_PORT_PROFILE]
    # protocols = [bluetooth.OBEX_UUID]
    )

    try:
        client, clientInfo = s.accept()
        while 1:   
            print("server recv from: ", clientInfo)
            data = client.recv(size)
            stringdata = data.decode('utf-8')
            if data:
                if stringdata == "8":
                    print("moving forward")
                    lab2_utils.goForward()
                elif stringdata == "5":
                    print("moving backwards")
                    lab2_utils.goBackwards()
                elif stringdata == "4":
                    print("going left")
                    lab2_utils.goLeft()
                elif stringdata == "6":
                    print("going right")
                    lab2_utils.goRight()
                elif stringdata == "0":
                    print("stopping")
                    lab2_utils.stop()
                else:
                    print("invalid")
                    print(data)
                    client.send(data) # Echo back to client
    except: 
        print("Closing socket")
        client.close()
        s.close()

def start_client():
    host = "A4:B1:C1:6B:45:03" # The address of Raspberry PI Bluetooth adapter on the server.
    port = 3
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    sock.connect((host, port))

    print("connected. Type something...")

    while True:
        data = "distance to nearest obstacle: " + str(fc.get_distance_at(0))
        time.sleep(5)
        sock.send(data)
    sock.close()

sth = threading.Thread(target=start_server)
cth = threading.Thread(target=start_client)

sth.start()
cth.start()

cth.join()
sth.join()

print ("Success, terminating")