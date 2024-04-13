import numpy as np
import cv2
import socket
import struct

clientsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
clientsocket.connect(('localhost', 3005))

SCALE_AMOUNT = 4

def recvImg():
    hdr = clientsocket.recv(8)
    magic, length = struct.unpack("!II", hdr)
    if magic != 0xA55A0FF0:
        raise RuntimeError("Invalid Magic: " + hex(magic))
    imgdata = clientsocket.recv(length)
    imarr = np.frombuffer(imgdata, np.uint8)
    contentarr = np.frombuffer(imarr, np.uint8)
    rxImg = cv2.imdecode(contentarr, cv2.IMREAD_COLOR)
    return cv2.resize(rxImg, (0, 0), fx = SCALE_AMOUNT, fy = SCALE_AMOUNT)

while True:
    # Using cv2.imshow() method
    # Displaying the image
    cv2.imshow("Underwater Camera", recvImg())

    # waits for user to press any key
    # (this is necessary to avoid Python kernel form crashing)
    if cv2.waitKey(10) == ord('q'):
        break

# closing all open windows
cv2.destroyAllWindows()
