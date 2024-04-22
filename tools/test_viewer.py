import socket
import struct

class CanmoreImageSocketListener:
    MAX_INTERFRAME_TIME_SEC = 5.0

    CTRL_TYPE_ENABLE = 1
    CTRL_TYPE_QUALITY = 2
    CTRL_TYPE_KEYPRESS = 3
    CTRL_TYPE_STREAM_SELECT = 4
    CTRL_TYPE_MAX_DIMENSION = 5

    def __init__(self, host, port=3005):
        self.clientsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.clientsocket.connect((host, port))
        self.clientsocket.settimeout(self.MAX_INTERFRAME_TIME_SEC)

    def _recvLength(self, packet_length) -> bytearray:
        data = bytearray()
        prev_len = 0
        while prev_len != packet_length:
            fragment = self.clientsocket.recv(packet_length - prev_len)
            if len(fragment) == 0:
                raise RuntimeError("Socket Closed")
            data += fragment
            prev_len = len(data)
        return data

    def recvJpeg(self) -> bytearray:
        try:
            hdr = self._recvLength(8)
            magic, length = struct.unpack("!II", hdr)
            if magic != 0xA55A0FF0:
                raise RuntimeError("Invalid Magic: " + hex(magic))
            return self._recvLength(length)
        except socket.timeout:
            return None

    def _sendControlPacket(self, msg_type: int, data: bytes = bytes()):
        if len(data) > 0xFF:
            raise RuntimeError("Control packet too long (length must fit into 8 bits)")
        self.clientsocket.sendall(bytearray([msg_type, len(data)]) + data)

    def sendKeypress(self, keypress: int):
        self._sendControlPacket(self.CTRL_TYPE_KEYPRESS, bytes([keypress]))

    def sendSetQuality(self, quality: int):
        self._sendControlPacket(self.CTRL_TYPE_QUALITY, bytes([quality]))

    def sendSetMaxDim(self, max_dimension: int):
        self._sendControlPacket(self.CTRL_TYPE_MAX_DIMENSION, struct.pack('!H', max_dimension))

    def sendStreamSelect(self, stream_id: int):
        self._sendControlPacket(self.CTRL_TYPE_STREAM_SELECT, bytes([stream_id]))

    def requestStreamEnable(self):
        self._sendControlPacket(self.CTRL_TYPE_ENABLE)

def main():
    import numpy as np
    import cv2

    imgsocket = CanmoreImageSocketListener('orin')
    target_width = 1280

    try:
        while True:
            # Using cv2.imshow() method
            # Displaying the image
            jpegdata = imgsocket.recvJpeg()
            if jpegdata is not None:
                # Decode the image
                imarr = np.frombuffer(jpegdata, np.uint8)
                contentarr = np.frombuffer(imarr, np.uint8)
                rxImg = cv2.imdecode(contentarr, cv2.IMREAD_COLOR)

                # Resize to make appear on screen okay
                (h, w) = rxImg.shape[:2]
                r = target_width / float(w)
                dim = (target_width, int(h * r))
                imgScaled = cv2.resize(rxImg, dim)

                # Show image
                cv2.imshow("Underwater Camera", imgScaled)

                # waits for user to press any key
                # (this is necessary to avoid Python kernel form crashing)
                key = cv2.waitKey(10)
                if key == ord('q'):
                    break
                elif key == ord('p'):
                    imgsocket.sendSetMaxDim(190)
                    imgsocket.sendSetQuality(25)
                elif key != -1:
                    imgsocket.sendKeypress(key)
            else:
                cv2.destroyAllWindows()
                imgsocket.requestStreamEnable()

    finally:
        # closing all open windows
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
