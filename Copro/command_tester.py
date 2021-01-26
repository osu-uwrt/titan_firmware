import socket
import array
import sys
from time import sleep

if '-h' in sys.argv or 'help' in sys.argv:
    print('Run this program to test individual commands with different arguments.')
    print('Use the --local flag to connect to the simulator.')
    print('Argument lists accept a list of comma separated bytes.')
    print('Ex: args -> 1, 2, 100, 254, 255')
    exit()
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    if '--local' in sys.argv:
        print('Connecting to the simulator')
        s.connect(('127.0.0.1', 50000))
        print('Connected')
    else:
        print('Connecting to robot. Use the --local flag to connect to the simulator')
        s.connect(('192.168.1.42', 50000))
        print('Connected')
except KeyboardInterrupt:
    print('')
    exit()

while True:
    try:
        command = int(input("command number -> "))
        args = input("args -> ")
        args = str(args).replace(" ", "").split(", ")
        if "" in args:
            args.remove("")
        args_array = [int(arg) for arg in args]
        args_array.insert(0, len(args_array) + 2)
        args_array.insert(1, command)
        byte_array = array.array('B', args_array).tobytes()
        print(byte_array)
        s.sendall(byte_array)
        resp = s.recv(1024)
        resp = [r for r in resp]
        print('The response is: {}'.format(resp))

    except (KeyboardInterrupt, Exception) as e:
        print(e)
        break

print('Closing')
s.sendall(bytearray([0]))
s.close()