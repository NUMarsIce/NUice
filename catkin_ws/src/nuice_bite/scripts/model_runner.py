import socket
import struct
import time

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def main():
    s.connect(('localhost', 50000))
    
    while True:
        time.sleep(0.1)

        data = [0.0, 1.0, 1.0, 0.0]
        data_out = struct.pack('<4f', *data)
        s.sendall(data_out)

        data_in = s.recv(1024)
        features = struct.unpack('<4f', data_in)



if __name__ == "__main__":
    try:
        main()
    finally:
        s.close()