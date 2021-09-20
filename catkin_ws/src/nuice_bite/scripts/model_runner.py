import os
import socket
import struct
import time
from tensorflow import keras
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
filepath = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'neural_model')
model = keras.models.load_model(filepath)

def main():
    s.connect(('localhost', 50000))
    while True:
        time.sleep(0.1)

        data = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        data_out = struct.pack('<6f', *data)
        s.sendall(data_out)

        data_in = s.recv(1024)
        features = struct.unpack('<3f', data_in)

        model.predict([features])


if __name__ == "__main__":
    try:
        main()
    finally:
        s.close()