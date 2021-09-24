import os
import socket
import struct
import time

import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
from tensorflow import keras

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

filepath = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'neural.h5')
print('FILEPATH', filepath)

model = keras.models.load_model(filepath)

def main():
    s.connect(('localhost', 50000))
    predictions = [[0,0,0,0,0,0]]
    while True:
        time.sleep(0.1)

        data_out = struct.pack('<6f', *predictions[0])
        s.sendall(data_out)

        data_in = s.recv(1024)
        features = struct.unpack('<2f', data_in)

        predictions = model.predict([features])


if __name__ == "__main__":
    try:
        main()
    finally:
        s.close()