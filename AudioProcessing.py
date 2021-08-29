import atexit
import pyaudio
import time
import wave

# You need to have all of the dependencies installed on your machine for this to work
# either use Anaconda or pip to install (preferably in a virtual environment or conda environment)

# Useful tutorial: https://publish.illinois.edu/augmentedlistening/tutorials/music-processing/tutorial-1-introduction-to-audio-processing-in-python/

CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 44100

def save_audio_files(streams, frames):
    for i in range(len(streams)):
        wave_output_filename = "{}_audio_output_{}.wav".format(i, int(time.time()))
        wf = wave.open(wave_output_filename, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames[i]))
        wf.close()

def stop_streams(streams):
    for stream in streams:
        stream.stop_stream()
        stream.close()
    p.terminate()

def onexit(streams, frames):
    print("Saving audio output. DO NOT INTERRUPT!")
    save_audio_files(streams, frames)
    stop_streams(streams)
    
if __name__ == "__main__":
    p = pyaudio.PyAudio()

    streams = []
    num_devices = p.get_device_count()
    print(f'num found devices {num_devices}')
    mics = []
    for i in range(num_devices):
        # try:
        #     stream = p.open(format=FORMAT,
        #                     channels=CHANNELS,
        #                     rate=RATE,
        #                     input=True,
        #                     input_device_index=i)
        #     streams.append(stream)
        #     mics.append(i)
        # except:
        #     print("Failed to use mic {}".format(i))

        device_info = p.get_device_info_by_index(i)
        print(device_info)
        device_max_channels = device_info['maxInputChannels']
        device_host_api = device_info['hostApi']
        if 'USB' in  device_info['name'] and device_max_channels == 1:
            stream = p.open(format=FORMAT,
                            channels=1,
                            rate=RATE,
                            input=True,
                            input_device_index=i)
            streams.append(stream)
            mics.append(i)

    num_used_mics = len(mics)
    frames = [[] for i in range(num_used_mics)]
    print("started recording")
    print(f'number microphones {len(mics)}')
    print(f'number streams {len(streams)}')
    
    atexit.register(lambda: onexit(streams, frames))
    while True:
        for i in range(num_used_mics):
            stream = streams[i]
            data = stream.read(CHUNK, exception_on_overflow=False)
            frames[i].append(data)