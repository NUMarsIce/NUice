import pyaudio
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
        wave_output_filename = "{}_audio_output.wav".format(i)
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
        if device_max_channels > 0 and device_host_api == 0:
            stream = p.open(format=FORMAT,
                            channels=device_max_channels,
                            rate=RATE,
                            input=True,
                            input_device_index=i)
            streams.append(stream)
    
    num_used_mics = len(mics)
    frames = [[] for i in range(num_used_mics)]
    print("started recording")
    print(f'number microphones {len(mics)}')
    print(f'number streams {len(streams)}')
    while True:
        try:
            for i in range(num_used_mics):
                stream = streams[i]
                data = stream.read(CHUNK)
                frames[i].append(data)

        except KeyboardInterrupt:
        # except (KeyboardInterrupt, GeneratorExit, SystemExit, InterruptedError) as e:
            # save audio files
            save_audio_files(streams, frames)
            stop_streams(streams)
            break