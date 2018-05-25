from sys import byteorder
from array import array
from struct import pack
import base64
import requests

import pyaudio
import wave

THRESHOLD = 500
CHUNK_SIZE = 1024
FORMAT = pyaudio.paInt16
RATE = 44100
MAX_SILENCE = 10
MAXIMUM = 16384

TRAINING_1 = "training_1.wav"
TRAINING_2 = "training_2.wav"
TRAINING_3 = "training_3.wav"

MODEL_FILE = "model.umdl"

endpoint = "https://snowboy.kitt.ai/api/v1/train/"

def get_wave(fname):
    with open(fname) as infile:
        return base64.b64encode(infile.read())

class ModelCreator():
    def __init__(self):
        self.token = "d904bff0833ad7c80f868bdea518b3540461a2b2"
        self.hotword_name = "MARVIN_BOT"
        self.language = "en"
        self.age_group = "20_29"
        self.gender = "M"
        self.microphone = "laptop microphone"

    def get_model(self):
        data = {
            "name": self.hotword_name,
            "language": self.language,
            "age_group": self.age_group,
            "gender": self.gender,
            "microphone": self.microphone,
            "token": self.token,
            "voice_samples": [
                {"wave": get_wave(TRAINING_1)},
                {"wave": get_wave(TRAINING_2)},
                {"wave": get_wave(TRAINING_3)}
            ]
        }

        response = requests.post(endpoint, json=data)

        if response.ok:
            with open(MODEL_FILE, "w") as outfile:
                outfile.write(response.content)
            print "Saved model to '%s'." % MODEL_FILE
        else:
            print "Request failed."
            print response.text

class WaveCreator():
    def __init__(self):
        pass

    def is_silent(self, snd_data):
        return max(snd_data) < THRESHOLD

    def save_recording(self, data, sample_width, path):
        data = pack('<' + ('h' * len(data)), *data)

        wf = wave.open(path, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(sample_width)
        wf.setframerate(RATE)
        wf.writeframes(data)
        wf.close()

    def normalize(self, snd_data):
        weight = float(MAXIMUM) / max(abs(i) for i in snd_data)

        normalized_recording = array('h')
        for i in snd_data:
            normalized_recording.append(int(i * weight))
        return normalized_recording

    def record_training_example(self, path):

        # create an instance of pyaudio
        p = pyaudio.PyAudio()

        # create a stream
        stream = p.open(format=FORMAT, channels=1, rate=RATE,
                        input=True, output=True,
                        frames_per_buffer=CHUNK_SIZE)

        # listen to the stream
        is_recording = True
        silent_count = 0

        recording = array('h')

        print("Start Recording")

        while is_recording:
            # read in the next chunk
            snd_data = array('h', stream.read(CHUNK_SIZE))
            if byteorder == 'big':
                snd_data.byteswap()

            # was it silent?
            if self.is_silent(snd_data):
                silent_count = silent_count + 1

                if silent_count > MAX_SILENCE:
                    is_recording = False

                continue

            # append next chunk to the recording
            recording.extend(snd_data)

        # close the stream and get associated sampling data
        sample_width = p.get_sample_size(FORMAT)
        stream.stop_stream()
        stream.close()
        p.terminate()

        print ("End Recording")

        # normalize the recording
        recording = self.normalize(recording)

        # save the recording
        self.save_recording(recording, sample_width, path)

if __name__ == '__main__':
    print("Starting Audio Recorder")

    waveCreator = WaveCreator()
    waveCreator.record_training_example(TRAINING_1)
    waveCreator.record_training_example(TRAINING_2)
    waveCreator.record_training_example(TRAINING_3)

    print ('Creating Model')
    modelCreator = ModelCreator()
    modelCreator.get_model()



