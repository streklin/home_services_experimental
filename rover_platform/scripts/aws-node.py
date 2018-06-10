#!/usr/bin/env python
import os
from sys import byteorder
from array import array
from struct import pack
import time
import requests
from contextlib import contextmanager
import pyaudio
import wave
import rospy
from std_msgs.msg import String
import boto3
import uuid
import os

lex_client = boto3.client('lex-runtime')
polly_client = boto3.client('polly')

THRESHOLD = 10000
CHUNK_SIZE = 1024
FORMAT = pyaudio.paInt16
RATE = 16000
MAX_SILENCE = 2
MAXIMUM = 200

TOP_DIR = os.path.dirname(os.path.abspath(__file__))
DETECT_DING = os.path.join(TOP_DIR, "ding.wav")

USERNAME = os.environ["ROS_USERNAME"]
PASSWORD = os.environ["ROS_PASSWORD_INTERNAL"]
TOKEN = None

# modified version of the WaveCreator class used for training SnowBoy
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

    def record_request(self):

        rospy.loginfo("Start Recording")

        # create an instance of pyaudio
        p = pyaudio.PyAudio()

        # create a stream
        stream = p.open(format=FORMAT, channels=1, rate=RATE,
                        input=True, output=True,
                        frames_per_buffer=CHUNK_SIZE)

        # listen to the stream
        is_recording = True
        silent_count = 0
        length = 0

        recording = array('h')

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
            else:
                silent_count = 0

            # append next chunk to the recording
            recording.extend(snd_data)

            # check for a maximum length
            length = length + 1

            if length > MAXIMUM:
                is_recording = False

        # close the stream and get associated sampling data
        sample_width = p.get_sample_size(FORMAT)
        stream.stop_stream()
        stream.close()
        p.terminate()


        # normalize the recording
        # recording = self.normalize(recording)

        rospy.loginfo("Recording Complete")

        # save the recording
        self.save_recording(recording, sample_width, 'data.wav')


# adapted from the snow boy source code.
@contextmanager
def no_alsa_error():
    try:
        asound = cdll.LoadLibrary('libasound.so')
        asound.snd_lib_error_set_handler(c_error_handler)
        yield
        asound.snd_lib_error_set_handler(None)
    except:
        yield
        pass

def play_audio_file(fname=DETECT_DING):
    ding_wav = wave.open(fname, 'rb')
    ding_data = ding_wav.readframes(ding_wav.getnframes())
    with no_alsa_error():
        audio = pyaudio.PyAudio()
    stream_out = audio.open(
        format=audio.get_format_from_width(ding_wav.getsampwidth()),
        channels=ding_wav.getnchannels(),
        rate=ding_wav.getframerate(), input=False, output=True)
    stream_out.start_stream()
    stream_out.write(ding_data)
    time.sleep(0.2)
    stream_out.stop_stream()
    stream_out.close()
    audio.terminate()

def play_sound(data):
    audio = pyaudio.PyAudio()

    stream = audio.open(
        format=audio.get_sample_size(FORMAT),
        channels=1,
        rate=8000,
        input=False,
        output=True
    )

    stream.start_stream()
    stream.write(data)

    time.sleep(2.0)

    stream.close()
    audio.terminate()

def speech_to_text(text):
    response = polly_client.synthesize_speech(
        OutputFormat="pcm",
        SampleRate="16000",
        Text=text,
        VoiceId="Matthew"
    )

    audio_data = response['AudioStream'].read()
    play_sound(audio_data)

def hotword_detection_callback(data):

    # make a "ding" noise to let the user know that listening is active
    play_audio_file()

    # when message is received record a request
    WAVE_CREATOR.record_request()

    # send the request to aws
    bot_name = "FoxwellRobotPrototype"
    bot_alias = "$LATEST"
    user_id = uuid.uuid4().hex
    content_type = 'audio/l16; rate=16000; channels=1'
    accept="audio/pcm"

    wav_file = open('data.wav', 'rb')

    response = lex_client.post_content(
        botName=bot_name,
        botAlias=bot_alias,
        userId=user_id,
        contentType=content_type,
        accept=accept,
        inputStream=wav_file
    )

    # play the resulting audio stream
    audio_data = response['audioStream'].read()
    play_sound(audio_data)

    # print the response for debugging purposes
    print (response)


    # was the request fullfilled?
    if (response['dialogState'] != 'Fulfilled'):
        return

    # pass the intent on to the middleware
    api_request = {
        "intent": response['intentName'],
        "variables": response['slots']
    }

    api_headers = {
        "Authorization": "Bearer " + TOKEN
    }

    api_response = requests.post("http://localhost:8080/robot/chat/lex", json=api_request, headers=api_headers)

    # do we need to output the robots reponse?
    response_text = api_response.text
    if response_text == "OK":
        return

    # use polly to provide feedback
    speech_to_text(response_text)

 # initialize the wave creator
WAVE_CREATOR = WaveCreator()

#gets the jwt login token
def get_login_token():
    global TOKEN

    login_request = {
        "username": USERNAME,
        "password": PASSWORD
    }

    login_response = requests.post("http://localhost:8080/robot/login", json=login_request)

    json_response = login_response.json()

    if json_response["status"] == "ERROR":
        print("Unable to login to middleware")

    TOKEN = json_response["token"]


def aws_node():

    # login in to the middleware
    get_login_token()

    # initialize the ROS node
    rospy.init_node('aws_voice_lex', anonymous=True)

    # need to listen for the hotword message from snowboy
    rospy.Subscriber('/hotword_detection', String, hotword_detection_callback)

    # let the user know that the node is ready to go.
    rospy.loginfo("AWS Node started ...")

    rospy.spin()


if __name__ == '__main__':
    try:
        aws_node()
    except rospy.ROSInterruptException:
        pass