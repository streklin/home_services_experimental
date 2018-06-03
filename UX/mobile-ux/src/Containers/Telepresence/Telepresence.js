import React, { Component } from 'react';
import { connect } from 'react-redux';
import RemoteCamera from '../../Components/RemoteCamera/RemoteCamera';
import RemoteControl from '../../Components/RemoteControl/RemoteControl';
import Webcam from 'react-webcam';
import SLAM from '../../Components/SLAM/SLAM';
import './Telepresence.css';
import openSocket from 'socket.io-client';
import createDataUri from 'create-data-uri';

let mediaRecorder = null;

export class Telepresence extends Component {

    state = {
        imageData: null,
        mapData: null
    };

    setRef = (webcam) => {
        this.webcam = webcam;
    };

    capture = () => {
        const imageSrc = this.webcam.getScreenshot();
        return imageSrc;
    };

    startMicrophone = () => {
        let constraints = {
            audio: true,
            video: false
        };

        navigator
            .mediaDevices
            .getUserMedia(constraints)
            .then((stream) => {
                console.log("GOT STREAM");
                let audioContext = window.AudioContext;
                let context = new audioContext();

                let audioInput = context.createMediaStreamSource(stream);
                let bufferSize = 2048;

                let recorder = context.createScriptProcessor(bufferSize, 1, 1);

                recorder.onaudioprocess = (event) => {
                    let left = event.inputBuffer.getChannelData(0);
                    this.socket.emit('pcmAudioChunk', left);
                };

                audioInput.connect(recorder);

                recorder.connect(context.destination);
            })
            .catch((err) => {
                console.log("ERROR: ", err);
            });
    };

    componentDidMount() {

        this.socket = openSocket('http://localhost:8000');
        this.socket.on('robot-camera', (data) => {
            let newState = Object.assign({}, this.state);
            newState.imageData = createDataUri("image/jpeg", data);
            this.setState(newState);
        });

        this.socket.on('robot-map', (data) => {
            let newState = Object.assign({}, this.state);
            newState.mapData = createDataUri("image/jpeg", data);
            this.setState(newState);
        });

        this.inverval = setInterval(() => {
            let imageData = this.capture();
            this.socket.emit('webcamImage', imageData);
        }, 100);

        this.startMicrophone();


    }

    render() {
        return (
            <div className="Telepresence">

                <div className="camera">
                    <RemoteCamera
                        imageData={this.state.imageData}
                    />
                </div>

                <div className="controls">
                    <RemoteControl
                        token={this.props.token}
                    />
                    <div className="webCam">
                        <Webcam
                            height={200}
                            width={200}
                            ref={this.setRef}
                            screenshotFormat="image/jpeg"
                        />
                    </div>

                    <SLAM
                        imageData={this.state.mapData}
                    />
                </div>

            </div>
        );
    }
}


const mapStateToProps = (state) => {
    return {
        token: state.appStore.token
    }
};


export default connect(mapStateToProps)(Telepresence);