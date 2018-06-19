import React, { Component } from 'react';
import { connect } from 'react-redux';
import RemoteCamera from '../../Components/RemoteCamera/RemoteCamera';
import RemoteControl from '../../Components/RemoteControl/RemoteControl';
import Webcam from 'react-webcam';
import SLAM from '../../Components/SLAM/SLAM';
import './Telepresence.css';
import openSocket from 'socket.io-client';
import createDataUri from 'create-data-uri';
import MicStreamer from '../../Components/MicStreamer/MicStreamer';
import {BASE_SOCKET} from "../../Services/config";

export class Telepresence extends Component {

    state = {
        imageData: null,
        mapData: null
    };

    constructor() {
        super();
        this.socket = openSocket(BASE_SOCKET);

        this.inverval = setInterval(() => {
            let imageData = this.capture();
            this.socket.emit('webcamImage', imageData);
        }, 100);
    }

    setRef = (webcam) => {
        this.webcam = webcam;
    };

    capture = () => {
        const imageSrc = this.webcam.getScreenshot();
        return imageSrc;
    };

    componentDidMount() {


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

    }

    componentWillUnmount() {
        this.interval();
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
                <MicStreamer
                    socket={this.socket}
                    broadcastTopic="clientPCMAudioChunk"
                />

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