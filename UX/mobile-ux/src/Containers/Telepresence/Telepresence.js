import React, { Component } from 'react';
import { connect } from 'react-redux';
import RemoteCamera from '../../Components/RemoteCamera/RemoteCamera';
import RemoteControl from '../../Components/RemoteControl/RemoteControl';
import Webcam from 'react-webcam';
import SLAM from '../../Components/SLAM/SLAM';
import './Telepresence.css';
import openSocket from 'socket.io-client';
import createDataUri from 'create-data-uri';

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

    componentDidMount() {

        const socket = openSocket('http://localhost:8000');
        socket.on('robot-camera', (data) => {
            let newState = Object.assign({}, this.state);
            newState.imageData = createDataUri("image/jpeg", data);
            this.setState(newState);
        });

        socket.on('robot-map', (data) => {
            let newState = Object.assign({}, this.state);
            newState.mapData = createDataUri("image/jpeg", data);
            this.setState(newState);
        });

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