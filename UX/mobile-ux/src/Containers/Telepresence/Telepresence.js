import React, { Component } from 'react';
import { connect } from 'react-redux';
import RemoteCamera from '../../Components/RemoteCamera/RemoteCamera';
import RemoteControl from '../../Components/RemoteControl/RemoteControl';
import Webcam from 'react-webcam';
import SLAM from '../../Components/SLAM/SLAM';
import './Telepresence.css';
import openSocket from 'socket.io-client';

export class Telepresence extends Component {


    setRef = (webcam) => {
        this.webcam = webcam;
    };

    capture = () => {
        const imageSrc = this.webcam.getScreenshot();
        return imageSrc;
    };

    componentDidMount() {
        const  socket = openSocket('http://localhost:8000');

        setInterval(() => {
            socket.emit('receiveImage', this.capture());
        }, 100);

    }

    render() {
        return (
            <div className="Telepresence">

                <div className="camera">
                    <RemoteCamera
                        camUrl={this.props.camUrl}
                    />
                </div>

                <div className="controls">
                    <RemoteControl/>
                    <div className="webCam">
                        <Webcam
                            height={200}
                            width={200}
                            ref={this.setRef}
                            screenshotFormat="image/jpeg"
                        />
                    </div>

                    <SLAM
                        mapImage={this.props.mapUrl}
                    />
                </div>

            </div>
        );
    }
}


const mapStateToProps = (state) => {
    return {
        mapUrl: state.appStore.mapUrl,
        camUrl: state.appStore.camUrl
    }
};


export default connect(mapStateToProps)(Telepresence);