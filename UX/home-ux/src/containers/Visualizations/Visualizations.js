import React, { Component } from 'react';
import axios from 'axios';
import {SERVER_URL} from '../../settings';
import './Visualizations.css';

const baseUrl = SERVER_URL;

class Visualizations extends Component {

    state = {
        mapUrl: baseUrl + "robot/images/map",
        isAutoMapActive: false
    };

    componentDidMount() {
        this.interval = setInterval(()=> {
            let newState = Object.assign({}, this.state);
            newState.mapUrl = baseUrl + "robot/images/map?t=" + new Date().getTime();
            this.setState(newState);
        }, 1000);
    }

    callForward = () => {
        axios.get(baseUrl + "robot/drive/forward");
    };

    callBackward = () => {
        axios.get(baseUrl + "robot/drive/backward");
    };

    callLeft = () => {
        axios.get(baseUrl + "robot/drive/left");
    };

    callRight = () => {
        axios.get(baseUrl + "robot/drive/right");
    };

    callStop = () => {
        axios.get(baseUrl + "robot/drive/stop");
    };

    toggleAutoMap = () => {
        let newState = Object.assign({}, this.state);
        newState.isAutoMapActive = !newState.isAutoMapActive;
        this.setState(newState);
        axios.get(baseUrl + "robot/state/toggleAutoMap");
    };

    render() {

        let autoMapClasses = ['autoDrive'];

        if (this.state.isAutoMapActive) {
            autoMapClasses.push("green");
        } else {
            autoMapClasses.push("red");
        }

        return (
            <div className="Visualizations">
                <div className="remote-controls">

                    <div className="controls-container">
                        <div onClick={this.callForward} className="remote-up remote-button">
                            <i className="fas fa-caret-up"></i>
                        </div>

                        <div className="remote-middle remote-button">
                            <div onClick={this.callLeft} className="remote-left remote-button">
                                <i className="fas fa-caret-left"></i>
                            </div>

                            <div onClick={this.callStop} className="remote-stop remote-button">
                                <i className="fas fa-circle"></i>
                            </div>

                            <div onClick={this.callRight} className="remote-right remote-button">
                                <i className="fas fa-caret-right"></i>
                            </div>
                        </div>

                        <div onClick={this.callBackward} className="remote-down remote-button">
                            <i className="fas fa-caret-down"></i>
                        </div>

                    </div>

                </div>
                <div className="mapOut">
                    <div onClick={this.toggleAutoMap} className={autoMapClasses.join(" ")}>
                        <span>Auto Map</span>
                    </div>
                    <div className="takePicture">
                        CAM
                    </div>
                    <img src={this.state.mapUrl} alt="Occupancy Map" />
                </div>
            </div>
        )
    }
}

export default Visualizations;