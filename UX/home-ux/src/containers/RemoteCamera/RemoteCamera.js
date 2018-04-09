import React, { Component } from 'react';
import './RemoteCamera.css';

const baseUrl = "http://192.168.0.13:8080/";

class RemoteCamera extends Component {

    state = {
        camUrl: "robot/images/camera"
    };

    componentDidMount() {
        this.interval = setInterval(()=> {
            let newState = Object.assign({}, this.state);
            newState.camUrl = baseUrl + "robot/images/camera?t=" + new Date().getTime();
            this.setState(newState);
        }, 250);
    }

    render() {
        return (
            <div className="RemoteCamera">
                <img src={this.state.camUrl} alt="Robot Camera Image" />
            </div>
        )
    }
}

export default RemoteCamera;