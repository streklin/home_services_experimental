import React, {Component} from 'react';
import RemoteCamera from '../../Components/RemoteCamera/RemoteCamera';
import RemoteControls from '../../Components/RemoteControl/RemoteControl';
import SLAM from '../../Components/SLAM/SLAM';
import AutoMap from '../../Components/AutoMap/AutoMap';
import { connect } from 'react-redux';
import * as actionCreators from '../../store/actionCreators';
import './RobotControls.css';
import openSocket from "socket.io-client";
import createDataUri from "create-data-uri";

export class RobotControls extends Component {

    state = {
        imageData: null,
        mapData: null
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
            <div className="RobotControls">
                <RemoteCamera
                    imageData={this.state.imageData}
                />
                <div className="Bottom">
                    <RemoteControls
                        token={this.props.token}
                    />
                    <SLAM
                        imageData={this.state.mapData}
                    />
                    <AutoMap
                        toggleAutoMap={this.props.toggleAutoMap}
                        isAutoMapActive={this.props.isAutoMapActive}
                    />
                </div>

            </div>
        );
    }
}

const mapStateToProps = (state) => {
    return {
        isAutoMapActive: state.appStore.robotState.isAutoMapActive,
        mapUrl: state.appStore.mapUrl,
        camUrl: state.appStore.camUrl,
        token: state.appStore.token
    }
};

const mapDispatchToProps = (dispatch) => {
    return {
        toggleAutoMap: () => dispatch(actionCreators.toggleAutoMap())
    }
};


export default connect(mapStateToProps, mapDispatchToProps)(RobotControls);