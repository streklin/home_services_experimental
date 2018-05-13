import React, {Component} from 'react';
import RemoteCamera from '../../Components/RemoteCamera/RemoteCamera';
import RemoteControls from '../../Components/RemoteControl/RemoteControl';
import SLAM from '../../Components/SLAM/SLAM';
import AutoMap from '../../Components/AutoMap/AutoMap';
import { connect } from 'react-redux';
import * as actionCreators from '../../store/actionCreators';
import './RobotControls.css';


export class RobotControls extends Component {

    componentDidMount() {
        this.cameraInterval = setInterval(() => this.props.updateCameraUrl(), 1000);
        this.mapInterval = setInterval(() => this.props.updateMapUrl(), 1000);
    }

    render() {
        return (
            <div className="RobotControls">
                <RemoteCamera
                    camUrl={this.props.camUrl}
                />
                <div className="Bottom">
                    <RemoteControls/>
                    <SLAM
                        mapImage={this.props.mapUrl}
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
        camUrl: state.appStore.camUrl
    }
};

const mapDispatchToProps = (dispatch) => {
    return {
        updateCameraUrl: () => dispatch(actionCreators.updateCameraImg()),
        updateMapUrl: () => dispatch(actionCreators.updateMapImg()),
        toggleAutoMap: () => dispatch(actionCreators.toggleAutoMap())
    }
};


export default connect(mapStateToProps, mapDispatchToProps)(RobotControls);