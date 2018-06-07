import React, { Component } from 'react';
import CreateSensors from '../../Components/CreateSensors/CreateSensors';
import BatteryBar from '../../Components/BatteryBar/BatteryBar';
import { connect } from 'react-redux';
import './RobotStatus.css';

export class RobotStatus extends Component {
    render() {
        return (
            <div className="RobotStatus">
                <CreateSensors
                    sensors={this.props.sensors}
                />
                <BatteryBar
                    power={this.props.sensors.batteryPower}
                />
            </div>
        );
    }
}

const mapStateToProps = (state) => {
    return {
        sensors: state.appStore.robotState
    }
};

export default connect(mapStateToProps)(RobotStatus);