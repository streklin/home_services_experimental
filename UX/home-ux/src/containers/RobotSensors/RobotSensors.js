import React, { Component } from 'react';
import axios from 'axios';
import BatteryBar from '../../components/BatteryBar/BatteryBar';
import CreateSensors from '../../components/CreateSensors/CreateSensors';
import Footer from '../Footer/Footer';
import './RobotSensors.css';
import SERVER_URL from '../../settings';

const baseUrl = SERVER_URL;

class RobotSensors extends Component {

    state = {
        isAutoMapActive: false,
        leftBumperPressed: false,
        rightBumperPressed: false,
        wheelsDropped: false,
        spotLightOn: false,
        debrisLightOn: false,
        isLeftSensorActive: false,
        isFrontLeftSensorActive: false,
        isCenterLeftSensorActive: false,
        isCenterRightSensorActive: false,
        isFrontRightSensorActive: false,
        isRightSensorActive: false,
        batteryPower: 0.0
    };

    componentDidMount() {
        this.interval = setInterval(()=> {
            axios.get(baseUrl + "robot/state/getState")
                .then((response) => {
                    this.setState(response.data);
                }
            );

        }, 250);
    }

    render() {

        return (
            <div className="RobotSensors">
                <CreateSensors
                    sensors={this.state}
                />
                <BatteryBar
                    power={this.state.batteryPower}
                />
                <Footer />
            </div>
        );
    }
}

export default RobotSensors;