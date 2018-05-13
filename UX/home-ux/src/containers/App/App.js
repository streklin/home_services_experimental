import React, {Component} from 'react';
import RobotControls from '../RobotControls/RobotControls';
import RobotSensors from '../RobotSensors/RobotSensors';
import ChatBot from '../ChatBot/ChatBot';
import {BrowserRouter, Route} from 'react-router-dom';
import AWS from 'aws-sdk';
import './App.css';

class App extends Component {
    render() {

        AWS.config.region = 'us-east-1'; // Region
        AWS.config.credentials = new AWS.CognitoIdentityCredentials({
            IdentityPoolId: 'us-east-1:4f6c8ea3-1716-48ac-8d9e-6303dcd3da30',
        });

        return (

        <BrowserRouter>
            <div className="App">
                <Route
                    path="/"
                    exact
                    component={RobotControls}
                />
                <Route
                    path="/robotSensors"
                    exact
                    component={RobotSensors}
                />
                <Route
                    path="/chatBot"
                    exact
                    component={ChatBot}
                />
            </div>
        </BrowserRouter>

    )
        ;
    }
}

export default App;
