import React, {Component} from 'react';
import {BrowserRouter, Route} from 'react-router-dom';
import SideMeu from '../../Components/SideMenu/sideMenu';
import RobotControls from '../RobotControls/RobotControls';
import ChatBot from '../ChatBot/ChatBot';
import RobotStatus from '../RobotStatus/RobotStatus';
import Telepresence from '../Telepresence/Telepresence';
import Login from '../Login/Login';
import AWS from 'aws-sdk';
import LoginRedirect from '../../Services/LoginRedirect/LoginRedirect';
import './App.css';

class App extends Component {

    connectToAWS = () => {
        AWS.config.region = 'us-east-1'; // Region
        AWS.config.credentials = new AWS.CognitoIdentityCredentials({
            IdentityPoolId: 'us-east-1:4f6c8ea3-1716-48ac-8d9e-6303dcd3da30',
        });
    };

    render() {

        this.connectToAWS();

        return (
            <BrowserRouter>
                <div className="App">
                    <SideMeu/>
                    <div className="container">

                        <Route
                            path="/"
                            exact
                            component={LoginRedirect(RobotControls)}
                        />

                        <Route
                            path="/chat"
                            exact
                            component={LoginRedirect(ChatBot)}
                        />

                        <Route
                            path="/status"
                            exact
                            component={LoginRedirect(RobotStatus)}
                        />

                        <Route
                            path="/telepresence"
                            exact
                            component={LoginRedirect(Telepresence)}
                        />

                        <Route
                            path="/login"
                            exact
                            component={Login}
                        />


                    </div>
                </div>
            </BrowserRouter>
        );
    }
}

export default App;
