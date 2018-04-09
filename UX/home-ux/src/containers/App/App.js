import React, { Component } from 'react';
import RobotControls from '../RobotControls/RobotControls';
import RobotSensors from '../RobotSensors/RobotSensors';
import {BrowserRouter, Route} from 'react-router-dom';
import './App.css';

class App extends Component {
  render() {
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
            </div>
        </BrowserRouter>

    );
  }
}

export default App;
