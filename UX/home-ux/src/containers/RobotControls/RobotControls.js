import React, { Component } from 'react';
import Footer from '../Footer/Footer';
import RemoteCamera from '../RemoteCamera/RemoteCamera';
import Visualizations from '../Visualizations/Visualizations';

class RobotControls extends Component {
    render() {

        return (
            <div>
                <RemoteCamera/>
                <Visualizations />
                <Footer />
            </div>
        );
    }
}

export default RobotControls;