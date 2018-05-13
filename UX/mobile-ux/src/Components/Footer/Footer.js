import React, { Component } from 'react';
import './Footer.css';
import foxwellRoboticsLogo from '../../assets/foxwellRoboticsLogo.png';

class Footer extends Component {
    render() {
        return (
            <div className="Footer">
                <div className="column">
                    <img src={foxwellRoboticsLogo} alt="Foxwell Robotics" />
                </div>
                <div className="column">
                    <div className="contactInfo">
                        <p>Foxwell Robotics</p>
                        <p>gene.foxwell@gmail.com</p>
                    </div>
                </div>
            </div>
        )
    }
}

export default Footer;