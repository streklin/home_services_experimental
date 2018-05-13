import React from 'react';
import './BatteryBar.css';

const batteryBar = (props) => {
    let powerLevelStyle = {
        width: props.power + "%"
    };

    let powerLevelClasses = ['powerLevel'];

    if (props.power < 25.0) {
        powerLevelClasses.push('bad');
    } else {
        powerLevelClasses.push('good');
    }

    return (
        <div className="BatteryBar">
            <div style={powerLevelStyle} className={powerLevelClasses.join(' ')}></div>
        </div>
    );
};

export default batteryBar;