import React from 'react';
import './AutoMap.css';


const autoMap = (props) => {

    let autoMapClick = () => {
        if (props.isAutoMapActive) {
            props.deactivate();
        } else {
            props.activate();
        }
    };

    let autoMapClasses = ['AutoMap'];

    if (props.isAutoMapActive) autoMapClasses.push('active');

    return (
        <div onClick={autoMapClick} className={autoMapClasses.join(" ")}>
            <span>Auto Map</span>
        </div>
    );
};

export default autoMap;