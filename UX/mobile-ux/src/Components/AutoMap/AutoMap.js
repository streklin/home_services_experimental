import React from 'react';
import './AutoMap.css';


const autoMap = (props) => {

    let autoMapClasses = ['AutoMap'];

    if (props.isAutoMapActive) autoMapClasses.push('active');

    return (
        <div onClick={props.toggleAutoMap} className={autoMapClasses.join(" ")}>
            <span>Auto Map</span>
        </div>
    );
};

export default autoMap;