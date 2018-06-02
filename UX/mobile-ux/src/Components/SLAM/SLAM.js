import React from 'react';
import './SLAM.css';

const slam = (props) => {
    return (
        <div className="SLAM">
            <img src={props.imageData} alt="Local Map" />
        </div>
    );
};

export default slam;